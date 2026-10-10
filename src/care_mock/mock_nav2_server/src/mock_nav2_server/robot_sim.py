"""
Pure Python, ROS-agnostic robot motion + battery simulator for the mock Nav2 server.

Loads its graph from the same JSON the CareRobotics env uses and mirrors the motion
semantics of src/environment/robot/robot_simulator.py (without training-only fields:
EdgeTraversalRecord, congestion features, headway, multi-robot accounting).

Usage from the ROS node:
    sim = RobotSim.from_config(config_dict, initial_node_id=0)
    sim.set_path([0, 1, 2])         # node ids; first is the current node
    sim.update(dt)                  # called from the /odom timer at ~10 Hz
    state = sim.get_state()         # snapshot for /odom + battery publishing

Charging is implicit: when idle at a node whose type is in _charging_node_types
(default: 'storage', 'hub'), battery climbs at _charging_rate_per_sec.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple
import math


DEFAULT_CHARGING_NODE_TYPES: Set[str] = {"storage", "hub"}

_MAX_DT = 5.0  # cap dt to avoid runaway first-tick / executor stall


@dataclass(frozen=True)
class NodeRec:
    node_id: int
    x: float
    y: float
    name: str
    type: str  # 'storage' | 'recovery' | 'hub' | ...


@dataclass(frozen=True)
class EdgeRec:
    from_id: int
    to_id: int
    distance_m: float
    max_v_ms: float


class RobotSim:
    """Single-robot motion + battery simulator, graph-aware, clock-blind."""

    def __init__(
        self,
        nodes: Dict[int, NodeRec],
        edges: Dict[Tuple[int, int], EdgeRec],
        initial_node_id: int,
        battery_drain_per_sec: float = 0.0001,
        battery_drain_per_meter: float = 0.0005,
        charging_rate_per_sec: float = 0.005,
        charging_node_types: Optional[Set[str]] = None,
    ) -> None:
        if initial_node_id not in nodes:
            raise ValueError(f"initial_node_id={initial_node_id} not in nodes")

        self._nodes = nodes
        self._edges = edges  # bidirectional: both (a,b) and (b,a) keys present

        types = charging_node_types or DEFAULT_CHARGING_NODE_TYPES
        self._charging_node_ids: Set[int] = {
            nid for nid, n in nodes.items() if n.type in types
        }

        n0 = nodes[initial_node_id]
        self._x: float = n0.x
        self._y: float = n0.y
        self._yaw: float = 0.0
        self._linear_v: float = 0.0
        self._angular_v: float = 0.0  # always 0 — no in-place rotation in mock

        self._current_node_id: Optional[int] = initial_node_id
        self._current_edge: Optional[EdgeRec] = None
        self._edge_from: Optional[int] = None
        self._edge_to: Optional[int] = None
        self._edge_progress: float = 0.0
        self._path_queue: List[int] = []

        self._battery: float = 1.0
        self._battery_drain_per_sec = battery_drain_per_sec
        self._battery_drain_per_meter = battery_drain_per_meter
        self._charging_rate_per_sec = charging_rate_per_sec

    # ─── Construction from CareRobotics-style env config ────────────────────────

    @classmethod
    def from_config(
        cls,
        config: dict,
        initial_node_id: int = 0,
        battery_drain_per_sec: float = 0.0001,
        battery_drain_per_meter: float = 0.0005,
        charging_rate_per_sec: float = 0.005,
    ) -> "RobotSim":
        """
        Build a RobotSim from a parsed CareRobotics environment JSON.

        Expected node schema:  {"id": int, "type": str, "name": str, "pos": [x, y], ...}
        Expected edge schema:  {"from": int, "to": int, "distance_m": float,
                                "max_v_ms_normal": float, ...}

        Edges are made bidirectional — both (from, to) and (to, from) resolve
        to the same EdgeRec.
        """
        nodes: Dict[int, NodeRec] = {}
        for n in config.get("nodes", []):
            nid = int(n["id"])
            pos = n["pos"]
            nodes[nid] = NodeRec(
                node_id=nid,
                x=float(pos[0]),
                y=float(pos[1]),
                name=str(n.get("name", f"node_{nid}")),
                type=str(n.get("type", "")),
            )

        edges: Dict[Tuple[int, int], EdgeRec] = {}
        for e in config.get("edges", []):
            f = int(e["from"])
            t = int(e["to"])
            rec = EdgeRec(
                from_id=f,
                to_id=t,
                distance_m=float(e["distance_m"]),
                max_v_ms=float(e.get("max_v_ms_normal", 0.5)),
            )
            edges[(f, t)] = rec
            edges[(t, f)] = rec

        return cls(
            nodes=nodes,
            edges=edges,
            initial_node_id=initial_node_id,
            battery_drain_per_sec=battery_drain_per_sec,
            battery_drain_per_meter=battery_drain_per_meter,
            charging_rate_per_sec=charging_rate_per_sec,
        )

    # ─── Public path control + tick ────────────────────────────────────────────

    def set_path(self, node_id_path: List[int]) -> None:
        """
        Replace the path queue with a sequence of node ids.

        Convention: node_id_path[0] should be the robot's current node;
        the rest are targets in order. If [0] differs from the current
        node, we use the whole list as the queue (lenient mode) and
        traverse from wherever the robot currently is.

        An empty or single-element path leaves the robot idle.
        Calling this while mid-edge cancels the active traversal and
        starts the new path from the last fully-arrived-at node.
        """
        if not node_id_path or len(node_id_path) < 2:
            self._path_queue = []
            self._current_edge = None
            self._edge_from = None
            self._edge_to = None
            self._edge_progress = 0.0
            self._linear_v = 0.0
            return

        if (
            self._current_node_id is not None
            and node_id_path[0] == self._current_node_id
        ):
            self._path_queue = list(node_id_path[1:])
        else:
            self._path_queue = list(node_id_path)

        self._current_edge = None
        self._edge_progress = 0.0
        self._start_next_edge()

    def update(self, dt: float) -> None:
        """
        Advance state by `dt` real seconds.

        Three regimes:
          - Idle at charging node: battery climbs, no motion, no drain.
          - Idle elsewhere:        nothing.
          - On an edge:            position advances at edge.max_v_ms,
                                   battery drains by per-sec + per-meter terms.
        """
        if dt <= 0:
            return
        dt = min(dt, _MAX_DT)

        if self._current_edge is None and not self._path_queue:
            self._linear_v = 0.0
            if (
                self._current_node_id in self._charging_node_ids
                and self._battery < 1.0
            ):
                self._battery = min(
                    1.0, self._battery + self._charging_rate_per_sec * dt
                )
            return

        if self._current_edge is None:
            self._start_next_edge()
            if self._current_edge is None:
                return

        edge = self._current_edge
        if edge.distance_m <= 0:
            self._edge_progress = 1.0
            self._arrive_at_node(self._edge_to)
            return

        v = edge.max_v_ms
        distance_step = v * dt
        progress_delta = distance_step / edge.distance_m
        self._edge_progress = min(1.0, self._edge_progress + progress_delta)
        self._linear_v = v
        self._update_position()
        self._battery = max(
            0.0,
            self._battery
            - self._battery_drain_per_sec * dt
            - self._battery_drain_per_meter * distance_step,
        )

        if self._edge_progress >= 1.0:
            self._arrive_at_node(self._edge_to)

    # ─── Read-only state accessors ─────────────────────────────────────────────

    def get_state(self) -> Dict:
        """Snapshot for /odom + battery + action feedback."""
        return {
            "x": self._x,
            "y": self._y,
            "yaw": self._yaw,
            "linear_x": self._linear_v,
            "angular_z": self._angular_v,
            "battery": self._battery,
            "current_node_id": self._current_node_id,
            "edge_progress": self._edge_progress,
            "is_idle": self.is_idle(),
            "is_charging": (
                self.is_idle()
                and self._current_node_id in self._charging_node_ids
                and self._battery < 1.0
            ),
        }

    def is_idle(self) -> bool:
        return self._current_edge is None and not self._path_queue

    def distance_remaining(self) -> float:
        """Metres along the remaining path: current edge fragment + queued edges."""
        total = 0.0
        if self._current_edge is not None:
            total += self._current_edge.distance_m * (1.0 - self._edge_progress)
        prev = self._edge_to if self._current_edge is not None else self._current_node_id
        for next_id in self._path_queue:
            if prev is None:
                break
            edge = self._edges.get((prev, next_id))
            if edge is not None:
                total += edge.distance_m
            elif prev in self._nodes and next_id in self._nodes:
                a = self._nodes[prev]
                b = self._nodes[next_id]
                total += math.hypot(b.x - a.x, b.y - a.y)
            prev = next_id
        return total

    def current_node_id(self) -> Optional[int]:
        return self._current_node_id

    @property
    def battery(self) -> float:
        return self._battery

    @property
    def position(self) -> Tuple[float, float]:
        return (self._x, self._y)

    @property
    def nodes(self) -> Dict[int, NodeRec]:
        """Read-only access — mock_node uses this to snap (x,y) → nearest node."""
        return self._nodes

    # ─── Internal helpers ──────────────────────────────────────────────────────

    def _start_next_edge(self) -> None:
        """Pop next node from queue and begin traversing the connecting edge."""
        if not self._path_queue:
            self._current_edge = None
            self._edge_from = None
            self._edge_to = None
            self._edge_progress = 0.0
            self._linear_v = 0.0
            return

        target = self._path_queue.pop(0)
        from_id = self._current_node_id
        if from_id is None:
            if target in self._nodes:
                n = self._nodes[target]
                self._x, self._y = n.x, n.y
                self._current_node_id = target
            self._start_next_edge()
            return

        edge = self._edges.get((from_id, target))
        if edge is None:
            # Lenient: non-adjacent nodes — teleport to target and continue.
            if target in self._nodes:
                n = self._nodes[target]
                self._x, self._y = n.x, n.y
            self._current_node_id = target
            self._current_edge = None
            self._edge_progress = 0.0
            self._start_next_edge()
            return

        self._current_edge = edge
        self._edge_from = from_id
        self._edge_to = target
        self._edge_progress = 0.0

        a = self._nodes[from_id]
        b = self._nodes[target]
        self._yaw = math.atan2(b.y - a.y, b.x - a.x)
        self._linear_v = edge.max_v_ms

    def _arrive_at_node(self, node_id: Optional[int]) -> None:
        """Snap to a node's coordinates; either continue path or go idle."""
        if node_id is None or node_id not in self._nodes:
            self._current_edge = None
            return
        n = self._nodes[node_id]
        self._x, self._y = n.x, n.y
        self._current_node_id = node_id
        self._current_edge = None
        self._edge_from = None
        self._edge_to = None
        self._edge_progress = 0.0

        if self._path_queue:
            self._start_next_edge()
        else:
            self._linear_v = 0.0

    def _update_position(self) -> None:
        """Linear interpolation along the current edge by progress."""
        if self._current_edge is None:
            return
        a = self._nodes[self._edge_from]
        b = self._nodes[self._edge_to]
        p = self._edge_progress
        self._x = a.x + (b.x - a.x) * p
        self._y = a.y + (b.y - a.y) * p