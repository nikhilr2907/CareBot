"""
Pure Python, ROS-agnostic SKU inventory simulator for the mock Nav2 server.

Loads per-node inventory from the same JSON the CareRobotics env uses.
Each tick, drains stock at consumption_enabled nodes at a configurable
per-real-second rate. Deliveries (called from the action server's
goal-success handler) bump stock via restock().

Driven by the ROS node:
    sim = ConsumptionSim.from_config(config_dict, default_rate_per_sec=0.05)
    sim.tick(dt)                   — called from the inventory timer at ~1 Hz
    sim.restock(node_id, amount)   — called when the robot delivers
    payload = sim.snapshot()       — JSON-ready list for /sensor/sku_inventory

Wire format matches what src/environment/gapo_env_real.py _update_inventory reads:
    [{
        "location_id":   str(node.id),                    # MUST equal GraphNode.node_id
        "location_name": str,
        "sku_inventory": { sku_id: { stock_level, max_level, par_level,
                                     reorder_point, category } },
        "category_inventory": { category: { total_stock, max_stock, num_skus } }
    }, ...]

Only `stock_level` is read by the env; the other fields are pass-through for
protocol cleanness.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class SKUState:
    sku_id: str
    stock: float
    max_capacity: float
    par_level: float
    reorder_point: float
    category: str = ""


@dataclass
class NodeInventory:
    node_id: int
    name: str
    consumption_enabled: bool
    skus: Dict[str, SKUState] = field(default_factory=dict)


class ConsumptionSim:
    """Per-node SKU dynamics: time-based drain + delivery-driven restock."""

    def __init__(
        self,
        nodes: Dict[int, NodeInventory],
        consumption_rates: Dict[Tuple[int, str], float],
        default_restock_amount: int = 5,
    ) -> None:
        self._nodes = nodes
        self._consumption_rates = consumption_rates
        self._default_restock_amount = default_restock_amount

    # ─── Construction from CareRobotics env JSON ────────────────────────────────

    @classmethod
    def from_config(
        cls,
        config: dict,
        default_rate_per_sec: float = 0.05,
        default_restock_amount: int = 5,
    ) -> "ConsumptionSim":
        """
        Build from a parsed env config.

        Each consumption_enabled node's SKUs drain at default_rate_per_sec
        items per real second. Non-consumption nodes (storage, hub) keep
        their stock static unless explicitly restocked.
        """
        nodes: Dict[int, NodeInventory] = {}
        rates: Dict[Tuple[int, str], float] = {}

        for n in config.get("nodes", []):
            nid = int(n["id"])
            consumption_enabled = bool(n.get("consumption_enabled", False))
            inv = NodeInventory(
                node_id=nid,
                name=str(n.get("name", f"node_{nid}")),
                consumption_enabled=consumption_enabled,
            )
            for sku_id, sku_def in n.get("inventory", {}).items():
                inv.skus[sku_id] = SKUState(
                    sku_id=sku_id,
                    stock=float(sku_def.get("initial_stock", 0)),
                    max_capacity=float(sku_def.get("max_capacity", 0)),
                    par_level=float(sku_def.get("par_level", 0)),
                    reorder_point=float(sku_def.get("reorder_point", 0)),
                    category=str(sku_def.get("category", "")),
                )
                if consumption_enabled:
                    rates[(nid, sku_id)] = default_rate_per_sec
            nodes[nid] = inv

        return cls(
            nodes=nodes,
            consumption_rates=rates,
            default_restock_amount=default_restock_amount,
        )

    # ─── Tick + restock ────────────────────────────────────────────────────────

    def tick(self, dt: float) -> None:
        """Drain stock at all consumption_enabled SKUs by rate * dt."""
        if dt <= 0:
            return
        for (nid, sku_id), rate in self._consumption_rates.items():
            node = self._nodes.get(nid)
            if node is None:
                continue
            sku = node.skus.get(sku_id)
            if sku is None:
                continue
            sku.stock = max(0.0, sku.stock - rate * dt)

    def restock(
        self,
        node_id: int,
        amount: Optional[int] = None,
        sku_id: Optional[str] = None,
    ) -> None:
        """
        Bump stock at a node.

        - If sku_id is given, only that SKU is restocked by `amount`.
        - Otherwise, `amount` is split evenly across all SKUs at the node
          (so total items added matches `amount`, mirroring a single
          delivery of N items spread across the node's product mix).
        - Stock is capped at max_capacity per SKU.
        - If amount is None, falls back to default_restock_amount.
        """
        node = self._nodes.get(node_id)
        if node is None or not node.skus:
            return

        amt = float(amount if amount is not None else self._default_restock_amount)

        if sku_id is not None:
            sku = node.skus.get(sku_id)
            if sku is None:
                return
            sku.stock = min(sku.max_capacity, sku.stock + amt)
            return

        per_sku = amt / len(node.skus)
        for sku in node.skus.values():
            sku.stock = min(sku.max_capacity, sku.stock + per_sku)

    # ─── Snapshot for /sensor/sku_inventory ────────────────────────────────────

    def snapshot(self) -> List[Dict]:
        """JSON-ready list matching the bridge's expected wire format."""
        out: List[Dict] = []
        for nid, node in self._nodes.items():
            sku_inv: Dict[str, Dict] = {}
            cat_totals: Dict[str, List[float]] = {}  # category -> [total_stock, max_stock, num_skus]

            for sku_id, sku in node.skus.items():
                sku_inv[sku_id] = {
                    "stock_level": round(sku.stock, 2),
                    "max_level": sku.max_capacity,
                    "par_level": sku.par_level,
                    "reorder_point": sku.reorder_point,
                    "category": sku.category,
                }
                if sku.category:
                    bucket = cat_totals.setdefault(sku.category, [0.0, 0.0, 0])
                    bucket[0] += sku.stock
                    bucket[1] += sku.max_capacity
                    bucket[2] += 1

            cat_inv = {
                cat: {
                    "total_stock": round(b[0], 2),
                    "max_stock": b[1],
                    "num_skus": int(b[2]),
                }
                for cat, b in cat_totals.items()
            }

            out.append({
                # CareRobotics's GraphNode.node_id is "node_<int>" (see
                # src/environment/graph/config_loader.py). The env looks up
                # inventories.get(node.node_id), so the location_id we publish
                # must use the same format or it silently fails to match.
                "location_id": f"node_{nid}",
                "location_name": node.name,
                "sku_inventory": sku_inv,
                "category_inventory": cat_inv,
            })
        return out

    # ─── Read-only accessors ───────────────────────────────────────────────────

    def get_stock(self, node_id: int, sku_id: str) -> Optional[float]:
        node = self._nodes.get(node_id)
        if node is None:
            return None
        sku = node.skus.get(sku_id)
        return None if sku is None else sku.stock

    def is_consumption_enabled(self, node_id: int) -> bool:
        node = self._nodes.get(node_id)
        return False if node is None else node.consumption_enabled

    def node_name(self, node_id: int) -> Optional[str]:
        node = self._nodes.get(node_id)
        return None if node is None else node.name

    def set_consumption_rate(self, node_id: int, sku_id: str, rate_per_sec: float) -> None:
        """Override the default rate for a specific (node, SKU) pair."""
        if node_id in self._nodes and sku_id in self._nodes[node_id].skus:
            self._consumption_rates[(node_id, sku_id)] = float(rate_per_sec)
