"""
Mock Nav2 server + sensor publisher.

ROS plumbing only — physics + inventory dynamics live in robot_sim.py and
consumption_sim.py. This node:

  - Loads the same JSON config the CareRobotics env uses.
  - Builds a RobotSim and a ConsumptionSim from it.
  - Exposes a NavigateThroughPoses action server (replaces real Nav2).
  - Publishes /odom (10 Hz), /sensor/battery (1 Hz), /sensor/sku_inventory (1 Hz).

Wall-clock time discipline:
  - Each owning timer measures its own dt from the node's clock and passes it
    to the sim modules.  RobotSim is advanced ONLY by the odom timer; the
    feedback loop and battery publisher only read state.
  - ConsumptionSim is advanced ONLY by the inventory timer.
  - Sim modules are clock-blind; this node is the sole source of dt.

Threading:
  - MultiThreadedExecutor + ReentrantCallbackGroup for the action server so
    its execute_callback can yield via Rate.sleep() while timers keep firing.
  - A single threading.Lock serialises access to RobotSim and ConsumptionSim
    across the action thread and timer threads.
"""

import json
import math
import threading
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from .consumption_sim import ConsumptionSim
from .robot_sim import RobotSim


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
    )


def _seconds_to_duration_msg(seconds: float) -> DurationMsg:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return DurationMsg(sec=sec, nanosec=nanosec)


class MockNav2Node(Node):
    """ROS 2 node that wraps RobotSim + ConsumptionSim and exposes the Nav2 action."""

    def __init__(self) -> None:
        super().__init__("mock_nav2_server")

        # ── Parameters (defaults match config/default.yaml) ──
        self.declare_parameter("config_path", "/care_configs/ilc_pilot_v1_care_robotics.json")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("battery_topic", "/sensor/battery")
        self.declare_parameter("sku_inventory_topic", "/sensor/sku_inventory")
        self.declare_parameter("nav2_action", "navigate_through_poses")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_rate_hz", 10.0)
        self.declare_parameter("battery_rate_hz", 1.0)
        self.declare_parameter("inventory_rate_hz", 1.0)
        self.declare_parameter("feedback_rate_hz", 10.0)
        self.declare_parameter("initial_node_id", 0)
        self.declare_parameter("battery_drain_per_sec", 0.0001)
        self.declare_parameter("battery_drain_per_meter", 0.0005)
        self.declare_parameter("default_consumption_rate_per_sec", 0.05)
        self.declare_parameter("default_restock_amount", 5)
        self.declare_parameter("arrival_tolerance_m", 0.15)

        config_path = self.get_parameter("config_path").value
        self._global_frame = self.get_parameter("global_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._odom_rate = float(self.get_parameter("odom_rate_hz").value)
        self._battery_rate = float(self.get_parameter("battery_rate_hz").value)
        self._inventory_rate = float(self.get_parameter("inventory_rate_hz").value)
        self._feedback_rate = float(self.get_parameter("feedback_rate_hz").value)
        self._default_restock = int(self.get_parameter("default_restock_amount").value)
        self._arrival_tol = float(self.get_parameter("arrival_tolerance_m").value)

        # ── Load config + build sims ──
        self.get_logger().info(f"Loading env config: {config_path}")
        with open(config_path, "r") as f:
            config = json.load(f)

        self.robot_sim = RobotSim.from_config(
            config,
            initial_node_id=int(self.get_parameter("initial_node_id").value),
            battery_drain_per_sec=float(self.get_parameter("battery_drain_per_sec").value),
            battery_drain_per_meter=float(self.get_parameter("battery_drain_per_meter").value),
        )
        self.consumption_sim = ConsumptionSim.from_config(
            config,
            default_rate_per_sec=float(self.get_parameter("default_consumption_rate_per_sec").value),
            default_restock_amount=self._default_restock,
        )

        self.get_logger().info(
            f"Loaded {len(self.robot_sim.nodes)} nodes; "
            f"initial pose=({self.robot_sim.position[0]:.2f}, {self.robot_sim.position[1]:.2f})"
        )

        # ── Sim lock — serialises access from action thread + timer threads ──
        self._sim_lock = threading.Lock()

        # ── Callback groups ──
        # ReentrantCallbackGroup for the action server (long-running execute_callback
        # needs to yield via Rate.sleep() while timers keep firing).
        # MutuallyExclusiveCallbackGroup for timers — they don't need to interleave.
        self._action_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, odom_qos
        )
        self._battery_pub = self.create_publisher(
            Float32, self.get_parameter("battery_topic").value, odom_qos
        )
        self._inventory_pub = self.create_publisher(
            String, self.get_parameter("sku_inventory_topic").value, odom_qos
        )

        # ── Timers ──
        now = self.get_clock().now()
        self._t_last_odom = now
        self._t_last_inv = now
        self.create_timer(
            1.0 / self._odom_rate, self._on_odom_tick, callback_group=self._timer_cb_group
        )
        self.create_timer(
            1.0 / self._battery_rate, self._on_battery_tick, callback_group=self._timer_cb_group
        )
        self.create_timer(
            1.0 / self._inventory_rate, self._on_inventory_tick, callback_group=self._timer_cb_group
        )

        # ── Action server ──
        self._action_server = ActionServer(
            self,
            NavigateThroughPoses,
            self.get_parameter("nav2_action").value,
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._action_cb_group,
        )
        self._active_goal_present = False

        self.get_logger().info(
            f"MockNav2Node ready: action='{self.get_parameter('nav2_action').value}', "
            f"/odom @ {self._odom_rate} Hz, "
            f"/sensor/battery @ {self._battery_rate} Hz, "
            f"/sensor/sku_inventory @ {self._inventory_rate} Hz"
        )

    # ─── Timer callbacks ────────────────────────────────────────────────────────

    def _on_odom_tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._t_last_odom).nanoseconds / 1e9
        self._t_last_odom = now

        with self._sim_lock:
            self.robot_sim.update(dt)
            state = self.robot_sim.get_state()

        self._odom_pub.publish(self._build_odometry_msg(state, now.to_msg()))

    def _on_battery_tick(self) -> None:
        with self._sim_lock:
            level = float(self.robot_sim.battery)
        self._battery_pub.publish(Float32(data=level))

    def _on_inventory_tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._t_last_inv).nanoseconds / 1e9
        self._t_last_inv = now

        with self._sim_lock:
            self.consumption_sim.tick(dt)
            payload = self.consumption_sim.snapshot()

        self._inventory_pub.publish(String(data=json.dumps(payload)))

    # ─── Action callbacks ───────────────────────────────────────────────────────

    def _on_goal_request(self, goal_request) -> GoalResponse:
        """Reject new goals while one is already in flight (single-robot mock)."""
        if self._active_goal_present:
            self.get_logger().warn("Goal rejected — another goal is already executing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested — accepting")
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        """Drive the robot through the requested poses, publishing feedback."""
        self._active_goal_present = True
        try:
            return self._execute_goal_inner(goal_handle)
        finally:
            self._active_goal_present = False

    def _execute_goal_inner(self, goal_handle):
        poses: List[PoseStamped] = list(goal_handle.request.poses)
        self.get_logger().info(f"New goal: {len(poses)} waypoints")

        if not poses:
            goal_handle.abort()
            return NavigateThroughPoses.Result()

        # Snap each waypoint to the nearest graph node id.
        node_ids: List[int] = []
        for p in poses:
            nid = self._snap_to_nearest_node(p.pose.position.x, p.pose.position.y)
            if nid is not None:
                node_ids.append(nid)

        if not node_ids:
            self.get_logger().error("Could not snap any waypoint to a graph node — aborting")
            goal_handle.abort()
            return NavigateThroughPoses.Result()

        # Prepend current node so RobotSim treats subsequent ids as targets.
        with self._sim_lock:
            current = self.robot_sim.current_node_id()
            path = ([current] + node_ids) if current is not None else node_ids
            self.robot_sim.set_path(path)

        # Drive the feedback loop until idle (or canceled).
        rate = self.create_rate(self._feedback_rate, self.get_clock())
        feedback_msg = NavigateThroughPoses.Feedback()
        goal_start = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._sim_lock:
                    self.robot_sim.set_path([])  # cancel motion
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return NavigateThroughPoses.Result()

            with self._sim_lock:
                state = self.robot_sim.get_state()
                distance_remaining = self.robot_sim.distance_remaining()
                idle = self.robot_sim.is_idle()

            now = self.get_clock().now()
            elapsed_s = (now - goal_start).nanoseconds / 1e9

            feedback_msg.current_pose = self._make_pose_stamped(state, now.to_msg())
            feedback_msg.distance_remaining = float(distance_remaining)
            feedback_msg.navigation_time = _seconds_to_duration_msg(elapsed_s)
            est_remaining = distance_remaining / max(state["linear_x"], 0.01) if not idle else 0.0
            feedback_msg.estimated_time_remaining = _seconds_to_duration_msg(est_remaining)
            feedback_msg.number_of_recoveries = 0
            goal_handle.publish_feedback(feedback_msg)

            if idle:
                break

            rate.sleep()

        # Robot arrived at the final node — restock if it's a recovery node.
        final_node_id = node_ids[-1]
        if self.consumption_sim.is_consumption_enabled(final_node_id):
            with self._sim_lock:
                self.consumption_sim.restock(final_node_id, amount=self._default_restock)
            self.get_logger().info(
                f"Delivered to node {final_node_id} ('{self.consumption_sim.node_name(final_node_id)}'); "
                f"restocked {self._default_restock} items"
            )

        goal_handle.succeed()
        self.get_logger().info(f"Goal succeeded after {(self.get_clock().now() - goal_start).nanoseconds / 1e9:.1f}s")
        return NavigateThroughPoses.Result()

    # ─── Helpers ────────────────────────────────────────────────────────────────

    def _snap_to_nearest_node(self, x: float, y: float) -> Optional[int]:
        """Return the node_id with smallest Euclidean distance to (x, y)."""
        best_id: Optional[int] = None
        best_d2 = float("inf")
        for nid, n in self.robot_sim.nodes.items():
            d2 = (n.x - x) ** 2 + (n.y - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_id = nid
        return best_id

    def _build_odometry_msg(self, state: dict, stamp) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._global_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = float(state["x"])
        msg.pose.pose.position.y = float(state["y"])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(float(state["yaw"]))
        msg.twist.twist.linear.x = float(state["linear_x"])
        msg.twist.twist.angular.z = float(state["angular_z"])
        return msg

    def _make_pose_stamped(self, state: dict, stamp) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self._global_frame
        ps.pose.position.x = float(state["x"])
        ps.pose.position.y = float(state["y"])
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_quaternion(float(state["yaw"]))
        return ps


# ─── Entry point ────────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockNav2Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
