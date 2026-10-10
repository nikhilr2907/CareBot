#!/usr/bin/env python3
"""
Bridge Node - Main ROS2 orchestrator
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, String
import asyncio
import json
import os
import queue
from typing import Dict, Any, Optional, Callable

from .task_state import TaskState
from .system_state import SystemState
from .nav2_controller import Nav2Controller
from .docking_controller import DockingController
from .message_converter import (
    odom_to_json,
    task_status_to_json,
    robot_state_to_json,
    system_state_to_json,
    location_inventory_to_json,
)


class BridgeNode(Node):
    """Main ROS2 node for robot bridge"""

    def __init__(self):
        super().__init__("robot_ros_bridge")

        # Declare all parameters from config
        self.declare_parameter("websocket_host", "0.0.0.0")
        self.declare_parameter("websocket_port", 8765)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("sku_inventory_topic", "/sensor/sku_inventory")
        self.declare_parameter("battery_topic", "/sensor/battery")
        self.declare_parameter("estop_topic", "/emergency_stop")
        self.declare_parameter("target_dock_tag_topic", "/target_dock_tag_id")
        self.declare_parameter("nav2_action", "navigate_through_poses")
        self.declare_parameter("dock_action", "/dock_robot")
        self.declare_parameter("nav2_timeout", 300.0)
        self.declare_parameter("robot_state_publish_rate", 10.0)
        self.declare_parameter("system_state_publish_rate", 1.0)
        self.declare_parameter("feedback_poll_rate", 10.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("buffer_inventory_updates", True)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("sensor_data_timeout", 30.0)
        self.declare_parameter("nav2_action_server_timeout", 5.0)
        self.declare_parameter("mqtt_enabled", False)
        self.declare_parameter("mqtt_broker_host", "localhost")
        self.declare_parameter("mqtt_broker_port", 1883)
        self.declare_parameter("mqtt_topic_prefix", "care/weight")
        self.declare_parameter("env_config_path", "")

        # Get all parameters
        self.config = {
            "websocket_host": self.get_parameter("websocket_host").value,
            "websocket_port": self.get_parameter("websocket_port").value,
            "odom_topic": self.get_parameter("odom_topic").value,
            "sku_inventory_topic": self.get_parameter("sku_inventory_topic").value,
            "battery_topic": self.get_parameter("battery_topic").value,
            "estop_topic": self.get_parameter("estop_topic").value,
            "target_dock_tag_topic": self.get_parameter("target_dock_tag_topic").value,
            "nav2_action": self.get_parameter("nav2_action").value,
            "dock_action": self.get_parameter("dock_action").value,
            "nav2_timeout": self.get_parameter("nav2_timeout").value,
            "robot_state_publish_rate": self.get_parameter("robot_state_publish_rate").value,
            "system_state_publish_rate": self.get_parameter("system_state_publish_rate").value,
            "feedback_poll_rate": self.get_parameter("feedback_poll_rate").value,
            "global_frame": self.get_parameter("global_frame").value,
            "base_frame": self.get_parameter("base_frame").value,
            "buffer_inventory_updates": self.get_parameter("buffer_inventory_updates").value,
            "log_level": self.get_parameter("log_level").value,
            "sensor_data_timeout": self.get_parameter("sensor_data_timeout").value,
            "nav2_action_server_timeout": self.get_parameter("nav2_action_server_timeout").value,
            "mqtt_enabled": self.get_parameter("mqtt_enabled").value,
            "mqtt_broker_host": self.get_parameter("mqtt_broker_host").value,
            "mqtt_broker_port": self.get_parameter("mqtt_broker_port").value,
            "mqtt_topic_prefix": self.get_parameter("mqtt_topic_prefix").value,
            "mqtt_shelf_config": self._load_shelf_config(
                self.get_parameter("env_config_path").value
            ),
        }

        # State management
        self.task_state = TaskState()
        self.system_state = SystemState()
        self.system_state.set_buffering(
            self.config["buffer_inventory_updates"],
        )

        # Message queue for thread-safe communication
        self.outgoing_queue: queue.Queue = queue.Queue(maxsize=1000)

        # Nav2 controller
        self.nav2_controller = Nav2Controller(
            self,
            global_frame=self.config["global_frame"],
            action_name=self.config["nav2_action"],
        )
        self.nav2_controller.set_callbacks(
            feedback_callback=self._on_nav2_feedback,
            result_callback=self._on_nav2_result,
        )

        # Docking controller for AprilTag-backed stations
        self.docking_controller = DockingController(
            self,
            action_name=self.config["dock_action"],
            action_server_timeout=self.config["nav2_action_server_timeout"],
        )
        self.docking_controller.set_callbacks(
            feedback_callback=self._on_docking_feedback,
            result_callback=self._on_docking_result,
        )
        self.target_dock_tag_pub = self.create_publisher(
            Int32,
            self.config["target_dock_tag_topic"],
            reliable_qos,
        )
        self.station_to_dock_id = {
            "home": "home_station",
            "home_station": "home_station",
            "delivery": "delivery_station_1",
            "delivery_1": "delivery_station_1",
            "delivery_station_1": "delivery_station_1",
            "first_delivery": "delivery_station_1",
            "first_delivery_station": "delivery_station_1",
        }
        self.dock_tag_ids = {
            "home_station": 1,
            "delivery_station_1": 2,
        }

        # WebSocket command handler callback
        self._websocket_command_handler: Optional[Callable] = None

        # Create subscriptions with QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odometry,
            qos,
        )

        self.inventory_sub = self.create_subscription(
            String,
            self.config["sku_inventory_topic"],
            self._on_sku_inventory,
            qos,
        )

        self.battery_sub = self.create_subscription(
            Float32,
            self.config["battery_topic"],
            self._on_battery,
            qos,
        )

        # Emergency stop uses RELIABLE QoS — safety signals must not be dropped
        self.estop_sub = self.create_subscription(
            Bool,
            self.config["estop_topic"],
            self._on_emergency_stop,
            reliable_qos,
        )

        # Create timers for periodic publishing
        robot_publish_period = 1.0 / self.config["robot_state_publish_rate"]
        system_publish_period = 1.0 / self.config["system_state_publish_rate"]

        self.robot_state_timer = self.create_timer(
            robot_publish_period,
            self._publish_robot_state,
        )

        self.system_state_timer = self.create_timer(
            system_publish_period,
            self._publish_system_state,
        )

        self.get_logger().info(
            f"BridgeNode initialized with config: {json.dumps(self.config, indent=2)}"
        )

    def _load_shelf_config(self, path: str) -> dict:
        """
        Parse the environment config JSON into a shelf lookup keyed by str(node_id).
        Merges per-SKU weight/category from the top-level 'skus' section with
        per-location par_level/reorder_point/max_capacity from each node's inventory.
        Returns: {str(node_id): {name, skus: {sku_id: {par_level, reorder_point,
                                                         max_capacity, weight_per_unit_g,
                                                         category, sku_name}}}}
        """
        if not path:
            return {}
        if not os.path.exists(path):
            self.get_logger().warn(f"env_config_path not found: {path}")
            return {}
        try:
            with open(path) as f:
                env = json.load(f)

            sku_defs = env.get("skus", {})
            result = {}

            for node in env.get("nodes", []):
                node_id = str(node["id"])
                node_skus = {}
                for sku_id, inv in node.get("inventory", {}).items():
                    sku_meta = sku_defs.get(sku_id, {})
                    node_skus[sku_id] = {
                        "par_level": inv.get("par_level", 0),
                        "reorder_point": inv.get("reorder_point", 0),
                        "max_capacity": inv.get("max_capacity", inv.get("initial_stock", 0)),
                        "weight_per_unit_g": sku_meta.get("weight_per_unit_g", 0.0),
                        "category": sku_meta.get("category", "unknown"),
                        "sku_name": sku_meta.get("name", sku_id),
                    }
                result[node_id] = {"name": node.get("name", node_id), "skus": node_skus}

            self.get_logger().info(
                f"Loaded shelf config: {len(result)} nodes, "
                f"{sum(len(v['skus']) for v in result.values())} SKU entries"
            )
            return result
        except Exception as e:
            self.get_logger().error(f"Failed to load env config: {e}")
            return {}

    def set_websocket_command_handler(self, handler: Callable) -> None:
        """
        Set the WebSocket command handler

        Args:
            handler: Callable(message_dict)
        """
        self._websocket_command_handler = handler

    def _on_odometry(self, msg: Odometry) -> None:
        """Callback for odometry updates"""
        odom_dict = odom_to_json(msg)

        # Update task state
        self.task_state.update_from_odom(
            position=odom_dict["position"],
            orientation=odom_dict["orientation_quat"],
            velocity=odom_dict["velocity"],
            frame_id=msg.header.frame_id,
        )

        # Queue robot state message
        robot_state = self.task_state.get_robot_state()
        self.outgoing_queue.put(("robot_state", robot_state))

    def _on_sku_inventory(self, msg: String) -> None:
        """Callback for SKU inventory"""
        try:
            data = json.loads(msg.data)

            if isinstance(data, dict):
                # Single location
                location_id = data.get("location_id")
                if location_id:
                    self.system_state.update_location_inventory(
                        location_id=location_id,
                        location_name=data.get("location_name", location_id),
                        sku_inventory=data.get("sku_inventory", {}),
                        category_inventory=data.get("category_inventory", {}),
                    )
            elif isinstance(data, list):
                # Multiple locations
                for item in data:
                    location_id = item.get("location_id")
                    if location_id:
                        self.system_state.update_location_inventory(
                            location_id=location_id,
                            location_name=item.get("location_name", location_id),
                            sku_inventory=item.get("sku_inventory", {}),
                            category_inventory=item.get("category_inventory", {}),
                        )

            if not self.config["buffer_inventory_updates"]:
                # Immediate publish
                self.outgoing_queue.put(
                    ("location_inventory", location_inventory_to_json(self.system_state.get_all_location_inventories()))
                )
        except Exception as e:
            self.get_logger().error(f"Error processing inventory: {e}")

    def _on_battery(self, msg: Float32) -> None:
        """Callback for battery level"""
        self.task_state.update_battery(msg.data)

    def _on_emergency_stop(self, msg: Bool) -> None:
        """Callback for hardware emergency stop signal"""
        if msg.data:
            self.get_logger().error("Emergency stop active — cancelling navigation")

            current_task = self.task_state.get_current_task()
            task_id = current_task.get("task_id")

            self._cancel_active_goals_threadsafe()
            self.task_state.clear_current_task()

            robot_state = self.task_state.get_robot_state()
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": True,
                    "reason": "hardware_triggered",
                    "task_id": task_id,
                    "position": robot_state.get("position"),
                },
            ))
        else:
            self.get_logger().info("Emergency stop cleared")
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": False,
                },
            ))

    def _publish_robot_state(self) -> None:
        """Timer callback to publish robot state"""
        robot_state = self.task_state.get_robot_state()
        self.outgoing_queue.put(("robot_state", robot_state))

    def _publish_system_state(self) -> None:
        """Timer callback to publish full system state"""
        robot_state = self.task_state.get_robot_state()
        inventories = self.system_state.get_all_location_inventories()

        system_state = system_state_to_json(robot_state, inventories)
        self.outgoing_queue.put(("system_state", system_state))

    def _on_nav2_feedback(self, task_id: str, feedback_data: Dict[str, Any]) -> None:
        """Callback for Nav2 feedback"""
        # Calculate progress
        distance_remaining = feedback_data.get("distance_remaining", 0.0)
        progress_percent = max(0, 100 - (distance_remaining * 10))  # Rough estimate

        status = task_status_to_json(
            task_id=task_id,
            status="in_progress",
            progress_percent=progress_percent,
            distance_remaining_m=distance_remaining,
            time_elapsed_s=feedback_data.get("navigation_time", 0.0),
        )
        self.outgoing_queue.put(("task_status", status))

    def _on_nav2_result(self, task_id: str, result_data: Dict[str, Any]) -> None:
        """Callback for Nav2 result"""
        status_str = result_data.get("status", "failed")

        if status_str == "succeeded":
            robot_state = self.task_state.get_robot_state()
            status = task_status_to_json(
                task_id=task_id,
                status="completed",
                final_position=robot_state["position"],
                total_time_s=0.0,
            )
        else:
            robot_state = self.task_state.get_robot_state()
            status = task_status_to_json(
                task_id=task_id,
                status="failed" if status_str != "canceled" else "canceled",
                error=f"Navigation {status_str}",
                position_at_failure=robot_state["position"],
            )

        self.outgoing_queue.put(("task_status", status))
        self.task_state.clear_current_task()

    def _on_docking_feedback(self, task_id: str, feedback_data: Dict[str, Any]) -> None:
        status = task_status_to_json(
            task_id=task_id,
            status="in_progress",
            progress_percent=0.0,
            distance_remaining_m=float(feedback_data.get("distance_remaining", 0.0) or 0.0),
            time_elapsed_s=0.0,
        )
        status["mode"] = "docking"
        status["dock_id"] = feedback_data.get("dock_id")
        self.outgoing_queue.put(("task_status", status))

    def _on_docking_result(self, task_id: str, result_data: Dict[str, Any]) -> None:
        status_str = result_data.get("status", "failed")
        robot_state = self.task_state.get_robot_state()

        if status_str == "succeeded":
            status = task_status_to_json(
                task_id=task_id,
                status="completed",
                final_position=robot_state["position"],
                total_time_s=0.0,
            )
        else:
            status = task_status_to_json(
                task_id=task_id,
                status="failed" if status_str != "canceled" else "canceled",
                error=f"Docking {status_str}",
                position_at_failure=robot_state["position"],
            )

        status["mode"] = "docking"
        status["dock_id"] = result_data.get("dock_id")
        self.outgoing_queue.put(("task_status", status))
        self.task_state.clear_current_task()

    def _resolve_dock_id(self, command_json: Dict[str, Any]) -> Optional[str]:
        requested = (
            command_json.get("dock_id")
            or command_json.get("station_id")
            or command_json.get("destination")
            or command_json.get("goal")
        )
        if not requested:
            return None
        return self.station_to_dock_id.get(str(requested), str(requested))

    def _resolve_tag_id(self, command_json: Dict[str, Any], dock_id: str) -> Optional[int]:
        tag_id = command_json.get("tag_id")
        if tag_id is not None:
            return int(tag_id)
        return self.dock_tag_ids.get(dock_id)

    def _publish_target_tag_id(self, tag_id: Optional[int]) -> None:
        msg = Int32()
        msg.data = int(tag_id) if tag_id is not None else -1
        self.target_dock_tag_pub.publish(msg)

    async def _cancel_active_goals(self) -> None:
        errors = []

        if self.nav2_controller.has_active_goal():
            try:
                await self.nav2_controller.cancel_goal()
            except Exception as e:
                errors.append(str(e))

        if self.docking_controller.has_active_goal():
            try:
                await self.docking_controller.cancel_goal()
            except Exception as e:
                errors.append(str(e))

        if errors:
            raise Exception("; ".join(errors))

    def _cancel_active_goals_threadsafe(self) -> None:
        loop = getattr(self, "fastapi_event_loop", None)
        if loop is None:
            self.get_logger().warn("FastAPI event loop unavailable; cannot cancel active goals")
            return
        asyncio.run_coroutine_threadsafe(self._cancel_active_goals(), loop)

    async def handle_task_command(self, task_json: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle task command from WebSocket

        Args:
            task_json: Task command dict

        Returns:
            Response dict
        """
        try:
            task_id = task_json.get("task_id")
            if not task_id:
                return {"status": "error", "message": "Missing task_id"}

            waypoints = task_json.get("waypoints", [])
            if not waypoints:
                return {"status": "error", "message": "Missing waypoints"}

            metadata = task_json.get("task_metadata", {})

            # Update task state
            self.task_state.set_current_task(task_id, waypoints, metadata)

            # Send to Nav2
            result = await self.nav2_controller.send_goal(task_id, waypoints)

            # Queue task status update
            status = task_status_to_json(
                task_id=task_id,
                status="in_progress",
                progress_percent=0.0,
                distance_remaining_m=0.0,
                time_elapsed_s=0.0,
            )
            self.outgoing_queue.put(("task_status", status))

            return {"status": "accepted", "task_id": task_id}
        except Exception as e:
            self.get_logger().error(f"Error handling task command: {e}")
            return {"status": "error", "message": str(e)}

    async def handle_station_command(self, command_json: Dict[str, Any]) -> Dict[str, Any]:
        """Handle a request to dock at a named AprilTag station."""
        try:
            task_id = command_json.get("task_id")
            if not task_id:
                dock_id = self._resolve_dock_id(command_json) or "station"
                task_id = f"dock_{dock_id}"

            dock_id = self._resolve_dock_id(command_json)
            if not dock_id:
                return {
                    "status": "error",
                    "message": "Missing dock_id/station_id/destination",
                }
            tag_id = self._resolve_tag_id(command_json, dock_id)
            self._publish_target_tag_id(tag_id)

            metadata = {
                "mode": "docking",
                "dock_id": dock_id,
                "tag_id": tag_id,
                "request": command_json,
            }
            self.task_state.set_current_task(task_id, [], metadata)

            result = await self.docking_controller.send_dock_goal(
                task_id=task_id,
                dock_id=dock_id,
                navigate_to_staging_pose=bool(command_json.get("navigate_to_staging_pose", True)),
                max_staging_time=float(command_json.get("max_staging_time", 60.0)),
            )

            status = task_status_to_json(
                task_id=task_id,
                status="in_progress",
                progress_percent=0.0,
                distance_remaining_m=0.0,
                time_elapsed_s=0.0,
            )
            status["mode"] = "docking"
            status["dock_id"] = dock_id
            status["tag_id"] = tag_id
            self.outgoing_queue.put(("task_status", status))

            return {
                "status": "accepted",
                "task_id": task_id,
                "dock_id": dock_id,
                "tag_id": tag_id,
                "goal_id": result.get("goal_id"),
            }
        except Exception as e:
            self.get_logger().error(f"Error handling station command: {e}")
            self.task_state.clear_current_task()
            return {"status": "error", "message": str(e)}

    async def handle_go_home(self, command_json: Dict[str, Any]) -> Dict[str, Any]:
        command = dict(command_json)
        command["dock_id"] = "home_station"
        command.setdefault("task_id", "dock_home_station")
        return await self.handle_station_command(command)

    async def handle_go_delivery_station_1(self, command_json: Dict[str, Any]) -> Dict[str, Any]:
        command = dict(command_json)
        command["dock_id"] = "delivery_station_1"
        command.setdefault("task_id", "dock_delivery_station_1")
        return await self.handle_station_command(command)

    async def handle_emergency_stop(self) -> Dict[str, Any]:
        """Handle emergency stop command received from CareRobotics via WebSocket"""
        try:
            self.get_logger().error("Emergency stop received from CareRobotics — cancelling navigation")

            current_task = self.task_state.get_current_task()
            task_id = current_task.get("task_id")

            await self._cancel_active_goals()
            self.task_state.clear_current_task()

            robot_state = self.task_state.get_robot_state()
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": True,
                    "reason": "operator_triggered",
                    "task_id": task_id,
                    "position": robot_state.get("position"),
                },
            ))

            return {"status": "ok", "message": "Emergency stop executed"}
        except Exception as e:
            self.get_logger().error(f"Error handling emergency stop: {e}")
            return {"status": "error", "message": str(e)}

    async def handle_task_cancel(self, task_id: str) -> Dict[str, Any]:
        """
        Handle task cancellation

        Args:
            task_id: Task ID to cancel

        Returns:
            Response dict
        """
        try:
            current_task = self.task_state.get_current_task()
            if current_task["task_id"] != task_id:
                return {"status": "error", "message": f"Task {task_id} is not active"}

            await self._cancel_active_goals()
            self.task_state.clear_current_task()

            # Queue task status
            status = task_status_to_json(
                task_id=task_id,
                status="canceled",
            )
            self.outgoing_queue.put(("task_status", status))

            return {"status": "canceled", "task_id": task_id}
        except Exception as e:
            self.get_logger().error(f"Error canceling task: {e}")
            return {"status": "error", "message": str(e)}

    def get_outgoing_queue(self) -> queue.Queue:
        """Get the outgoing message queue"""
        return self.outgoing_queue
