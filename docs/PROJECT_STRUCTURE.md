# CareRobotics workspace

This is a ROS 2 Humble colcon workspace. Layer directories group packages;
ROS package names and existing paths remain unchanged.

| Layer | Packages / sources | Current responsibility |
|---|---|---|
| Interfaces | `care_interfaces` | Scaffold for shared messages, services and actions |
| Robot description | `care_description` | URDF, sensor frames and meshes |
| Hardware | `care_bridge/care_hw_bridge`, `care_bridge/care_tof_bridge` | Drive STM32 commands and ToF input; combined bridge owns one serial port |
| Perception | `care_perception/*` | Sensor fusion, ToF processing and vision scaffolds |
| Camera experiments | `care_april_tag_detection` | Camera calibration, detection and ROS AprilTag dock-pose publishing scripts |
| Localization | `care_localization/*` | Package scaffolds; active localization is launched by care_navigation |
| Navigation | `care_navigation` | Gazebo, mapping, saved-map localization, Nav2 and docking configuration |
| Docking dependency | `opennav_docking` | Empty reserved directory; docking dependency must be supplied |
| Manipulation | `care_manipulation/care_pick_place_bridge` | Empty C++ package scaffold for a separate pick-and-place STM32 |
| Application | `care_app/care_ml_websocket_bridge` | ML WebSocket commands, navigation/docking actions and telemetry |
| Mock system | `care_mock/mock_nav2_server` | Simulated navigation and inventory for application integration |
| Bringup | `care_bringup` | Top-level launch scaffold; no unified robot launch implemented yet |

## Controller boundaries

The drive STM32 owns motors and ToF sensors. The pick-and-place STM32 is a
separate device with its own serial connection and firmware. Neither bridge
should open the other controller's port. Use stable `/dev/serial/by-id/` device
paths rather than relying on changing ttyACM numbering.

Firmware remains outside ROS packages. When importing controller source, use
`firmware/drive/` and `firmware/pick_place/`, preserving their build files and
licenses. Do not put embedded firmware into a ROS node's `src/` directory.

## Development entry points

- Application/mock: `ros2 launch mock_nav2_server mock_and_bridge.launch.py`
- Gazebo navigation: `ros2 launch care_navigation care_nav2.launch.py`
- Drive controller: `ros2 launch care_hw_bridge stm32_serial_bridge.launch.py`
- Manipulator integration: `care_pick_place_bridge` is a scaffold with no executable yet.

Docker configuration is under `docker/`; devcontainer configuration is under
`.devcontainer/`; container improvements are a separate next step.
