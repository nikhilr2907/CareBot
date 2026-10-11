# Pick-and-place ROS integration

Ament CMake package reserved for the ROS adapter to the separate pick-and-place
STM32 controller. It currently has no runtime code or launch files.

The imported embedded firmware and host control tools live in
[firmware/pick_place](../../../firmware/pick_place/README.md), outside the ROS
package. The STM32 application and ESP32 sensor nodes have their own embedded
build configurations; they are not compiled by colcon.
