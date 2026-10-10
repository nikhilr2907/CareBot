# Pick-and-place controller integration

Empty ament CMake package reserved for integrating the separate pick-and-place
STM32 controller. No runtime code, launch files or hardware protocol are defined.

The drive STM32 owns the motors and ToF sensors. This package is reserved for
communication with the separate manipulator controller once its source and
protocol are available.

Import embedded source under `firmware/pick_place/`, preserving its build files
and licenses. The eventual ROS C++ implementation will live in this package.
Before implementation, establish connection settings, message framing, commands,
acknowledgements, feedback, cancellation, errors and completion behavior.
