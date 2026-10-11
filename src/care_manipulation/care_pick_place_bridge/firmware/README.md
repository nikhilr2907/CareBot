# Pick-and-place firmware

Code that runs on the embedded hardware: the STM32F446 master controller and the
two ESP32 sensor nodes. Host-side developer tools live in `../dev_tools/`.

## Layout

| Directory | Contents |
|---|---|
| `stm32_master/` | STM32 application, startup/linker inputs, CMake configuration and vendor drivers |
| `esp32_vl53l0x_ultrasonic_node/` | Point ToF and ultrasonic sensor node |
| `esp32_vl53l5_fan_node/` | ToF array and fan-control sensor node |
| `docs/` | Motor mapping, wiring and protocol notes |

## Controller interfaces

ESP32 nodes send RS485 frames at 115200 baud:

```text
AA 55 LEN CMD PAYLOAD... CS
```

`CS` is the low byte of the sum of `LEN`, `CMD` and the payload. `CMD=0x01`
contains point-ToF and ultrasonic readings; `CMD=0x02` contains ToF-array readings
and online/stop/fan status. See [the frame specification](docs/protocols/rs485_sensor_frames.md).

The STM32 exposes its host command/status channel over USART2/STLink VCP.
Motor positions are based on commanded pulses rather than encoder measurements.

## Build inputs

STM32 builds use ARM GNU tools, CMake and Ninja with the `Debug` or `Release`
presets in `stm32_master/`. ESP32 projects use their own PlatformIO configuration.
`../dev_tools/build_all.sh` builds the three active projects. Hardware flashing is separate.

HAL/CMSIS and VL53L0X sources are retained under `stm32_master/Drivers/`, including
vendor licence notices.

See [motor mapping](docs/motor_map.md) for hardware conventions, and
[../dev_tools/](../dev_tools/) for host-side scripts and web consoles.
