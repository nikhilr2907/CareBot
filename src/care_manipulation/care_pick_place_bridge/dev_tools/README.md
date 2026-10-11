# Pick-and-place developer tools

Host-side scripts and consoles for building, flashing, and debugging the firmware
in `../firmware/`. **None of this runs on the microcontrollers** — these are
developer utilities that run on your PC and talk to the hardware over serial.

## Build / flash

| File | Purpose |
|---|---|
| `build_all.sh` | Build STM32 + both ESP32 projects |
| `flash_stm32_master.sh` | Flash the compiled STM32 image to the board |

## Serial monitors

| File | Purpose |
|---|---|
| `stm32_vcp_live.py` | Live view of the STM32's USART2 debug stream |
| `rs485_live_feed.py` | Capture and decode RS485 sensor frames from the ESP32 nodes |
| `stm32_loadcell_monitor.py` | Stream HX711 load-cell readings |

## Testing / calibration utilities

| File | Purpose |
|---|---|
| `stm32_motor_finder.py` | Discover and verify stepper wiring |
| `stm32_load_stage_tester.py` | Exercise lift/load-stage mechanics |

## Browser consoles

Python servers that expose a web UI for interactive control. See each folder's
README for how to run them.

| Folder | Purpose |
|---|---|
| `robot_control/` | Conveyor / manipulator control dashboard |
| `lift_calibrator/` | Lift mechanism calibration |

`start_lift_calibrator.command` is a macOS launcher shortcut for the lift
calibrator server.
