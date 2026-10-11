#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIO="${PIO:-platformio}"
if [ -n "${ARM_GCC_BIN:-}" ]; then
  export PATH="$ARM_GCC_BIN:$PATH"
fi

echo "== STM32 master =="
(
  cd "$ROOT_DIR/stm32_master"
  cmake --preset Debug
  cmake --build --preset Debug
)

echo "== ESP32 VL53L0X + ultrasonic node =="
(
  cd "$ROOT_DIR/esp32_vl53l0x_ultrasonic_node"
  "$PIO" run
)

echo "== ESP32 VL53L5CX + fan node =="
(
  cd "$ROOT_DIR/esp32_vl53l5_fan_node"
  "$PIO" run
)

echo "All builds completed."
