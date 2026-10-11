#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="$ROOT_DIR/stm32_master"
NCPU="${BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"
if [ -n "${ARM_GCC_BIN:-}" ]; then
  export PATH="$ARM_GCC_BIN:$PATH"
fi

printf "\nSTM32 master build/flash\n"
printf "Project: %s\n\n" "$PROJECT"

if [ ! -f "$PROJECT/CMakeLists.txt" ]; then
  printf "Cannot find STM32 CMake project at:\n  %s\n" "$PROJECT"
  exit 1
fi

if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  printf "arm-none-eabi-gcc was not found. Add it to PATH or set ARM_GCC_BIN.\n"
  exit 1
fi

cd "$PROJECT"
cmake --preset Debug
cmake --build --preset Debug --parallel "$NCPU"

ARTIFACT="$PROJECT/build/Debug/stm32f446LowerLevel.bin"
if [ ! -f "$ARTIFACT" ]; then
  printf "Build completed, but firmware binary was not found:\n  %s\n" "$ARTIFACT"
  exit 1
fi

printf "\nFirmware built:\n  %s\n\n" "$ARTIFACT"

if ! command -v st-info >/dev/null 2>&1 || ! st-info --probe 2>/dev/null | grep -q "Found 1 stlink"; then
  printf "No STLink is visible right now. Connect STM32 Micro-USB, then rerun this script.\n"
  exit 1
fi

if command -v st-flash >/dev/null 2>&1; then
  printf "Flashing with st-flash...\n"
  st-flash --reset write "$ARTIFACT" 0x08000000
elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  printf "Flashing with STM32_Programmer_CLI...\n"
  STM32_Programmer_CLI -c port=SWD mode=UR -w "$ARTIFACT" 0x08000000 -v -rst
else
  printf "Build succeeded, but neither st-flash nor STM32_Programmer_CLI was found.\n"
  exit 1
fi

printf "\nFlash complete.\n"
printf "Open the motor finder with:\n"
printf "  python3 %s/tools/stm32_motor_finder.py --port <serial-device>\n" "$ROOT_DIR"
