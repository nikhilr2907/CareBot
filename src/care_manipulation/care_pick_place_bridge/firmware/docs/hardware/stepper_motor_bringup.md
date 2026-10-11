# Stepper Motor Bring-Up

The STM32 firmware exposes a manual stepper terminal on USART2/STLink VCP at
115200 baud. Use it to identify which physical DM542 driver is M1..M7.

## Motor Map

| Terminal motor | PUL pin | DIR pin |
| --- | --- | --- |
| M1 | PA6 | PB12 |
| M2 | PA7 | PB14 |
| M3 | PC8 | PB15 |
| M4 | PB1 | PB2 |
| M5 | PA11 | PC4 |
| M6 | PB8 | PC5 |
| M7 | PB9 | PC1 |

## Desktop Tool

```sh
python3 tools/stm32_motor_finder.py --port /dev/cu.usbmodem2103
```

The tool starts by sending `S` and `LOG 0`. It will not move a motor until
Space is pressed for continuous mode or Enter is pressed for a jog.

Useful keys:

- `1`..`7`: select motor, stopping the previous one first.
- `c`: continuous mode.
- `j`: jog/by-step mode.
- `Space`: start or stop continuous pulses.
- `Enter`: jog the selected motor by the shown pulse count.
- `+` / `-`: speed up/down in continuous mode, or change jog pulse count in
  jog mode.
- `f` / `r`: direction.
- `0`: stop all motors immediately.
- `q`: stop all and quit.

## Electrical Checklist

- The STM32 Micro-USB cable only powers/programs the STM32/STLink side.
- The DM542 stepper driver still needs its own motor supply enabled through the
  distribution/fuse/E-stop path.
- The motor coils must be connected to the DM542 output terminals.
- The driver input side must see the STM32 PUL/DIR signals and their matching
  reference/common. If the driver expects 5 V opto input wiring, a bare 3.3 V
  STM32 pin may not trigger it reliably.
- Check ENA wiring and DIP settings. On many stepper drivers, the wrong ENA
  level disables the output stage even while PUL/DIR are correct.
- If the board says `OK: Mx continuous ...` but no motor ever moves, suspect
  driver power, ENA, PUL/DIR/common wiring, or coil wiring before changing code.
