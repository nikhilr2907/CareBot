# STM32 Stepper Motor Map

ENA is not driven by firmware. Leave the DM542-style driver ENA inputs disconnected/floating unless hardware is revised.

| Motor | Physical function | PUL pin | DIR pin |
| --- | --- | --- | --- |
| M1 | Conveyor X axis left | PA6 | PB12 |
| M2 | Y axis right | PA7 | PB14 |
| M3 | Conveyor belt right | PC8 | PB15 |
| M4 | Conveyor belt left | PB1 | PB2 |
| M5 | Y axis left | PA11 | PC4 |
| M6 | Conveyor X axis right | PB8 | PC5 |
| M7 | X axis angle motor | PB9 | PC1 |

Useful pairs:

| Pair | Motors |
| --- | --- |
| Y axis | M5 + M2 |
| Conveyor X axis | M1 + M6 |
| Conveyor belt | M4 + M3 |

