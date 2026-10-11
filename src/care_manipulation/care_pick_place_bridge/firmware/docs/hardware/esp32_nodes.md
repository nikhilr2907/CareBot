# ESP32 Sensor Nodes

## VL53L0X + Ultrasonic Node

Folder: `esp32_vl53l0x_ultrasonic_node/`

| Signal | ESP32 pin |
| --- | --- |
| I2C SDA | GPIO21 |
| I2C SCL | GPIO22 |
| VL53L0X XSHUT | GPIO32, GPIO33, GPIO25, GPIO26 |
| DYP-A02 UART RX | GPIO4 |
| DYP-A02 UART TX/trigger | GPIO5 |
| RS485 UART RX | GPIO16 |
| RS485 UART TX | GPIO17 |
| Status LED | GPIO2 |

RS485 frame: `CMD=0x01`.

## VL53L5CX + Fan Node

Folder: `esp32_vl53l5_fan_node/`

| Signal | ESP32 pin |
| --- | --- |
| I2C SDA | GPIO21 |
| I2C SCL | GPIO22 |
| LTC4311 enable | GPIO32 |
| VL53L5CX LPn | GPIO14, GPIO27, GPIO25 |
| VL53L5CX INT | GPIO13, GPIO35, GPIO26 |
| Fan PWM | GPIO18, GPIO19 |
| RS485 UART RX | GPIO16 |
| RS485 UART TX | GPIO17 |
| Status LED | GPIO2 |

RS485 frame: `CMD=0x02`.

If the final harness uses different pins, change the arrays in
`esp32_vl53l5_fan_node/src/main.cpp`.
