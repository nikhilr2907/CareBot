# STM32F446RE Peripheral Map

From `stm32_master/stm32f446LowerLevel.ioc` and the current firmware:

| Peripheral | Pins | Usage |
| --- | --- | --- |
| USART2 | PA2 TX, PA3 RX | STLink VCP live terminal, Raspberry Pi command channel |
| UART4 | PA0 TX, PA1 RX | Preferred ESP32 sensor-node RS485 input at 115200 baud |
| USART3 | PC10 TX, PC11 RX | Fallback ESP32 sensor-node RS485 input at 115200 baud |
| USART1 | PA9 TX, PA10 RX | BLD-515C motor-driver RS485 at 9600 baud |
| I2C2 | PB3 SDA, PB10 SCL | BNO055 IMU |
| TIM3 | PA6, PA7, PC8, PB1 | PWM outputs retained from original motor/IO config |
| TIM1 | PA11 | PWM output retained from original config |
| TIM10 | PB8 | PWM output retained from original config |
| TIM11 | PB9 | PWM output retained from original config |
| GPIO | PA5 | Nucleo LD2 green LED |
| GPIO | PB12, PB14, PB15, PB2, PC1, PC4, PC5 | Motor direction/control outputs from original config |
| GPIO | PC3 | BNO055 reset |
| GPIO | PB6, PB7, PC2, PC12 | Legacy ultrasonic trigger outputs; current firmware does not toggle them |

Boot motor movement is disabled. The STM32 may probe the BLD-515C bus, but it
does not run a canned spin-up sequence.
