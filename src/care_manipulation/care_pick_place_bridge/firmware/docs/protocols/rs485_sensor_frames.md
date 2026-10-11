# RS485 Sensor Frames

All ESP32 sensor nodes use the same binary envelope:

```text
AA 55 LEN CMD PAYLOAD... CS
```

- `AA 55` - frame header.
- `LEN` - payload length only, not including header, `LEN`, `CMD`, or `CS`.
- `CMD` - node payload type.
- `CS` - low byte of `LEN + CMD + sum(PAYLOAD)`.
- Multi-byte distances are unsigned 16-bit big-endian millimetres.

## `CMD=0x01` VL53L0X + Ultrasonic Node

Payload length: `10`.

```text
tof0_hi tof0_lo
tof1_hi tof1_lo
tof2_hi tof2_lo
tof3_hi tof3_lo
ultra_hi ultra_lo
```

Distance sentinel: `9999` means offline, timeout, or out of range.

This is the frame shape verified during the original ESP32/RS485 test.

## `CMD=0x02` VL53L5CX + Fan Node

Payload length: `99`.

```text
online_mask
stop_flags
fan_duty_percent
sensor0_zone0_hi sensor0_zone0_lo ... sensor0_zone15_hi sensor0_zone15_lo
sensor1_zone0_hi sensor1_zone0_lo ... sensor1_zone15_hi sensor1_zone15_lo
sensor2_zone0_hi sensor2_zone0_lo ... sensor2_zone15_hi sensor2_zone15_lo
```

- `online_mask`: bit `n` is set when sensor `n` initialized successfully.
- `stop_flags`: bit `n` is set when sensor `n` has at least one valid zone below
  its stop threshold.
- `fan_duty_percent`: current PWM duty sent to the fans.
- Grid sentinel: `0xFFFF` means invalid/no target for that zone.

At 20 Hz this frame is about 2.1 kB/s, which is comfortable at 115200 baud.
