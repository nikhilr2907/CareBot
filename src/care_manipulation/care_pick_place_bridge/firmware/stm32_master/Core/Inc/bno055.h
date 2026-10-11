#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fused Euler angles in degrees. BNO055 outputs heading 0..360,
 * roll -180..180, pitch -90..90 once it has a valid magnetic + gravity
 * reference. Before calibration the absolute values drift -- watch the
 * calib field to know when to trust them. */
typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
} bno055_euler_t;

/* Per-subsystem calibration status. 0 = uncalibrated, 3 = fully calibrated.
 * Gyro/accel hit 3 quickly with normal motion; mag needs a figure-8 wave
 * to settle. sys reaches 3 once all three sources are in agreement. */
typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_calib_t;

/* Hardware-reset the BNO055 via PC3 and configure it for NDOF (9-DOF
 * sensor fusion). Blocks for ~750 ms while the chip boots -- call once
 * during MCU init, not in a loop. Returns false if the chip doesn't ACK
 * or the CHIP_ID register doesn't read 0xA0. */
bool bno055_init(void);

/* Read fused Euler angles. Single I2C burst, ~600 us at 100 kHz. */
bool bno055_read_euler(bno055_euler_t *out);

/* Read calibration byte. Useful in heartbeat logs to see when the chip
 * is ready to trust. */
bool bno055_read_calib(bno055_calib_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
