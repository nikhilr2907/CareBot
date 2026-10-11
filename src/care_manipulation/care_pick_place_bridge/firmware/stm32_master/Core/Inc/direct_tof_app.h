#ifndef DIRECT_TOF_APP_H
#define DIRECT_TOF_APP_H

#include <stdint.h>

/* Compile-time switches for the pair of VL53L0X ToF sensors wired directly to
 * the STM32 on I2C2 (left XSHUT=PA5, right XSHUT=PD2, shared SDA=PB3/SCL=PB10).
 *
 *   DIRECT_TOF_TEST_MODE       1 keeps motion disabled and runs the ToF
 *                                diagnostic loop (retry, bench, status prints).
 *   DIRECT_TOF_LONG_CABLE_MODE 1 slows I2C2 to 10 kHz. Used by MX_I2C2_Init in
 *                                main.c, declared here so it is visible there. */
#define DIRECT_TOF_TEST_MODE       0
#define DIRECT_TOF_LONG_CABLE_MODE 1

/* Bring up both VL53L0X sensors on I2C2: init, SDA/SCL recovery, bus scan,
 * wiring banner. Call once from main() after HAL and MX_I2C2_Init have run. */
void direct_tof_bringup(void);

/* Per-tick service. Retries bringup when either sensor is offline, runs the
 * optional benchmark sweep (DIRECT_TOF_BENCH_ENABLE), and throttles a one-line
 * status print. No-op when DIRECT_TOF_TEST_MODE is 0. */
void direct_tof_service(uint32_t now_ms);

/* One-line status print of the sensor that is physically installed on the
 * robot's right side. It remains the legacy "left" slot inside the pair
 * driver, so this wrapper exists to label it correctly in logs. */
void direct_tof_print_physical_right(uint32_t now_ms);

#endif /* DIRECT_TOF_APP_H */
