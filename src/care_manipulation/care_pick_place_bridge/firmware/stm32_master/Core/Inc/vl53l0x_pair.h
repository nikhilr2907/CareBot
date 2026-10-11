#ifndef VL53L0X_PAIR_H
#define VL53L0X_PAIR_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Legacy software labels. In the installed robot, left_* is the physical
     * RIGHT D13/PA5 sensor and right_* is the physical LEFT PD2 sensor. */
    uint16_t left_mm;
    uint16_t right_mm;
    /* Most recent successful readings, retained for diagnostics. */
    uint16_t left_raw_mm;
    uint16_t right_raw_mm;
    uint8_t left_average_samples;
    uint8_t right_average_samples;
    uint8_t left_range_status;
    uint8_t right_range_status;
    bool left_online;
    bool right_online;
    uint32_t left_samples;
    uint32_t right_samples;
    uint32_t left_errors;
    uint32_t right_errors;
    uint32_t left_last_update_ms;
    uint32_t right_last_update_ms;
    uint32_t timing_budget_us;
    uint32_t intershot_gap_ms;
} vl53l0x_pair_snapshot_t;

/* D13/PA5 (physical RIGHT) and PD2 (physical LEFT) must already be configured
 * as outputs and driven low. This function hard-resets both sensors, releases
 * PD2 and assigns it address 0x30, then releases D13 at address 0x29. If PD2
 * fails, D13 is brought up alone so right-conveyor feedback remains usable. */
bool vl53l0x_pair_init(I2C_HandleTypeDef *i2c);

/* Non-blocking state-machine service. Call on every main-loop pass. */
void vl53l0x_pair_service(uint32_t now_ms);

/* Change the ranging tradeoff without reflashing. Both sensors remain in
 * alternating single-shot mode. */
bool vl53l0x_pair_set_profile(uint32_t timing_budget_us,
                             uint32_t intershot_gap_ms);

void vl53l0x_pair_get_snapshot(vl53l0x_pair_snapshot_t *out);
const char *vl53l0x_pair_init_stage(void);
int vl53l0x_pair_init_error(void);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_PAIR_H */
