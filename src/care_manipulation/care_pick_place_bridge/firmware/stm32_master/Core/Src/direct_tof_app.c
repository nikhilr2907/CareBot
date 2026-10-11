/* Direct VL53L0X pair application
 * ---------------------------------------------------------------------------
 * Status prints, benchmark state machine, I2C2 bus scan and recovery routines
 * for the two ToF sensors wired directly to the STM32 (SDA=PB3, SCL=PB10,
 * XSHUT pins PA5/PD2). The driver lives in Core/Src/vl53l0x_pair.c; the code
 * here only talks to that driver and to the HAL. */

#include "direct_tof_app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_pair.h"

/* Owned by main.c -- the HAL I2C handle and its init routine. The init is
 * re-run during bus recovery to re-arm the peripheral after a cable glitch. */
extern I2C_HandleTypeDef hi2c2;
extern void MX_I2C2_Init(void);

/* ===========================================================================
 *                                 Tunables
 * ========================================================================= */

#define DIRECT_TOF_BENCH_ENABLE    0
#define DIRECT_TOF_PRINT_PERIOD_MS 1000u
#define DIRECT_TOF_RETRY_PERIOD_MS 10000u
#define DIRECT_TOF_BENCH_WARMUP_MS 2000u
#define DIRECT_TOF_BENCH_RUN_MS   15000u

/* ===========================================================================
 *                                   State
 * ========================================================================= */

static bool direct_tof_ok = false;
static uint32_t direct_tof_last_print_ms = 0u;
static uint32_t direct_tof_last_retry_ms = 0u;

typedef struct {
    const char *name;
    uint32_t timing_budget_us;
    uint32_t intershot_gap_ms;
} direct_tof_bench_profile_t;

static const direct_tof_bench_profile_t direct_tof_bench_profiles[] = {
    { "BASELINE", 33000u, 70u },
    { "NO_GAP",   33000u,  0u },
    { "FAST",     20000u,  0u },
    { "ACCURACY", 66000u,  0u },
};

typedef struct {
    uint32_t last_sample_counter;
    uint32_t start_error_counter;
    uint32_t samples;
    uint32_t valid;
    uint32_t invalid;
    uint32_t first_update_ms;
    uint32_t last_update_ms;
    uint32_t interval_count;
    uint32_t interval_sum_ms;
    uint32_t interval_min_ms;
    uint32_t interval_max_ms;
    uint64_t distance_sum;
    uint64_t distance_sq_sum;
    uint64_t absolute_step_sum;
    uint32_t absolute_step_count;
    uint16_t distance_min;
    uint16_t distance_max;
    uint16_t previous_valid_mm;
    bool have_valid;
} direct_tof_bench_sensor_t;

typedef enum {
    DIRECT_TOF_BENCH_WAITING = 0,
    DIRECT_TOF_BENCH_WARMUP,
    DIRECT_TOF_BENCH_RUNNING,
    DIRECT_TOF_BENCH_COMPLETE
} direct_tof_bench_state_t;

static direct_tof_bench_sensor_t direct_tof_bench_left;
static direct_tof_bench_sensor_t direct_tof_bench_right;
static direct_tof_bench_state_t direct_tof_bench_state =
    DIRECT_TOF_BENCH_WAITING;
static uint8_t direct_tof_bench_profile_index = 0u;
static uint32_t direct_tof_bench_stage_started_ms = 0u;

/* ===========================================================================
 *                              Status printers
 * ========================================================================= */

static void direct_tof_print_status(uint32_t now_ms)
{
    vl53l0x_pair_snapshot_t tof;
    char left[24];
    char right[24];
    vl53l0x_pair_get_snapshot(&tof);

    if (!tof.left_online) {
        snprintf(left, sizeof(left), "OFF");
    } else if (tof.left_samples == 0u) {
        snprintf(left, sizeof(left), "WAIT e=%lu",
                 (unsigned long)tof.left_errors);
    } else {
        snprintf(left, sizeof(left), "a%u r%u n%u s%u",
                 (unsigned)tof.left_mm,
                 (unsigned)tof.left_raw_mm,
                 (unsigned)tof.left_average_samples,
                 (unsigned)tof.left_range_status);
    }

    if (!tof.right_online) {
        snprintf(right, sizeof(right), "OFF");
    } else if (tof.right_samples == 0u) {
        snprintf(right, sizeof(right), "WAIT e=%lu",
                 (unsigned long)tof.right_errors);
    } else {
        snprintf(right, sizeof(right), "a%u r%u n%u s%u",
                 (unsigned)tof.right_mm,
                 (unsigned)tof.right_raw_mm,
                 (unsigned)tof.right_average_samples,
                 (unsigned)tof.right_range_status);
    }

    dbg_printf("XTOF L=[%s] R=[%s] samples=%lu/%lu errors=%lu/%lu age=%lu/%lums\r\n",
               left, right,
               (unsigned long)tof.left_samples,
               (unsigned long)tof.right_samples,
               (unsigned long)tof.left_errors,
               (unsigned long)tof.right_errors,
               tof.left_samples > 0u
                   ? (unsigned long)(now_ms - tof.left_last_update_ms) : 0ul,
               tof.right_samples > 0u
                   ? (unsigned long)(now_ms - tof.right_last_update_ms) : 0ul);
}

void direct_tof_print_physical_right(uint32_t now_ms)
{
    vl53l0x_pair_snapshot_t tof;
    vl53l0x_pair_get_snapshot(&tof);

    /* The installed D13/PA5 branch is physically on the robot's right side.
     * It remains the legacy `left` slot inside vl53l0x_pair.c. */
    if (!tof.left_online) {
        dbg_printf("TOF PHYSICAL_RIGHT OFF pin=D13/PA5 stage=\"%s\" error=%d\r\n",
                   vl53l0x_pair_init_stage(), vl53l0x_pair_init_error());
        return;
    }

    if (tof.left_samples == 0u) {
        dbg_printf("TOF PHYSICAL_RIGHT WAIT pin=D13/PA5 samples=0 errors=%lu\r\n",
                   (unsigned long)tof.left_errors);
        return;
    }

    dbg_printf("TOF PHYSICAL_RIGHT %u mm raw=%u mm avg_n=%u status=%u "
               "samples=%lu errors=%lu age=%lums pin=D13/PA5\r\n",
               (unsigned)tof.left_mm,
               (unsigned)tof.left_raw_mm,
               (unsigned)tof.left_average_samples,
               (unsigned)tof.left_range_status,
               (unsigned long)tof.left_samples,
               (unsigned long)tof.left_errors,
               (unsigned long)(now_ms - tof.left_last_update_ms));
}

/* ===========================================================================
 *                        Benchmark profile sweep (opt-in)
 * ========================================================================= */

static void direct_tof_bench_sensor_reset(direct_tof_bench_sensor_t *stats,
                                          uint32_t sample_counter,
                                          uint32_t error_counter)
{
    memset(stats, 0, sizeof(*stats));
    stats->last_sample_counter = sample_counter;
    stats->start_error_counter = error_counter;
    stats->interval_min_ms = UINT32_MAX;
    stats->distance_min = UINT16_MAX;
}

static void direct_tof_bench_reset(const vl53l0x_pair_snapshot_t *tof)
{
    direct_tof_bench_sensor_reset(&direct_tof_bench_left,
                                  tof->left_samples, tof->left_errors);
    direct_tof_bench_sensor_reset(&direct_tof_bench_right,
                                  tof->right_samples, tof->right_errors);
}

static void direct_tof_bench_observe_sensor(
    direct_tof_bench_sensor_t *stats,
    uint32_t sample_counter,
    uint32_t update_ms,
    uint16_t raw_mm,
    uint8_t range_status)
{
    if (sample_counter == stats->last_sample_counter) return;

    uint32_t new_samples = sample_counter - stats->last_sample_counter;
    stats->last_sample_counter = sample_counter;
    stats->samples += new_samples;

    if (stats->first_update_ms == 0u) {
        stats->first_update_ms = update_ms;
    } else if (update_ms != stats->last_update_ms) {
        uint32_t interval_ms = update_ms - stats->last_update_ms;
        stats->interval_sum_ms += interval_ms;
        stats->interval_count++;
        if (interval_ms < stats->interval_min_ms) {
            stats->interval_min_ms = interval_ms;
        }
        if (interval_ms > stats->interval_max_ms) {
            stats->interval_max_ms = interval_ms;
        }
    }
    stats->last_update_ms = update_ms;

    if (range_status != 0u || raw_mm == UINT16_MAX || new_samples != 1u) {
        stats->invalid += new_samples;
        return;
    }

    stats->valid++;
    stats->distance_sum += raw_mm;
    stats->distance_sq_sum += (uint64_t)raw_mm * (uint64_t)raw_mm;
    if (raw_mm < stats->distance_min) stats->distance_min = raw_mm;
    if (raw_mm > stats->distance_max) stats->distance_max = raw_mm;

    if (stats->have_valid) {
        uint16_t delta = raw_mm >= stats->previous_valid_mm
                             ? (uint16_t)(raw_mm - stats->previous_valid_mm)
                             : (uint16_t)(stats->previous_valid_mm - raw_mm);
        stats->absolute_step_sum += delta;
        stats->absolute_step_count++;
    }
    stats->previous_valid_mm = raw_mm;
    stats->have_valid = true;
}

static double direct_tof_bench_sqrt(double value)
{
    if (value <= 0.0) return 0.0;

    double estimate = value >= 1.0 ? value : 1.0;
    for (uint8_t iteration = 0u; iteration < 12u; ++iteration) {
        estimate = 0.5 * (estimate + value / estimate);
    }
    return estimate;
}

static void direct_tof_bench_print_sensor(
    const char *side,
    const direct_tof_bench_sensor_t *stats,
    uint32_t current_error_counter)
{
    double rate_hz = 0.0;
    if (stats->samples > 1u &&
        stats->last_update_ms > stats->first_update_ms) {
        rate_hz = ((double)(stats->samples - 1u) * 1000.0) /
                  (double)(stats->last_update_ms - stats->first_update_ms);
    }

    double valid_percent = stats->samples > 0u
                               ? (100.0 * (double)stats->valid /
                                  (double)stats->samples)
                               : 0.0;
    double mean_mm = stats->valid > 0u
                         ? (double)stats->distance_sum / (double)stats->valid
                         : 0.0;
    double variance = stats->valid > 0u
                          ? (double)stats->distance_sq_sum /
                                (double)stats->valid - mean_mm * mean_mm
                          : 0.0;
    if (variance < 0.0) variance = 0.0;
    double stddev_mm = direct_tof_bench_sqrt(variance);
    double mean_step_mm = stats->absolute_step_count > 0u
                              ? (double)stats->absolute_step_sum /
                                    (double)stats->absolute_step_count
                              : 0.0;
    double mean_interval_ms = stats->interval_count > 0u
                                  ? (double)stats->interval_sum_ms /
                                        (double)stats->interval_count
                                  : 0.0;

    if (stats->valid == 0u) {
        dbg_printf("BENCH %s samples=%lu valid=0 rate=%.2fHz invalid=%lu errors=+%lu\r\n",
                   side,
                   (unsigned long)stats->samples,
                   rate_hz,
                   (unsigned long)stats->invalid,
                   (unsigned long)(current_error_counter -
                                   stats->start_error_counter));
        return;
    }

    dbg_printf("BENCH %s n=%lu valid=%lu(%.1f%%) rate=%.2fHz mean=%.2f sd=%.2f min/max=%u/%u step=%.2f dt=%lu/%.1f/%lu errors=+%lu\r\n",
               side,
               (unsigned long)stats->samples,
               (unsigned long)stats->valid,
               valid_percent,
               rate_hz,
               mean_mm,
               stddev_mm,
               (unsigned)stats->distance_min,
               (unsigned)stats->distance_max,
               mean_step_mm,
               (unsigned long)(stats->interval_min_ms == UINT32_MAX
                                   ? 0u : stats->interval_min_ms),
               mean_interval_ms,
               (unsigned long)stats->interval_max_ms,
               (unsigned long)(current_error_counter -
                               stats->start_error_counter));
}

static bool direct_tof_bench_start_profile(uint8_t profile_index,
                                           uint32_t now_ms)
{
    const direct_tof_bench_profile_t *profile =
        &direct_tof_bench_profiles[profile_index];
    bool configured = vl53l0x_pair_set_profile(profile->timing_budget_us,
                                               profile->intershot_gap_ms);
    dbg_printf("\r\nBENCH START %s budget=%lums gap=%lums warmup=%lums run=%lums configured=%s\r\n",
               profile->name,
               (unsigned long)(profile->timing_budget_us / 1000u),
               (unsigned long)profile->intershot_gap_ms,
               (unsigned long)DIRECT_TOF_BENCH_WARMUP_MS,
               (unsigned long)DIRECT_TOF_BENCH_RUN_MS,
               configured ? "YES" : "NO");
    if (!configured) return false;

    direct_tof_bench_stage_started_ms = now_ms;
    direct_tof_bench_state = DIRECT_TOF_BENCH_WARMUP;
    return true;
}

static void direct_tof_bench_service(uint32_t now_ms)
{
    vl53l0x_pair_snapshot_t tof;
    vl53l0x_pair_get_snapshot(&tof);

    if (!tof.left_online || !tof.right_online) {
        if (direct_tof_bench_state != DIRECT_TOF_BENCH_WAITING &&
            direct_tof_bench_state != DIRECT_TOF_BENCH_COMPLETE) {
            dbg_printf("BENCH ABORT sensor offline; waiting for pair recovery\r\n");
        }
        if (direct_tof_bench_state != DIRECT_TOF_BENCH_COMPLETE) {
            direct_tof_bench_state = DIRECT_TOF_BENCH_WAITING;
            direct_tof_bench_profile_index = 0u;
        }
        return;
    }

    if (direct_tof_bench_state == DIRECT_TOF_BENCH_WAITING) {
        (void)direct_tof_bench_start_profile(0u, now_ms);
        return;
    }

    if (direct_tof_bench_state == DIRECT_TOF_BENCH_WARMUP) {
        if ((now_ms - direct_tof_bench_stage_started_ms) <
            DIRECT_TOF_BENCH_WARMUP_MS) {
            return;
        }
        direct_tof_bench_reset(&tof);
        direct_tof_bench_stage_started_ms = now_ms;
        direct_tof_bench_state = DIRECT_TOF_BENCH_RUNNING;
        dbg_printf("BENCH COLLECT %s\r\n",
                   direct_tof_bench_profiles[
                       direct_tof_bench_profile_index].name);
        return;
    }

    if (direct_tof_bench_state != DIRECT_TOF_BENCH_RUNNING) return;

    direct_tof_bench_observe_sensor(&direct_tof_bench_left,
                                    tof.left_samples,
                                    tof.left_last_update_ms,
                                    tof.left_raw_mm,
                                    tof.left_range_status);
    direct_tof_bench_observe_sensor(&direct_tof_bench_right,
                                    tof.right_samples,
                                    tof.right_last_update_ms,
                                    tof.right_raw_mm,
                                    tof.right_range_status);

    if ((now_ms - direct_tof_bench_stage_started_ms) <
        DIRECT_TOF_BENCH_RUN_MS) {
        return;
    }

    dbg_printf("BENCH RESULT %s budget=%lums gap=%lums\r\n",
               direct_tof_bench_profiles[direct_tof_bench_profile_index].name,
               (unsigned long)(tof.timing_budget_us / 1000u),
               (unsigned long)tof.intershot_gap_ms);
    direct_tof_bench_print_sensor("LEFT", &direct_tof_bench_left,
                                  tof.left_errors);
    direct_tof_bench_print_sensor("RIGHT", &direct_tof_bench_right,
                                  tof.right_errors);

    direct_tof_bench_profile_index++;
    if (direct_tof_bench_profile_index <
        (sizeof(direct_tof_bench_profiles) /
         sizeof(direct_tof_bench_profiles[0]))) {
        (void)direct_tof_bench_start_profile(direct_tof_bench_profile_index,
                                             now_ms);
        return;
    }

    (void)vl53l0x_pair_set_profile(33000u, 0u);
    direct_tof_bench_state = DIRECT_TOF_BENCH_COMPLETE;
    dbg_printf("BENCH COMPLETE; continuing budget=33ms gap=0ms\r\n\r\n");
}

/* ===========================================================================
 *                           Bus scan and recovery
 * ========================================================================= */

static void direct_tof_scan_bus(void)
{
    char addresses[96];
    size_t used = 0u;
    unsigned found = 0u;

    addresses[0] = '\0';
    for (uint8_t address = 0x08u; address <= 0x77u; ++address) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(address << 1),
                                  1u, 5u) != HAL_OK) {
            continue;
        }

        int written = snprintf(addresses + used, sizeof(addresses) - used,
                               "%s0x%02X", found == 0u ? "" : " ",
                               (unsigned)address);
        if (written > 0 && (size_t)written < sizeof(addresses) - used) {
            used += (size_t)written;
        }
        found++;
    }

    unsigned left_mode = (unsigned)((GPIOA->MODER >> (5u * 2u)) & 0x3u);
    unsigned left_type = (unsigned)((GPIOA->OTYPER >> 5u) & 0x1u);
    unsigned left_odr = (unsigned)((GPIOA->ODR >> 5u) & 0x1u);
    unsigned left_idr = (unsigned)((GPIOA->IDR >> 5u) & 0x1u);
    unsigned right_mode = (unsigned)((GPIOD->MODER >> (2u * 2u)) & 0x3u);
    unsigned right_type = (unsigned)((GPIOD->OTYPER >> 2u) & 0x1u);
    unsigned right_odr = (unsigned)((GPIOD->ODR >> 2u) & 0x1u);
    unsigned right_idr = (unsigned)((GPIOD->IDR >> 2u) & 0x1u);

    if (found == 0u) {
        dbg_printf(">> I2C2 scan: no ACK; SDA=%u SCL=%u D13/L m%u t%u c%u s%u PD2/R m%u t%u c%u s%u state=0x%02X err=0x%08lX\r\n",
                   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET,
                   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET,
                   left_mode, left_type, left_odr, left_idr,
                   right_mode, right_type, right_odr, right_idr,
                   (unsigned)HAL_I2C_GetState(&hi2c2),
                   (unsigned long)HAL_I2C_GetError(&hi2c2));
    } else {
        dbg_printf(">> I2C2 scan: %u ACK(s): %s; D13/L m%u t%u c%u s%u PD2/R m%u t%u c%u s%u\r\n",
                   found, addresses,
                   left_mode, left_type, left_odr, left_idr,
                   right_mode, right_type, right_odr, right_idr);
    }
}

static bool direct_tof_recover_bus(void)
{
    GPIO_InitTypeDef gpio = {0};

    (void)HAL_I2C_DeInit(&hi2c2);

    /* Preload both open-drain outputs as released before changing their mode. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_Delay(1u);

    for (unsigned pulse = 0u;
         pulse < 9u && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET;
         ++pulse) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1u);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1u);
    }

    /* Generate an I2C STOP: SDA low, release SCL, then release SDA. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(1u);

    bool released =
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET &&
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET;

    MX_I2C2_Init();
    return released;
}

/* Strong override of the __weak default in Drivers/VL53L0X/vl53l0x_platform.c.
 * The platform layer calls this after a bus-error so the pair driver can keep
 * running without a reset. */
int VL53L0X_I2C_Recover(I2C_HandleTypeDef *i2c)
{
    if (i2c != &hi2c2) return -1;
    return direct_tof_recover_bus() ? 0 : -1;
}

/* ===========================================================================
 *                          Public bringup and service
 * ========================================================================= */

void direct_tof_bringup(void)
{
    dbg_printf("\r\n>> Bringing up direct VL53L0X pair on I2C2...\r\n");
    direct_tof_ok = vl53l0x_pair_init(&hi2c2);
    dbg_printf(">> VL53L0X %s: %s (error=%d)\r\n",
               direct_tof_ok ? "ONLINE" : "OFFLINE",
               vl53l0x_pair_init_stage(), vl53l0x_pair_init_error());
    dbg_printf(">> I2C2 recovery after initial probe: %s\r\n",
               direct_tof_recover_bus() ? "SDA/SCL released" : "line still held LOW");
    direct_tof_scan_bus();
    dbg_printf(">> ToF wiring: SDA=D3/PB3, SCL=D6/PB10, left XSHUT=D13/PA5, right XSHUT=PD2\r\n");
    dbg_printf(">> Long-cable profile: I2C=10kHz, alternating shots, 120ms quiet gap\r\n");
}

void direct_tof_service(uint32_t now_ms)
{
#if DIRECT_TOF_TEST_MODE
    vl53l0x_pair_snapshot_t tof;
    vl53l0x_pair_get_snapshot(&tof);
    if ((!tof.left_online || !tof.right_online) &&
        (now_ms - direct_tof_last_retry_ms) >= DIRECT_TOF_RETRY_PERIOD_MS) {
        direct_tof_last_retry_ms = now_ms;
        dbg_printf(">> I2C2 recovery: %s\r\n",
                   direct_tof_recover_bus() ? "SDA/SCL released"
                                            : "line still held LOW");
        direct_tof_ok = vl53l0x_pair_init(&hi2c2);
        dbg_printf(">> ToF rescan: %s - %s (error=%d)\r\n",
                   direct_tof_ok ? "FOUND" : "not found",
                   vl53l0x_pair_init_stage(), vl53l0x_pair_init_error());
        direct_tof_scan_bus();
    }
#if DIRECT_TOF_BENCH_ENABLE
    direct_tof_bench_service(now_ms);
#endif
    if ((now_ms - direct_tof_last_print_ms) >= DIRECT_TOF_PRINT_PERIOD_MS) {
        direct_tof_last_print_ms = now_ms;
        direct_tof_print_status(now_ms);
    }
#else
    (void)now_ms;
#endif
}
