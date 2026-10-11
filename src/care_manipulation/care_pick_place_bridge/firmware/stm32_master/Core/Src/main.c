/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>   /* strtol */
#include <string.h>   /* strncpy */
#include "motors.h"
#include "bld515c.h"
#include "bno055.h"
#include "hx711_parallel.h"
#include "lac_actuators.h"
#include "vl53l0x_pair.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* ESP32 sensor-node link state, updated from RS485 UARTs.
 *
 * Both slave nodes use one length-prefixed frame shape:
 *   AA 55 LEN CMD PAYLOAD... CS
 *
 * CS is the low byte of the sum of LEN, CMD and all payload bytes.
 * cmd=0x01: 4x VL53L0X point distances + 1 ultrasonic distance.
 * cmd=0x02: 3x VL53L5CX 4x4 grids + online/stop/fan status.
 */
typedef enum {
    SENSOR_SRC_NONE = 0,
    SENSOR_SRC_USART3
} sensor_source_t;

#define SENSOR_VL53L5_COUNT        3u
#define SENSOR_VL53L5_ZONES        16u

typedef struct {
    uint16_t tof_mm[4];
    uint16_t ultra_mm;
    sensor_source_t source;
    uint32_t last_update_ms;
    uint32_t valid_frames;
    uint32_t bad_checksum_count;
    uint32_t bad_format_count;
    uint32_t rx_bytes;       /* raw bytes consumed -- 0 = USART3/RS485 wire dead */
} sensor_node_t;

typedef struct {
    uint16_t grid_mm[SENSOR_VL53L5_COUNT][SENSOR_VL53L5_ZONES];
    uint8_t online_mask;
    uint8_t stop_flags;
    uint8_t fan_duty_percent;
    sensor_source_t source;
    uint32_t last_update_ms;
    uint32_t valid_frames;
} vl53l5_node_t;
 
static sensor_node_t sensor_node = {
    { 9999u, 9999u, 9999u, 9999u },
    9999u,
    SENSOR_SRC_NONE,
    0u, 0u, 0u, 0u, 0u
};

static vl53l5_node_t vl53l5_node = {0};
 
#define SENSOR_NODE_TIMEOUT_MS      300u
#define SENSOR_FRAME_MAX_PAYLOAD    (3u + (SENSOR_VL53L5_COUNT * SENSOR_VL53L5_ZONES * 2u))
#define SENSOR_FRAME_MAX_LEN        (SENSOR_FRAME_MAX_PAYLOAD + 5u)
#define SENSOR_CMD_VL53L0X_ULTRA    0x01u
#define SENSOR_CMD_VL53L5_GRID      0x02u
#define SENSOR_VL53L0X_PAYLOAD_LEN  10u
#define SENSOR_VL53L5_PAYLOAD_LEN   SENSOR_FRAME_MAX_PAYLOAD
#define SENSOR_NODE_INVALID_MM      9999u
#define SENSOR_GRID_INVALID_MM      0xFFFFu
#define SENSOR_STOP_THRESHOLD_MM    250u
#define SENSOR_PRINT_PERIOD_MS      250u
 
/* Circular DMA RX buffer for the USART3 RS485 sensor bus. */
#define USART3_RX_DMA_BUF_SIZE  512u
static uint8_t  usart3_rx_dma_buf[USART3_RX_DMA_BUF_SIZE];
static uint32_t usart3_rx_dma_read_pos = 0;
 
/* Test command interface on USART2 (ST-Link VCP for now; Raspberry Pi later).
 * One byte at a time via interrupt. The ISR accumulates into cmd_buf until it
 * sees '\n' or '\r', then sets cmd_ready=true. While that flag is true the ISR
 * drops new chars so main() has a stable buffer to read; main() clears the
 * flag after copying out the line. */
#define CMD_BUF_SIZE 64
static volatile char    cmd_buf[CMD_BUF_SIZE];
static volatile uint8_t cmd_idx   = 0;
static volatile bool    cmd_ready = false;
static volatile uint8_t usart2_rx_byte;
static volatile uint32_t usart2_rx_count = 0u;
static volatile uint32_t usart2_line_count = 0u;
static volatile uint32_t disarmed_reject_count = 0u;

/* Keep USART2 readable during manual motor testing. */
static bool terminal_log_enabled = false;

/* Raw load-cell diagnostics are separately rate-limited so they can be used
 * without enabling the much noisier range-sensor and IMU log stream. */
#define LOAD_CELL_PRINT_PERIOD_MS 250u
static bool load_cell_stream_enabled = false;
static uint32_t load_cell_last_print_ms = 0u;

/* Two active-low mechanical/limit-switch inputs. The internal pull-ups keep
 * open contacts at a defined HIGH level; a closed contact pulls the pin LOW. */
#define SWITCH_1_GPIO_PORT       GPIOC
#define SWITCH_1_GPIO_PIN        GPIO_PIN_6
#define SWITCH_2_GPIO_PORT       GPIOC
#define SWITCH_2_GPIO_PIN        GPIO_PIN_9
#define SWITCH_PRINT_PERIOD_MS   250u
static bool switch_stream_enabled = false;
static uint32_t switch_last_print_ms = 0u;
static uint32_t switch_pull_mode = GPIO_PULLUP;

/* BNO055 IMU state. Polled from the main loop when the poll period has
 * elapsed. imu_ok is set by bno055_init(); if false, the poll is skipped
 * so a missing IMU doesn't generate hundreds of failed I2C transactions
 * per second. Print is rate-limited separately so logs don't flood. */
#define IMU_POLL_PERIOD_MS  10u    /* 10 ms = 100 Hz target               */
#define IMU_PRINT_PERIOD_MS 500u   /* print one summary line every 500 ms */
static bno055_euler_t imu_euler         = {0};
static bno055_calib_t imu_calib         = {0};
static bool           imu_ok            = false;
static uint32_t       imu_read_count    = 0;
static uint32_t       imu_err_count     = 0;
static uint32_t       imu_last_poll_ms  = 0;
static uint32_t       imu_last_print_ms = 0;

/* Keep motion outputs disabled while validating the newly wired direct ToF
 * pair. Change to 0 after the sensor installation is proven. */
#define DIRECT_TOF_TEST_MODE       0
#define DIRECT_TOF_LONG_CABLE_MODE 1
#define Y_LIFT_TEST_MODE           0
#define DIRECT_TOF_BENCH_ENABLE    0
#define DIRECT_TOF_PRINT_PERIOD_MS 1000u
#define DIRECT_TOF_RETRY_PERIOD_MS 10000u
#define DIRECT_TOF_BENCH_WARMUP_MS 2000u
#define DIRECT_TOF_BENCH_RUN_MS   15000u
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

/* Timed two-motor lift command used by the local calibration console. M2 is
 * the right Y axis and M5 is the left Y axis. The board owns the timeout so
 * the outputs stop even if the browser, host process, or USB link disappears. */
#define LIFT_M2_INDEX             1
#define LIFT_M5_INDEX             4
#define LIFT_MIN_HZ             100u
#define LIFT_MAX_HZ            5000u
#define LIFT_MIN_DURATION_MS     100u
#define LIFT_MAX_DURATION_MS   10000u
static bool     lift_timed_active = false;
static uint32_t lift_started_ms   = 0u;
static uint32_t lift_duration_ms  = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
/* Lightweight printf-over-USART2 for debug. Blocks for up to 100 ms while the
 * line drains -- fine for periodic status prints, don't call from ISRs. */
static void dbg_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
}

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

static void direct_tof_print_physical_right(uint32_t now_ms)
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

int VL53L0X_I2C_Recover(I2C_HandleTypeDef *i2c)
{
    if (i2c != &hi2c2) return -1;
    return direct_tof_recover_bus() ? 0 : -1;
}

typedef struct {
    uint8_t frame[SENSOR_FRAME_MAX_LEN];
    uint16_t idx;
    uint16_t expected_len;
} sensor_parser_t;

static sensor_parser_t sensor_parser_usart3 = { {0}, 0u, 0u };

static const char *sensor_source_name(sensor_source_t source)
{
    switch (source) {
    case SENSOR_SRC_USART3: return "USART3";
    default:                return "none";
    }
}

static uint16_t sensor_get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static bool sensor_distance_valid(uint16_t mm)
{
    return (mm != 0u) && (mm != SENSOR_NODE_INVALID_MM) && (mm < 8000u);
}

static bool grid_distance_valid(uint16_t mm)
{
    return (mm != 0u) && (mm != SENSOR_GRID_INVALID_MM) && (mm < 8000u);
}

static void sensor_format_mm(char *out, size_t out_sz, uint16_t mm)
{
    if (sensor_distance_valid(mm)) {
        snprintf(out, out_sz, "%u", (unsigned)mm);
    } else {
        snprintf(out, out_sz, "----");
    }
}

static uint8_t sensor_checksum(const uint8_t *frame, uint16_t last_payload_idx)
{
    uint8_t sum = 0u;
    for (uint16_t i = 2u; i <= last_payload_idx; ++i) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return sum;
}

static void sensor_node_accept_vl53l0x_frame(const uint8_t *frame,
                                             sensor_source_t source)
{
    for (uint8_t i = 0; i < 4; ++i) {
        sensor_node.tof_mm[i] = sensor_get_u16_be(&frame[4 + (i * 2)]);
    }
    sensor_node.ultra_mm       = sensor_get_u16_be(&frame[12]);
    sensor_node.source         = source;
    sensor_node.last_update_ms = HAL_GetTick();
    sensor_node.valid_frames++;
}

static void sensor_node_accept_vl53l5_frame(const uint8_t *frame,
                                            sensor_source_t source)
{
    const uint8_t *payload = &frame[4];
    vl53l5_node.online_mask      = payload[0];
    vl53l5_node.stop_flags       = payload[1];
    vl53l5_node.fan_duty_percent = payload[2];

    uint16_t p = 3u;
    for (uint8_t sensor = 0; sensor < SENSOR_VL53L5_COUNT; ++sensor) {
        for (uint8_t zone = 0; zone < SENSOR_VL53L5_ZONES; ++zone) {
            vl53l5_node.grid_mm[sensor][zone] = sensor_get_u16_be(&payload[p]);
            p = (uint16_t)(p + 2u);
        }
    }

    vl53l5_node.source         = source;
    vl53l5_node.last_update_ms = HAL_GetTick();
    vl53l5_node.valid_frames++;
}

static void sensor_node_accept_frame(const uint8_t *frame,
                                     uint16_t frame_len,
                                     sensor_source_t source)
{
    if (frame_len < 5u) {
        sensor_node.bad_format_count++;
        return;
    }

    uint8_t payload_len = frame[2];
    uint8_t cmd = frame[3];
    if (frame_len != (uint16_t)(payload_len + 5u)) {
        sensor_node.bad_format_count++;
        return;
    }

    if (sensor_checksum(frame, (uint16_t)(frame_len - 2u)) != frame[frame_len - 1u]) {
        sensor_node.bad_checksum_count++;
        return;
    }

    if (cmd == SENSOR_CMD_VL53L0X_ULTRA && payload_len == SENSOR_VL53L0X_PAYLOAD_LEN) {
        sensor_node_accept_vl53l0x_frame(frame, source);
    } else if (cmd == SENSOR_CMD_VL53L5_GRID && payload_len == SENSOR_VL53L5_PAYLOAD_LEN) {
        sensor_node_accept_vl53l5_frame(frame, source);
    } else {
        sensor_node.bad_format_count++;
    }
}

static void sensor_node_feed(sensor_parser_t *parser, sensor_source_t source, uint8_t b)
{
    sensor_node.rx_bytes++;

    if (parser->idx == 0u) {
        if (b == 0xAAu) {
            parser->frame[parser->idx++] = b;
        }
        return;
    }

    if (parser->idx == 1u) {
        if (b == 0x55u) {
            parser->frame[parser->idx++] = b;
        } else if (b == 0xAAu) {
            parser->frame[0] = b;
            parser->idx = 1u;
        } else {
            parser->idx = 0u;
        }
        return;
    }

    if (parser->idx == 2u) {
        parser->frame[parser->idx++] = b;
        if (b > SENSOR_FRAME_MAX_PAYLOAD) {
            sensor_node.bad_format_count++;
            parser->idx = 0u;
            parser->expected_len = 0u;
        } else {
            parser->expected_len = (uint16_t)b + 5u;
        }
        return;
    }

    parser->frame[parser->idx++] = b;
    if (parser->idx >= parser->expected_len) {
        sensor_node_accept_frame(parser->frame, parser->expected_len, source);
        parser->idx = 0u;
        parser->expected_len = 0u;
    }
}

static void sensor_node_service_usart3(void)
{
    uint32_t write_pos = USART3_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    while (usart3_rx_dma_read_pos != write_pos) {
        sensor_node_feed(&sensor_parser_usart3,
                         SENSOR_SRC_USART3,
                         usart3_rx_dma_buf[usart3_rx_dma_read_pos]);
        usart3_rx_dma_read_pos++;
        if (usart3_rx_dma_read_pos >= USART3_RX_DMA_BUF_SIZE) {
            usart3_rx_dma_read_pos = 0u;
        }
    }
}

static bool sensor_node_alive(void)
{
    return (sensor_node.valid_frames > 0u) &&
           ((HAL_GetTick() - sensor_node.last_update_ms) < SENSOR_NODE_TIMEOUT_MS);
}

static bool vl53l5_node_alive(void)
{
    return (vl53l5_node.valid_frames > 0u) &&
           ((HAL_GetTick() - vl53l5_node.last_update_ms) < SENSOR_NODE_TIMEOUT_MS);
}

static bool sensor_node_obstacle_stop(void)
{
    if (!sensor_node_alive() && !vl53l5_node_alive()) {
        return true;
    }

    if (sensor_node_alive()) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (sensor_distance_valid(sensor_node.tof_mm[i]) &&
                sensor_node.tof_mm[i] < SENSOR_STOP_THRESHOLD_MM) {
                return true;
            }
        }
        if (sensor_distance_valid(sensor_node.ultra_mm) &&
            sensor_node.ultra_mm < SENSOR_STOP_THRESHOLD_MM) {
            return true;
        }
    }

    return vl53l5_node_alive() && (vl53l5_node.stop_flags != 0u);
}

static uint16_t vl53l5_min_for_sensor(uint8_t sensor)
{
    uint16_t min_mm = SENSOR_GRID_INVALID_MM;
    if (sensor >= SENSOR_VL53L5_COUNT) {
        return min_mm;
    }

    for (uint8_t zone = 0; zone < SENSOR_VL53L5_ZONES; ++zone) {
        uint16_t mm = vl53l5_node.grid_mm[sensor][zone];
        if (grid_distance_valid(mm) &&
            (min_mm == SENSOR_GRID_INVALID_MM || mm < min_mm)) {
            min_mm = mm;
        }
    }
    return min_mm;
}

static void sensor_node_emit_status(uint32_t now,
                                    uint32_t *last_print_ms,
                                    uint32_t *last_valid_frames,
                                    uint32_t *last_rx_bytes)
{
    if ((now - *last_print_ms) < SENSOR_PRINT_PERIOD_MS) {
        return;
    }

    uint32_t dt = (now > *last_print_ms) ? (now - *last_print_ms) : SENSOR_PRINT_PERIOD_MS;
    if (*last_print_ms == 0u) {
        dt = SENSOR_PRINT_PERIOD_MS;
    }

    uint32_t frames_delta = sensor_node.valid_frames - *last_valid_frames;
    uint32_t rx_delta     = sensor_node.rx_bytes - *last_rx_bytes;
    uint32_t fps10        = (dt > 0u) ? ((frames_delta * 10000u) / dt) : 0u;
    uint32_t bps          = (dt > 0u) ? ((rx_delta * 1000u) / dt) : 0u;
    const char *state     = sensor_node_alive() ? "LIVE" :
                            (sensor_node.valid_frames > 0u ? "TIMEOUT" : "WAIT");

    char t0[8], t1[8], t2[8], t3[8], ultra[8];
    sensor_format_mm(t0, sizeof t0, sensor_node.tof_mm[0]);
    sensor_format_mm(t1, sizeof t1, sensor_node.tof_mm[1]);
    sensor_format_mm(t2, sizeof t2, sensor_node.tof_mm[2]);
    sensor_format_mm(t3, sizeof t3, sensor_node.tof_mm[3]);
    sensor_format_mm(ultra, sizeof ultra, sensor_node.ultra_mm);

    if (sensor_node.valid_frames == 0u) {
        dbg_printf("SENSOR %s src=%s frames=0 rx=%luB/s raw=%lu bad=%lu/%lu ToF=[%s %s %s %s] Ultra=%s\r\n",
                   state,
                   sensor_source_name(sensor_node.source),
                   (unsigned long)bps,
                   (unsigned long)sensor_node.rx_bytes,
                   (unsigned long)sensor_node.bad_checksum_count,
                   (unsigned long)sensor_node.bad_format_count,
                   t0, t1, t2, t3, ultra);
    } else {
        dbg_printf("SENSOR %s src=%s age=%lums frames=%lu fps=%lu.%01lu rx=%luB/s bad=%lu/%lu stop=%u ToF=[%s %s %s %s] Ultra=%s\r\n",
                   state,
                   sensor_source_name(sensor_node.source),
                   (unsigned long)(now - sensor_node.last_update_ms),
                   (unsigned long)sensor_node.valid_frames,
                   (unsigned long)(fps10 / 10u),
                   (unsigned long)(fps10 % 10u),
                   (unsigned long)bps,
                   (unsigned long)sensor_node.bad_checksum_count,
                   (unsigned long)sensor_node.bad_format_count,
                   sensor_node_obstacle_stop() ? 1u : 0u,
                   t0, t1, t2, t3, ultra);
    }

    *last_print_ms     = now;
    *last_valid_frames = sensor_node.valid_frames;
    *last_rx_bytes     = sensor_node.rx_bytes;

    const char *l5_state = vl53l5_node_alive() ? "LIVE" :
                           (vl53l5_node.valid_frames > 0u ? "TIMEOUT" : "WAIT");
    char l5_min[SENSOR_VL53L5_COUNT][8];
    for (uint8_t i = 0; i < SENSOR_VL53L5_COUNT; ++i) {
        uint16_t mm = vl53l5_min_for_sensor(i);
        if (mm == SENSOR_GRID_INVALID_MM) {
            snprintf(l5_min[i], sizeof l5_min[i], "----");
        } else {
            snprintf(l5_min[i], sizeof l5_min[i], "%u", (unsigned)mm);
        }
    }

    if (vl53l5_node.valid_frames == 0u) {
        dbg_printf("L5 %s src=%s frames=0 online=0x%02X stop=0x%02X fan=%u%% min=[%s %s %s]\r\n",
                   l5_state,
                   sensor_source_name(vl53l5_node.source),
                   vl53l5_node.online_mask,
                   vl53l5_node.stop_flags,
                   vl53l5_node.fan_duty_percent,
                   l5_min[0], l5_min[1], l5_min[2]);
    } else {
        dbg_printf("L5 %s src=%s age=%lums frames=%lu online=0x%02X stop=0x%02X fan=%u%% min=[%s %s %s]\r\n",
                   l5_state,
                   sensor_source_name(vl53l5_node.source),
                   (unsigned long)(now - vl53l5_node.last_update_ms),
                   (unsigned long)vl53l5_node.valid_frames,
                   vl53l5_node.online_mask,
                   vl53l5_node.stop_flags,
                   vl53l5_node.fan_duty_percent,
                   l5_min[0], l5_min[1], l5_min[2]);
    }
}
 
/* ===================================================================
 * ROS differential-drive command interface (see README protocol).
 *
 * The Raspberry Pi high-level controller streams ASCII lines on USART2:
 *      "V <left_rad_s> <right_rad_s>\n"
 * where the two values are LEFT and RIGHT drive-wheel angular velocities
 * in rad/s (signed; negative = reverse). We convert each to a BLD-515C
 * motor speed (RPM) + direction and push it over RS-485 (USART1).
 * =================================================================== */

#define DRIVE_LEFT_ADDR      0x01u   /* BLD-515C slave id - LEFT wheel  */
#define DRIVE_RIGHT_ADDR     0x02u   /* BLD-515C slave id - RIGHT wheel */

/* Motor revolutions per WHEEL revolution (total gearbox reduction).
 * Drivetrain: F6B250-24GU-30S BLDC (3000 rpm rated) + 6GU20K 20:1 spur gearbox.
 * Combined with the 150 rpm motor minimum, this 20:1 ratio gives a usable
 * robot-speed band of ~0.047 m/s (min) .. ~0.94 m/s (max) at the 0.06 m wheel
 * radius. A /cmd_vel below ~0.047 m/s maps under the motor minimum, so the
 * wheel is held stopped (that's why 0.03 m/s did nothing -- it's ~95 motor
 * RPM, below 150). */
#define DRIVE_GEAR_RATIO     20.0f

/* Drive wheels are usually mirror-mounted, so one side must spin the opposite
 * way to move the robot "forward". Flip these (0/1) to match your mechanics. */
#define DRIVE_LEFT_INVERT    0
#define DRIVE_RIGHT_INVERT   1

#define DRIVE_MIN_RPM        150u     /* below this we just stop (mfr minimum)  */
#define DRIVE_MAX_RPM        3000u    /* safety clamp                           */
#define DRIVE_CMD_TIMEOUT_MS 500u     /* no valid V line in this window -> stop */
#define RADS_TO_RPM          9.54929659f   /* 60 / (2*pi) */

/* Per-wheel cache so steady-state cruising issues ZERO RS-485 writes -- at
 * 9600 baud each register write costs ~25 ms, so we only re-send speed or
 * direction when they actually change. */
typedef struct { uint16_t rpm; bool reverse; bool running; bool valid; } wheel_state_t;
static wheel_state_t drive_wheel[2] = {0};       /* [0]=left, [1]=right */
static uint32_t      drive_last_cmd_ms = 0;
static bool          drive_timed_out   = true;   /* start safe (stopped) */
static bool          motion_armed      = false;  /* explicit ARM after boot */

/* Convert one signed wheel velocity (rad/s) to a BLD-515C command and send it,
 * skipping the write when nothing changed. `side` selects the cache slot. */
static bool drive_set_wheel(int side, uint8_t addr, float wheel_rad_s, int invert)
{
    float rpm_f   = wheel_rad_s * RADS_TO_RPM * DRIVE_GEAR_RATIO;
    bool  reverse = (rpm_f < 0.0f);
    if (invert) reverse = !reverse;
    long  mag = (long)(rpm_f < 0.0f ? -rpm_f : rpm_f);

    wheel_state_t *w = &drive_wheel[side];
    bool ok = true;

    if (mag < (long)DRIVE_MIN_RPM) {          /* too slow to run -> stop */
        if (!w->valid || w->running) ok = bld515c_stop(addr);
        w->running = false; w->rpm = 0; w->valid = true;
        return ok;
    }
    if (mag > (long)DRIVE_MAX_RPM) mag = (long)DRIVE_MAX_RPM;
    uint16_t rpm = (uint16_t)mag;

    if (!w->valid || w->rpm != rpm)
        ok = bld515c_set_speed_rpm(addr, rpm) && ok;
    if (!w->valid || !w->running || w->reverse != reverse)
        ok = bld515c_run(addr, reverse) && ok;

    w->rpm = rpm; w->reverse = reverse; w->running = true; w->valid = true;
    return ok;
}

/* Force both wheels stopped and reset the cache (timeout / startup). */
static void drive_stop_all(void)
{
    bld515c_stop(DRIVE_LEFT_ADDR);
    bld515c_stop(DRIVE_RIGHT_ADDR);
    drive_wheel[0] = (wheel_state_t){ 0, false, false, true };
    drive_wheel[1] = (wheel_state_t){ 0, false, false, true };
}

/* Stop every motor output without touching the LAC position commands.  ARM
 * uses this path so enabling stepper/drive commands can never reposition a
 * linear actuator. */
static void motion_stop_motors_preserve_lac(void)
{
    motor_stop_all();
    drive_stop_all();
    drive_timed_out = true;
}

static void motion_disarm_all(void)
{
    motion_stop_motors_preserve_lac();
    (void)lac_actuators_set_both_percent(0u);
    motion_armed = false;
}

/* Parse one "V <left> <right>" line and drive both wheels. A line that
 * doesn't start with 'V' or lacks two numbers is rejected (motors left as-is,
 * the watchdog timeout still applies). */
static void drive_apply_v(const char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line != 'V' && *line != 'v') {
        dbg_printf(">> ignore (no leading V): \"%s\"\r\n", line);
        return;
    }

    const char *p = line + 1;
    char *end;
    float left  = strtof(p, &end);
    if (end == p) { dbg_printf(">> bad cmd: missing left value\r\n");  return; }
    p = end;
    float right = strtof(p, &end);
    if (end == p) { dbg_printf(">> bad cmd: missing right value\r\n"); return; }

    bool okL = drive_set_wheel(0, DRIVE_LEFT_ADDR,  left,  DRIVE_LEFT_INVERT);
    bool okR = drive_set_wheel(1, DRIVE_RIGHT_ADDR, right, DRIVE_RIGHT_INVERT);

    drive_last_cmd_ms = HAL_GetTick();
    drive_timed_out   = false;

    /* Show the mapped motor RPM (pre-clamp) so a "received but too slow to
     * spin" command is obvious rather than looking like no response. */
    int lrpm = (int)(left  * RADS_TO_RPM * DRIVE_GEAR_RATIO);
    int rrpm = (int)(right * RADS_TO_RPM * DRIVE_GEAR_RATIO);
    dbg_printf("V L=%.3f R=%.3f rad/s -> map L=%d R=%d rpm | L:%s[%s] R:%s[%s]\r\n",
               (double)left, (double)right, lrpm, rrpm,
               drive_wheel[0].running ? (drive_wheel[0].reverse ? "REV" : "FWD") : "STOP",
               okL ? "OK" : "ERR",
               drive_wheel[1].running ? (drive_wheel[1].reverse ? "REV" : "FWD") : "STOP",
               okR ? "OK" : "ERR");

    if ((left  != 0.0f && !drive_wheel[0].running) ||
        (right != 0.0f && !drive_wheel[1].running)) {
        dbg_printf("   note: |rpm| < %u min -> wheel HELD STOPPED. Raise DRIVE_GEAR_RATIO\r\n"
                   "   to your gearbox, or command a higher speed.\r\n",
                   (unsigned)DRIVE_MIN_RPM);
    }

    /* D13/PA5 and PD2 are reserved for the two VL53L0X XSHUT signals. */
}



/* STM32_LIVE_MOTOR_TERMINAL_PATCH_V2
 * Manual stepper motor terminal on USART2.
 * Commands used by the Mac live tester:
 *   M <motor> <hz> [F|R]        continuous run, one motor at a time
 *   J <motor> <steps> [F|R] <pulse_us> exact step pulses
 *   X <steps> [F|R] <pulse_us>  diagnostic: pulse all PUL pins together
 *   Y <steps> [F|R] <pulse_us>  diagnostic: active-low all-PUL pulse
 *   P <0|1> / N                 diagnostic: force/release all PUL pins
 *   S / A                       stop all steppers
 *   LOG 0|1                     quiet / verbose terminal logs
 * Existing drive-wheel command remains: V <left_rad_s> <right_rad_s>
 */
static char cli_upper(char c)
{
    if (c >= 'a' && c <= 'z') return (char)(c - ('a' - 'A'));
    return c;
}

static const char *cli_skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static const char *cli_skip_alpha(const char *p)
{
    while ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) p++;
    return p;
}

// Advance past the command keyword so the dispatcher can hand remaining args
// to the handler. Lets "M 1 F 500" and "MOTOR 1 F 500" both resolve the same.
static const char *cli_skip_cmd_word(const char *p)
{
    p++;
    if ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) p = cli_skip_alpha(p);
    return p;
}

static bool cli_parse_long(const char **pp, long *out)
{
    const char *p = cli_skip_ws(*pp);
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    *out = v;
    *pp = end;
    return true;
}

static bool cli_word_is(const char *p, const char *word)
{
    p = cli_skip_ws(p);
    while (*word != '\0') {
        if (cli_upper(*p) != cli_upper(*word)) return false;
        p++;
        word++;
    }
    return (*p == '\0' || *p == ' ' || *p == '\t');
}

static bool cli_parse_dir_token(const char **pp, motor_dir_t *dir)
{
    const char *p = cli_skip_ws(*pp);
    if (*p == '\0') {
        *pp = p;
        return true;
    }

    char c = cli_upper(*p);
    if (c == 'F' || c == '+') {
        *dir = MOTOR_DIR_FWD;
    } else if (c == 'R' || c == '-') {
        *dir = ((motor_dir_t)1);
    } else {
        return false;
    }

    while (*p != '\0' && *p != ' ' && *p != '\t') p++;
    *pp = p;
    return true;
}

static void lift_cancel_timeout(void)
{
    lift_timed_active = false;
}

static void lift_stop_outputs(void)
{
    motor_stop(LIFT_M2_INDEX);
    motor_stop(LIFT_M5_INDEX);
    lift_timed_active = false;
}

static void cli_print_help(void)
{
    dbg_printf("\r\nManual motor commands on USART2 @ 115200:\r\n");
    dbg_printf("  ARM / DISARM                enable motion / force every output safe\r\n");
    dbg_printf("  M <1-%d> <hz> [F|R]       continuous run one stepper only\r\n", MOTOR_COUNT);
    dbg_printf("  G <list|ALL> <hz> [F|R]  continuous run motor group, e.g. G 1,5 100 F\r\n");
    dbg_printf("  D <motor> <hz> <F|R> [...]  run 2-7 motors with independent directions\r\n");
    dbg_printf("  C <slot_hz> <motor> <steps> <F|R> [...]  exact coordinated run\r\n");
    dbg_printf("  LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>  timed independent Y-axis run\r\n");
    dbg_printf("  J <1-%d> <steps> [F|R] <pulse_us>  exact step pulses\r\n", MOTOR_COUNT);
    dbg_printf("  X <steps> [F|R] <pulse_us>  diagnostic: pulse all PUL pins together\r\n");
    dbg_printf("  Y <steps> [F|R] <pulse_us>  diagnostic: active-low all-PUL pulse\r\n");
    dbg_printf("  P 0 / P 1                 diagnostic: hold every PUL pin low/high\r\n");
    dbg_printf("  N                         restore PUL pins to normal timer mode\r\n");
    dbg_printf("  S or A                    stop all steppers\r\n");
    dbg_printf("  LOG 0 / LOG 1             quiet / verbose board messages\r\n");
    dbg_printf("  LC STATUS                  one raw four-load-cell snapshot\r\n");
    dbg_printf("  LC TARE                    zero all four currently unloaded cells\r\n");
    dbg_printf("  LC STREAM 0|1              stop/start raw load-cell stream\r\n");
    dbg_printf("  LC RESET                   reset observed min/max signal span\r\n");
    dbg_printf("  SW STATUS                  read PC6 and PC9 switch levels\r\n");
    dbg_printf("  SW STREAM 0|1              stop/start switch-level stream\r\n");
    dbg_printf("  SW PULL NONE|UP|DOWN       configure input pull resistors\r\n");
    dbg_printf("  TOF STATUS                  read physical-right D13/PA5 ToF only\r\n");
    dbg_printf("  LAC <1|2> <0-100>          set one actuator position percent\r\n");
    dbg_printf("  LAC BOTH <0-100>           set both actuator positions\r\n");
    dbg_printf("  LAC HOME / LAC STATUS      retract both / show commanded positions\r\n");
    dbg_printf("Example: M 1 500 F, G ALL 100 F, then S.  Example step: J 1 1 F 500\r\n\r\n");
}

static void cli_lac(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        dbg_printf("LAC commanded: 1=%u%% LEFT on A0/PA0, "
                   "2=%u%% RIGHT on A1/PA1; signal="
#if LAC_RC_TEST_MODE
                   "temporary RC 50Hz (1-2ms); "
#else
                   "3.3V 1kHz PWM (logical 0-100%% maps to 1-99%% duty); "
#endif
                   "command only, no feedback\r\n",
                   (unsigned)lac_actuator_get_percent(1u),
                   (unsigned)lac_actuator_get_percent(2u));
        return;
    }

    if (cli_word_is(args, "HOME")) {
        if (!lac_actuators_set_both_percent(0u)) {
            dbg_printf("ERR: LAC outputs are not initialized\r\n");
            return;
        }
        dbg_printf("OK: both LAC actuators commanded HOME (0%%)\r\n");
        return;
    }

    if (cli_word_is(args, "BOTH")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long percent = 0;
        if (!cli_parse_long(&args, &percent) || percent < 0 || percent > 100) {
            dbg_printf("ERR: use LAC BOTH <0-100>\r\n");
            return;
        }
        if (!lac_actuators_set_both_percent((uint8_t)percent)) {
            dbg_printf("ERR: LAC outputs are not initialized\r\n");
            return;
        }
        dbg_printf("OK: both LAC actuators commanded %ld%%\r\n", percent);
        return;
    }

    long actuator = 0;
    long percent = 0;
    if (!cli_parse_long(&args, &actuator) ||
        !cli_parse_long(&args, &percent) ||
        actuator < 1 || actuator > (long)LAC_ACTUATOR_COUNT ||
        percent < 0 || percent > 100) {
        dbg_printf("ERR: use LAC <1|2> <0-100>, LAC BOTH <0-100>, "
                   "LAC HOME, or LAC STATUS\r\n");
        return;
    }

    if (!lac_actuator_set_percent((uint8_t)actuator, (uint8_t)percent)) {
        dbg_printf("ERR: LAC outputs are not initialized\r\n");
        return;
    }
    dbg_printf("OK: LAC %ld commanded %ld%%\r\n", actuator, percent);
}

static void cli_run_stepper(const char *args)
{
    long motor_num = 0;
    long hz_signed = 0;

    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: missing motor number. Example: M 1 500 F\r\n");
        return;
    }
    if (motor_num < 1 || motor_num > MOTOR_COUNT) {
        dbg_printf("ERR: motor must be 1..%d\r\n", MOTOR_COUNT);
        return;
    }
    if (!cli_parse_long(&args, &hz_signed)) {
        dbg_printf("ERR: missing speed in Hz. Example: M %ld 500 F\r\n", motor_num);
        return;
    }

    motor_dir_t dir = MOTOR_DIR_FWD;
    uint32_t hz;
    if (hz_signed < 0) {
        dir = ((motor_dir_t)1);
        hz = (uint32_t)(-hz_signed);
    } else {
        hz = (uint32_t)hz_signed;
    }

    const char *dir_args = args;
    if (*cli_skip_ws(dir_args) != '\0' && !cli_parse_dir_token(&dir_args, &dir)) {
        dbg_printf("ERR: direction must be F or R. Example: M %ld %lu R\r\n",
                   motor_num, (unsigned long)hz);
        return;
    }

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();

    if (hz == 0u) {
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    int idx = (int)motor_num - 1;
    motor_set_dir(idx, dir);
    motor_set_speed_hz(idx, hz);
    dbg_printf("OK: M%ld continuous %lu Hz DIR=%s\r\n",
               motor_num, (unsigned long)hz,
               (dir == MOTOR_DIR_FWD) ? "F" : "R");
}

static bool cli_parse_motor_mask(const char **pp, uint8_t *mask)
{
    const char *p = cli_skip_ws(*pp);
    uint8_t parsed = 0u;

    if (*p == '*') {
        *mask = (uint8_t)((1u << MOTOR_COUNT) - 1u);
        *pp = p + 1;
        return true;
    }

    if ((p[0] == 'A' || p[0] == 'a') &&
        (p[1] == 'L' || p[1] == 'l') &&
        (p[2] == 'L' || p[2] == 'l') &&
        (p[3] == '\0' || p[3] == ' ' || p[3] == '\t')) {
        *mask = (uint8_t)((1u << MOTOR_COUNT) - 1u);
        *pp = p + 3;
        return true;
    }

    while (true) {
        long motor_num = 0;

        p = cli_skip_ws(p);
        if (*p < '0' || *p > '9') {
            return false;
        }

        while (*p >= '0' && *p <= '9') {
            motor_num = motor_num * 10 + (*p - '0');
            p++;
        }

        if (motor_num < 1 || motor_num > MOTOR_COUNT) {
            return false;
        }
        parsed |= (uint8_t)(1u << (motor_num - 1));

        const char *q = cli_skip_ws(p);
        if (*q == ',' || *q == '+' || *q == '/') {
            p = q + 1;
            continue;
        }

        *mask = parsed;
        *pp = q;
        return parsed != 0u;
    }
}

static void cli_run_stepper_group(const char *args)
{
    uint8_t mask = 0u;
    long hz_signed = 0;

    if (!cli_parse_motor_mask(&args, &mask)) {
        dbg_printf("ERR: missing motor list. Example: G 1,5 100 F or G ALL 100 F\r\n");
        return;
    }
    if (!cli_parse_long(&args, &hz_signed)) {
        dbg_printf("ERR: missing speed in Hz. Example: G 1,5 100 F\r\n");
        return;
    }

    motor_dir_t dir = MOTOR_DIR_FWD;
    uint32_t hz;
    if (hz_signed < 0) {
        dir = MOTOR_DIR_REV;
        hz = (uint32_t)(-hz_signed);
    } else {
        hz = (uint32_t)hz_signed;
    }

    const char *dir_args = args;
    if (*cli_skip_ws(dir_args) != '\0' && !cli_parse_dir_token(&dir_args, &dir)) {
        dbg_printf("ERR: direction must be F or R. Example: G 1,5 %lu R\r\n",
                   (unsigned long)hz);
        return;
    }

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();

    if (hz == 0u) {
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) == 0u) continue;
        motor_set_dir(idx, dir);
        motor_set_speed_hz(idx, hz);
    }

    dbg_printf("OK: group mask 0x%02X continuous %lu Hz DIR=%s\r\n",
               mask, (unsigned long)hz,
               (dir == MOTOR_DIR_FWD) ? "F" : "R");
}

static void cli_run_stepper_pair(const char *args)
{
    uint32_t speeds[MOTOR_COUNT] = {0u};
    motor_dir_t directions[MOTOR_COUNT];
    uint8_t mask = 0u;
    int count = 0;

    while (*cli_skip_ws(args) != '\0') {
        long motor_num = 0;
        long hz = 0;
        motor_dir_t dir = MOTOR_DIR_FWD;

        if (!cli_parse_long(&args, &motor_num) ||
            !cli_parse_long(&args, &hz) ||
            *cli_skip_ws(args) == '\0' ||
            !cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: use D <motor> <hz> <F|R> repeated 2-%d times\r\n",
                       MOTOR_COUNT);
            return;
        }

        if (motor_num < 1 || motor_num > MOTOR_COUNT || hz < 0) {
            dbg_printf("ERR: motors must be 1-%d and speeds must be non-negative\r\n",
                       MOTOR_COUNT);
            return;
        }

        int idx = (int)motor_num - 1;
        uint8_t bit = (uint8_t)(1u << idx);
        if ((mask & bit) != 0u) {
            dbg_printf("ERR: M%ld was listed more than once\r\n", motor_num);
            return;
        }

        mask |= bit;
        speeds[idx] = (uint32_t)hz;
        directions[idx] = dir;
        count++;
    }

    if (count < 2) {
        dbg_printf("ERR: D requires at least two different motors\r\n");
        return;
    }

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();

    /* Set every direction before emitting the first pulse, then start all
     * selected channels back-to-back. Motors sharing TIM3 must use the same
     * frequency; motor_set_speed_hz() keeps its active channels synchronized. */
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            motor_set_dir(idx, directions[idx]);
        }
    }
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            motor_set_speed_hz(idx, speeds[idx]);
        }
    }

    dbg_printf("OK: D started %d motors, mask=0x%02X\r\n", count, mask);
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            dbg_printf("  M%d %lu Hz DIR=%s\r\n", idx + 1,
                       (unsigned long)speeds[idx],
                       (directions[idx] == MOTOR_DIR_FWD) ? "F" : "R");
        }
    }
}

static void cli_run_coordinated(const char *args)
{
    long slot_hz_signed = 0;
    uint32_t steps[MOTOR_COUNT] = {0u};
    motor_dir_t directions[MOTOR_COUNT];
    uint8_t mask = 0u;
    int count = 0;

    if (!cli_parse_long(&args, &slot_hz_signed) ||
        slot_hz_signed < 1 || slot_hz_signed > 4000) {
        dbg_printf("ERR: coordinated slot rate must be 1..4000 Hz\r\n");
        return;
    }

    while (*cli_skip_ws(args) != '\0') {
        long motor_num = 0;
        long motor_steps = 0;
        motor_dir_t dir = MOTOR_DIR_FWD;
        if (!cli_parse_long(&args, &motor_num) ||
            !cli_parse_long(&args, &motor_steps) ||
            *cli_skip_ws(args) == '\0' ||
            !cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: use C <slot_hz> <motor> <steps> <F|R> repeated\r\n");
            return;
        }
        if (motor_num < 1 || motor_num > MOTOR_COUNT ||
            motor_steps < 1 || motor_steps > 100000) {
            dbg_printf("ERR: motors must be 1-%d and steps must be 1..100000\r\n",
                       MOTOR_COUNT);
            return;
        }
        int idx = (int)motor_num - 1;
        uint8_t bit = (uint8_t)(1u << idx);
        if ((mask & bit) != 0u) {
            dbg_printf("ERR: M%ld was listed more than once\r\n", motor_num);
            return;
        }
        mask |= bit;
        steps[idx] = (uint32_t)motor_steps;
        directions[idx] = dir;
        count++;
    }

    if (count < 2) {
        dbg_printf("ERR: C requires at least two different motors\r\n");
        return;
    }

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();
    if (!motor_coordinated_start(steps, directions, (uint32_t)slot_hz_signed)) {
        dbg_printf("ERR: coordinated motion could not start\r\n");
        return;
    }

    uint32_t max_steps = 0u;
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if (steps[idx] > max_steps) max_steps = steps[idx];
    }
    dbg_printf("OK: C started %d motors, slot=%ld Hz, max=%lu steps, about %lums\r\n",
               count, slot_hz_signed, (unsigned long)max_steps,
               (unsigned long)((max_steps * 1000u) /
                               (uint32_t)slot_hz_signed));
}

static void cli_run_lift(const char *args)
{
    args = cli_skip_ws(args);

    motor_dir_t dir;
    const char *direction_name;
    if (cli_word_is(args, "UP")) {
        dir = MOTOR_DIR_REV;
        direction_name = "UP";
    } else if (cli_word_is(args, "DOWN")) {
        dir = MOTOR_DIR_FWD;
        direction_name = "DOWN";
    } else {
        dbg_printf("ERR: use LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>\r\n");
        return;
    }
    args = cli_skip_alpha(args);

    long m2_hz = 0;
    long m5_hz = 0;
    long duration_ms = 0;
    if (!cli_parse_long(&args, &m2_hz) ||
        !cli_parse_long(&args, &m5_hz) ||
        !cli_parse_long(&args, &duration_ms)) {
        dbg_printf("ERR: use LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>\r\n");
        return;
    }

    bool m2_valid = (m2_hz == 0) ||
                    (m2_hz >= (long)LIFT_MIN_HZ && m2_hz <= (long)LIFT_MAX_HZ);
    bool m5_valid = (m5_hz == 0) ||
                    (m5_hz >= (long)LIFT_MIN_HZ && m5_hz <= (long)LIFT_MAX_HZ);
    if (!m2_valid || !m5_valid || (m2_hz == 0 && m5_hz == 0)) {
        dbg_printf("ERR: each lift speed must be 0 or %u..%u Hz; at least one must run\r\n",
                   (unsigned)LIFT_MIN_HZ, (unsigned)LIFT_MAX_HZ);
        return;
    }
    if (duration_ms < (long)LIFT_MIN_DURATION_MS ||
        duration_ms > (long)LIFT_MAX_DURATION_MS) {
        dbg_printf("ERR: lift duration must be %u..%u ms\r\n",
                   (unsigned)LIFT_MIN_DURATION_MS,
                   (unsigned)LIFT_MAX_DURATION_MS);
        return;
    }

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();
    motor_set_dir(LIFT_M2_INDEX, dir);
    motor_set_dir(LIFT_M5_INDEX, dir);
    motor_set_speed_hz(LIFT_M2_INDEX, (uint32_t)m2_hz);
    motor_set_speed_hz(LIFT_M5_INDEX, (uint32_t)m5_hz);

    lift_started_ms = HAL_GetTick();
    lift_duration_ms = (uint32_t)duration_ms;
    lift_timed_active = true;

    dbg_printf("OK: LIFT %s M2=%ldHz M5=%ldHz duration=%ldms\r\n",
               direction_name, m2_hz, m5_hz, duration_ms);
}

static void cli_jog_stepper(const char *args)
{
    long motor_num = 0;
    long steps = 1;
    long pulse_us = 500;
    motor_dir_t dir = MOTOR_DIR_FWD;

    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: missing motor number. Example: J 1 1 F 500\r\n");
        return;
    }
    if (motor_num < 1 || motor_num > MOTOR_COUNT) {
        dbg_printf("ERR: motor must be 1..%d\r\n", MOTOR_COUNT);
        return;
    }

    const char *after_motor = args;
    if (cli_parse_long(&args, &steps)) {
        /* steps supplied */
    } else {
        args = after_motor;
        steps = 1;
    }

    const char *p = cli_skip_ws(args);
    if (*p != '\0' && (cli_upper(*p) == 'F' || cli_upper(*p) == 'R' || *p == '+' || *p == '-')) {
        if (!cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: direction must be F or R. Example: J %ld %ld R 500\r\n", motor_num, steps);
            return;
        }
    }

    const char *after_dir = args;
    if (!cli_parse_long(&args, &pulse_us)) {
        args = after_dir;
        pulse_us = 500;
    }

    if (steps < 1) steps = 1;
    if (steps > 10000) steps = 10000;
    if (pulse_us < 2) pulse_us = 2;
    if (pulse_us > 100000) pulse_us = 100000;

    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();
    motor_step_pulses((int)motor_num - 1, dir, (uint32_t)steps, (uint32_t)pulse_us);
    motor_stop_all();

    dbg_printf("OK: M%ld step/jog %ld step(s), DIR=%s, pulse=%ld us\r\n",
               motor_num, steps,
               (dir == MOTOR_DIR_FWD) ? "F" : "R",
               pulse_us);
}

static void cli_stop_stepper(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "ALL") || *args == '*') {
        lift_cancel_timeout();
        motor_release_all_pul_timer();
        motor_stop_all();
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    long motor_num = 0;
    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: use S, S ALL, or S <1-%d>\r\n", MOTOR_COUNT);
        return;
    }
    if (motor_num < 1 || motor_num > MOTOR_COUNT) {
        dbg_printf("ERR: motor must be 1..%d\r\n", MOTOR_COUNT);
        return;
    }

    if (motor_num == (LIFT_M2_INDEX + 1) || motor_num == (LIFT_M5_INDEX + 1)) {
        lift_cancel_timeout();
    }
    motor_stop((int)motor_num - 1);
    dbg_printf("OK: M%ld stopped\r\n", motor_num);
}

static void cli_force_pul_pins(const char *args)
{
    long high = 0;
    if (!cli_parse_long(&args, &high)) {
        dbg_printf("ERR: use P 0 or P 1\r\n");
        return;
    }

    lift_cancel_timeout();
    motor_force_all_pul_gpio(high != 0);
    dbg_printf("OK: all PUL pins forced %s. Use N or S to restore normal mode.\r\n",
               high ? "HIGH" : "LOW");
}

static void cli_pulse_all_steppers(const char *args, bool active_low)
{
    long steps = 1000;
    long pulse_us = 500;
    motor_dir_t dir = MOTOR_DIR_FWD;

    const char *after_start = args;
    if (!cli_parse_long(&args, &steps)) {
        args = after_start;
        steps = 1000;
    }

    const char *p = cli_skip_ws(args);
    if (*p != '\0' && (cli_upper(*p) == 'F' || cli_upper(*p) == 'R' || *p == '+' || *p == '-')) {
        if (!cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: direction must be F or R. Example: X 1000 F 500\r\n");
            return;
        }
    }

    const char *after_dir = args;
    if (!cli_parse_long(&args, &pulse_us)) {
        args = after_dir;
        pulse_us = 500;
    }

    if (steps < 1) steps = 1;
    if (steps > 10000) steps = 10000;
    if (pulse_us < 2) pulse_us = 2;
    if (pulse_us > 100000) pulse_us = 100000;

    lift_cancel_timeout();
    motor_stop_all();
    if (active_low) {
        motor_step_all_pulses_active_low(dir, (uint32_t)steps, (uint32_t)pulse_us);
    } else {
        motor_step_all_pulses(dir, (uint32_t)steps, (uint32_t)pulse_us);
    }
    motor_stop_all();

    dbg_printf("OK: all PUL pins pulsed %ld step(s), DIR=%s, pulse=%ld us, mode=%s\r\n",
               steps,
               (dir == MOTOR_DIR_FWD) ? "F" : "R",
               pulse_us,
               active_low ? "active-low" : "active-high");
}

static void cli_set_log(const char *args)
{
    long enable = 0;
    if (!cli_parse_long(&args, &enable)) {
        dbg_printf("LOG is %s. Use LOG 0 or LOG 1.\r\n",
                   terminal_log_enabled ? "ON" : "OFF");
        return;
    }
    terminal_log_enabled = (enable != 0);
    dbg_printf("OK: periodic terminal logs %s\r\n",
               terminal_log_enabled ? "ON" : "OFF");
}

static void load_cell_print_status(void)
{
    hx711_parallel_state_t state;
    hx711_parallel_get_state(&state);

    if (state.sample_count == 0u) {
        dbg_printf("LC WAIT samples=0 valid=0x%X ready_now=0x%X "
                   "last_timeout_ready=0x%X timeouts=%lu\r\n",
                   state.valid_mask,
                   state.ready_mask,
                   state.last_timeout_ready_mask,
                   (unsigned long)state.timeout_count);
        dbg_printf("LC wiring bits: ch0=PB0 ch1=PB5 ch2=PC7 ch3=PA8; shared SCK=PC0/A5\r\n");
        return;
    }

    uint32_t now = HAL_GetTick();
    int32_t delta[HX711_PARALLEL_CHANNEL_COUNT];
    int32_t span[HX711_PARALLEL_CHANNEL_COUNT];
    for (uint8_t i = 0u; i < HX711_PARALLEL_CHANNEL_COUNT; ++i) {
        delta[i] = state.raw[i] - state.zero[i];
        span[i] = state.max_delta[i] - state.min_delta[i];
    }

    dbg_printf("LC %s samples=%lu age=%lums valid=0x%X ready_now=0x%X "
               "timeouts=%lu last_timeout_ready=0x%X\r\n",
               (state.valid_mask == HX711_PARALLEL_ALL_MASK) ? "LIVE" : "STALE",
               (unsigned long)state.sample_count,
               (unsigned long)(now - state.last_update_ms),
               state.valid_mask,
               state.ready_mask,
               (unsigned long)state.timeout_count,
               state.last_timeout_ready_mask);
    dbg_printf("LC raw=[%ld %ld %ld %ld] delta=[%ld %ld %ld %ld]\r\n",
               (long)state.raw[0], (long)state.raw[1],
               (long)state.raw[2], (long)state.raw[3],
               (long)delta[0], (long)delta[1],
               (long)delta[2], (long)delta[3]);
    dbg_printf("LC span=[%ld %ld %ld %ld] (max-min counts since tare/reset)\r\n",
               (long)span[0], (long)span[1],
               (long)span[2], (long)span[3]);
}

static void cli_load_cell(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        load_cell_print_status();
        return;
    }

    if (cli_word_is(args, "TARE")) {
        if (hx711_parallel_tare()) {
            dbg_printf("OK: all four load-cell channels tared; no load should be applied\r\n");
            load_cell_print_status();
        } else {
            dbg_printf("ERR: cannot tare until all four HX711 channels are LIVE\r\n");
            load_cell_print_status();
        }
        return;
    }

    if (cli_word_is(args, "RESET")) {
        hx711_parallel_reset_span();
        dbg_printf("OK: load-cell observed spans reset\r\n");
        return;
    }

    if (cli_word_is(args, "STREAM")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long enable = 0;
        if (!cli_parse_long(&args, &enable)) {
            dbg_printf("LC stream is %s. Use LC STREAM 0 or LC STREAM 1.\r\n",
                       load_cell_stream_enabled ? "ON" : "OFF");
            return;
        }
        load_cell_stream_enabled = (enable != 0);
        load_cell_last_print_ms = 0u;
        dbg_printf("OK: load-cell stream %s\r\n",
                   load_cell_stream_enabled ? "ON" : "OFF");
        return;
    }

    dbg_printf("ERR: use LC STATUS, LC TARE, LC STREAM 0|1, or LC RESET\r\n");
}

static const char *switch_pull_name(void)
{
    if (switch_pull_mode == GPIO_PULLUP) return "UP";
    if (switch_pull_mode == GPIO_PULLDOWN) return "DOWN";
    return "NONE";
}

static void switch_configure_inputs(uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SWITCH_1_GPIO_PIN | SWITCH_2_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    switch_pull_mode = pull;
}

static void switch_print_status(void)
{
    unsigned pc6 = (HAL_GPIO_ReadPin(SWITCH_1_GPIO_PORT, SWITCH_1_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    unsigned pc9 = (HAL_GPIO_ReadPin(SWITCH_2_GPIO_PORT, SWITCH_2_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    dbg_printf("SW PC6=%u(%s) PC9=%u(%s) pull=%s\r\n",
               pc6, pc6 ? "HIGH" : "LOW",
               pc9, pc9 ? "HIGH" : "LOW",
               switch_pull_name());
}

static void cli_switch(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        switch_print_status();
        return;
    }

    if (cli_word_is(args, "STREAM")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long enable = 0;
        if (!cli_parse_long(&args, &enable)) {
            dbg_printf("SW stream is %s. Use SW STREAM 0 or SW STREAM 1.\r\n",
                       switch_stream_enabled ? "ON" : "OFF");
            return;
        }
        switch_stream_enabled = (enable != 0);
        switch_last_print_ms = 0u;
        dbg_printf("OK: switch stream %s\r\n",
                   switch_stream_enabled ? "ON" : "OFF");
        return;
    }

    if (cli_word_is(args, "PULL")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        args = cli_skip_ws(args);
        if (cli_word_is(args, "UP")) {
            switch_configure_inputs(GPIO_PULLUP);
        } else if (cli_word_is(args, "DOWN")) {
            switch_configure_inputs(GPIO_PULLDOWN);
        } else if (cli_word_is(args, "NONE")) {
            switch_configure_inputs(GPIO_NOPULL);
        } else {
            dbg_printf("ERR: use SW PULL NONE, SW PULL UP, or SW PULL DOWN\r\n");
            return;
        }
        dbg_printf("OK: PC6/PC9 input pull=%s\r\n", switch_pull_name());
        switch_print_status();
        return;
    }

    dbg_printf("ERR: use SW STATUS, SW STREAM 0|1, or SW PULL NONE|UP|DOWN\r\n");
}

static void cli_apply_line(const char *line)
{
    const char *p = cli_skip_ws(line);
    if (*p == '\0') return;

    if (cli_word_is(p, "DISARM")) {
        motion_disarm_all();
        dbg_printf("OK: DISARMED; all motion outputs stopped\r\n");
        return;
    }

    if (cli_word_is(p, "ARM")) {
        motion_stop_motors_preserve_lac();
        motion_armed = true;
        dbg_printf("OK: ARMED; motor outputs stopped, LAC targets preserved\r\n");
        return;
    }

    char cmd = cli_upper(*p);

    if (cmd == '?' || cmd == 'H') {
        cli_print_help();
        return;
    }

    if (cli_word_is(p, "TOF") || cli_word_is(p, "TOF STATUS")) {
        direct_tof_print_physical_right(HAL_GetTick());
        return;
    }

    if (!motion_armed) {
        if (cli_word_is(p, "LAC STATUS")) {
            cli_lac(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "SW") || cli_word_is(p, "LC") ||
            cli_word_is(p, "LOG")) {
            dbg_printf("LOCKED: diagnostic command available after ARM; type H for help\r\n");
            return;
        }
        disarmed_reject_count++;
        dbg_printf("LOCKED: motion is DISARMED; send ARM deliberately\r\n");
        return;
    }

    if (cmd == 'V') {
        drive_apply_v(line);
        return;
    }

    if (cmd == 'M') {
        cli_run_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'G') {
        cli_run_stepper_group(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'D') {
        cli_run_stepper_pair(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'C') {
        cli_run_coordinated(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'J' || cmd == 'T') {
        cli_jog_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'S') {
        if (cli_word_is(p, "SW")) {
            cli_switch(cli_skip_cmd_word(p));
            return;
        }
        cli_stop_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'X') {
        cli_pulse_all_steppers(cli_skip_cmd_word(p), false);
        return;
    }

    if (cmd == 'Y') {
        cli_pulse_all_steppers(cli_skip_cmd_word(p), true);
        return;
    }

    if (cmd == 'P') {
        cli_force_pul_pins(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'N') {
        lift_cancel_timeout();
        motor_release_all_pul_timer();
        dbg_printf("OK: PUL pins restored to normal timer mode\r\n");
        return;
    }

    if (cmd == 'A') {
        lift_cancel_timeout();
        motor_release_all_pul_timer();
        motor_stop_all();
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    if (cmd == 'L') {
        if (cli_word_is(p, "LIFT")) {
            cli_run_lift(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "LAC")) {
            cli_lac(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "LC")) {
            cli_load_cell(cli_skip_cmd_word(p));
            return;
        }
        cli_set_log(cli_skip_cmd_word(p));
        return;
    }

    dbg_printf("ERR: unknown command '%c'. Type H for help.\r\n", cmd);
}


/* ---- BLD-515C BLDC bring-up test on USART1 / RS-485 -----------------------
 * Runs once at boot. Watch USART2 (ST-Link VCP @ 115200) for the trace.
 *
 *   1. Round-trip read of the control word (0x8106) -> proves the bus,
 *      baud, address and wiring are all correct before we spin anything.
 *   2. Set a conservative target speed, then run forward / stop / reverse.
 *
 * If step 1 reports "no reply": confirm the driver's baud is also 9600
 * (huart1 is set to 9600), the slave address isn't 1, or A/B are swapped.
 * If your converter needs manual DE/RE, enable the
 * BLD515C_DE_RE_PORT/PIN #defines in bld515c.h (PB6 is free).            */
static void bldc_bringup_probe(void)
{
    const uint8_t bldc_addr = BLD515C_DEFAULT_ADDR;

    bld515c_init(&huart1);
    drive_stop_all();
    dbg_printf(">> boot interlock: STOP sent to both drive addresses before probe\r\n");
    dbg_printf("\r\n>> BLD-515C bring-up on USART1 @ 9600 (slave 0x%02X)...\r\n", bldc_addr);

    /* Raw probe of the read-only device-type reg (0x8000) -- dumps every byte
     * that comes back for human eyes. A correct reply contains the sequence
     * 01 03 02 01 92 <crc> <crc> (possibly preceded by an echo of our own
     * request on a half-duplex converter). */
    uint8_t raw[24];
    int n = bld515c_probe(bldc_addr, BLD515C_REG_DEVTYPE, raw, sizeof raw, 0);
    dbg_printf(">> probe read 0x8000: %d byte(s) back:", n);
    for (int i = 0; i < n; ++i) dbg_printf(" %02X", raw[i]);
    dbg_printf("\r\n");

    /* Authoritative check: read_regs() scans the capture for a CRC-valid
     * device-type response (0x0192), skipping any leading bus echo. */
    uint16_t devtype = 0;
    bool online = bld515c_read_regs(bldc_addr, BLD515C_REG_DEVTYPE, &devtype, 1);

    if (!online) {
        /* MCU self-check. The byte-by-byte loopback is reliable (unlike the
         * burst probe above). It only means something when PA9<->PA10 are
         * jumpered together with the converter UNPLUGGED; with the converter
         * wired up this is expected to FAIL and says nothing about the MCU. */
        bool loop = bld515c_selftest_loopback();
        dbg_printf(">> USART1 loopback self-test: %s  (valid only with PA9<->PA10\r\n"
                   "   jumpered & converter unplugged)\r\n",
                   loop ? "PASS -> USART1 TX+RX are healthy; you do NOT need another UART"
                        : "FAIL");

        if (n == 0) {
            dbg_printf(">> SILENT BUS: nothing came back. If NOT in loopback, check\r\n"
                       "   converter power, A/B polarity, GND, and TTL TX/RX orientation\r\n"
                       "   (PA9->conv RXD, PA10->conv TXD).\r\n");
        } else if (raw[0] != bldc_addr) {
            dbg_printf(">> Got bytes but the first isn't our address (0x%02X) -> A/B\r\n"
                       "   likely swapped, baud mismatch (driver not at 9600), or wrong\r\n"
                       "   slave addr.\r\n", bldc_addr);
        } else if (n >= 2 && (raw[1] & 0x80u)) {
            dbg_printf(">> Driver replied with a Modbus EXCEPTION (code 0x%02X) -- it's\r\n"
                       "   alive and addressed correctly; the register/function was\r\n"
                       "   rejected. Wiring & baud are GOOD.\r\n", n >= 3 ? raw[2] : 0);
        } else {
            dbg_printf(">> Got %d byte(s) but no CRC-valid frame -> partial/garbled reply.\r\n"
                       "   Suspect baud mismatch (driver not exactly 9600) or noise on A/B.\r\n", n);
        }
        dbg_printf(">> Skipping spin-up until the link is verified.\r\n");
    } else {
        dbg_printf(">> BLD-515C ONLINE. device type 0x8000 = 0x%04X\r\n", devtype);

        /* Boot-time motor movement test intentionally disabled. Keep the
         * bus probe above, but never run canned wheel commands at startup. */
        drive_stop_all();
        dbg_printf(">> boot motor movement test disabled; wheels commanded stopped.\r\n");
        dbg_printf(">> now listening for live V commands.\r\n");
    }
}


/* ---- Direct VL53L0X pair bring-up on I2C2 ---------------------------------
 * Boots the two VL53L0X ToF sensors directly wired to the STM32 (left on
 * PA5/XSHUT, right on PD2/XSHUT, shared PB3/PB10 bus). Reports the init stage
 * and the recovery status so a stuck SDA/SCL line is visible in the log, and
 * dumps the I2C address scan to confirm which devices answered.             */
static void direct_tof_bringup(void)
{
#if Y_LIFT_TEST_MODE
    dbg_printf("\r\n>> Direct ToF initialization skipped during bounded Y lift test\r\n");
#else
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
#endif
}


/* ---- USART2 command terminal bring-up -------------------------------------
 * Clears line-assembly state, flushes any stale byte sitting in the USART2
 * data register, enables the NVIC line and arms single-byte interrupt RX so
 * HAL_UART_RxCpltCallback() can accumulate characters into cmd_buf. Prints
 * the boot banner so a human on the VCP sees the terminal come up.          */
static void cli_terminal_init(void)
{
#if DIRECT_TOF_TEST_MODE
    dbg_printf("\r\n==== DIRECT ToF SENSOR FEED (read-only) ====\r\n");
    dbg_printf(">> Motion commands are disabled in this diagnostic build.\r\n");
#else
    cmd_idx = 0u;
    cmd_ready = false;
    usart2_rx_count = 0u;
    usart2_line_count = 0u;
    disarmed_reject_count = 0u;
    volatile uint32_t uart2_flush = huart2.Instance->SR;
    uart2_flush = huart2.Instance->DR;
    (void)uart2_flush;
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
    dbg_printf("\r\n==== STM32 motor terminal ready ====\r\n");
#if Y_LIFT_TEST_MODE
    dbg_printf(">> Bounded mode: only LIFT UP/DOWN and S/A stop commands accepted.\r\n");
#else
    dbg_printf(">> Manual stepper terminal on USART2 @ 115200. Type H for help.\r\n");
    dbg_printf(">> Mac tester uses M for continuous, J for exact step pulses, S to stop.\r\n");
    dbg_printf(">> Existing ROS drive command still works: V <left_rad_s> <right_rad_s>\\n\r\n");
    dbg_printf(">>   e.g. 'V 1.0 1.0' (fwd), 'V -1.908 1.908' (turn), 'V 0 0' (stop)\r\n");
    dbg_printf(">>   no valid V line for %u ms -> both wheels auto-stop.\r\n",
               (unsigned)DRIVE_CMD_TIMEOUT_MS);
    dbg_printf(">> BOOT STATE: DISARMED. Send ARM before any motion command.\r\n");
#endif
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
 
  usart3_rx_dma_read_pos = 0u;
  if (HAL_UART_Receive_DMA(&huart3, usart3_rx_dma_buf, USART3_RX_DMA_BUF_SIZE) != HAL_OK) {
      Error_Handler();
  }
#if DIRECT_TOF_TEST_MODE
  dbg_printf("\r\n>> DIRECT TOF TEST MODE: stepper, BLDC and LAC movement disabled\r\n");
#elif Y_LIFT_TEST_MODE
  motor_init();
  motor_stop_all();
  dbg_printf("\r\n>> Y LIFT TEST MODE: only timed M2/M5 lift commands enabled\r\n");
  dbg_printf(">> M1/M3/M4/M6/M7, BLDC and LAC movement disabled\r\n");
#else
  motor_init();
  motor_stop_all();
  if (!lac_actuators_init(&htim2)) {
      Error_Handler();
  }
#endif
  hx711_parallel_init();
  switch_configure_inputs(GPIO_PULLUP);
  dbg_printf(">> HX711 load cells: SCK=PC0/A5, DOUT=[PB0 PB5 PC7 PA8], waiting for all four\r\n");
  dbg_printf(">> limit switches: PC6 and PC9 active-low inputs, pull=UP; use SW STATUS\r\n");

  direct_tof_bringup();

  /* BNO055 IMU on I2C2 (PB3/PB10), RST on PC3. Blocks ~750 ms for the chip
   * to boot. Status is printed over USART2 (TX still works even though RX
   * is currently disabled by the physical-layer test mode above). */
#if DIRECT_TOF_TEST_MODE || Y_LIFT_TEST_MODE
  dbg_printf("\r\n>> BNO055 skipped so I2C2 carries only ToF diagnostic traffic\r\n");
#else
  dbg_printf("\r\n>> Bringing up BNO055 IMU...\r\n");
  imu_ok = bno055_init();
  dbg_printf(">> BNO055 %s\r\n", imu_ok ? "READY (NDOF, polled @ ~100 Hz)"
                                        : "NOT FOUND (check PB3/PB10/PC3 wiring & pullups)");
#endif

#if Y_LIFT_TEST_MODE
  dbg_printf(">> LAC actuators disabled during bounded Y lift test\r\n");
#else
  dbg_printf(">> LAC actuators: LEFT=A0/PA0, RIGHT=A1/PA1, "
#if LAC_RC_TEST_MODE
             "temporary RC 50Hz (1-2ms); startup command=HOME\r\n");
#else
             "3.3V 1kHz PWM (1-99%% duty); startup command=HOME\r\n");
#endif
#endif
  dbg_printf(">> ESP32 sensor nodes on USART3/RS485 @ 115200\r\n");
  dbg_printf(">> cmd=0x01: 4x VL53L0X + 1x ultrasonic; cmd=0x02: 3x VL53L5CX grids + fans\r\n");
  dbg_printf(">> USART2 is terminal/Raspberry Pi only; sensor bytes stay on RS485 UARTs.\r\n");

#if !DIRECT_TOF_TEST_MODE && !Y_LIFT_TEST_MODE
  bldc_bringup_probe();
#else
  dbg_printf(">> BLDC probe skipped in direct ToF test mode\r\n");
#endif

  cli_terminal_init();


  uint32_t sensor_last_print_ms    = 0u;
  uint32_t sensor_last_valid_frames = 0u;
  uint32_t sensor_last_rx_bytes     = 0u;
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    sensor_node_service_usart3();
    uint32_t now_ms = HAL_GetTick();
#if !Y_LIFT_TEST_MODE
    vl53l0x_pair_service(now_ms);
#endif
#if DIRECT_TOF_TEST_MODE
    {
        vl53l0x_pair_snapshot_t tof;
        vl53l0x_pair_get_snapshot(&tof);
        if ((!tof.left_online || !tof.right_online) &&
            (now_ms - direct_tof_last_retry_ms) >= DIRECT_TOF_RETRY_PERIOD_MS) {
            direct_tof_last_retry_ms = now_ms;
            dbg_printf(">> I2C2 recovery: %s\r\n",
                       direct_tof_recover_bus()
                           ? "SDA/SCL released"
                           : "line still held LOW");
            direct_tof_ok = vl53l0x_pair_init(&hi2c2);
            dbg_printf(">> ToF rescan: %s - %s (error=%d)\r\n",
                       direct_tof_ok ? "FOUND" : "not found",
                       vl53l0x_pair_init_stage(), vl53l0x_pair_init_error());
            direct_tof_scan_bus();
        }
    }
#if DIRECT_TOF_BENCH_ENABLE
    direct_tof_bench_service(now_ms);
#endif
#endif
#if DIRECT_TOF_TEST_MODE
    if ((now_ms - direct_tof_last_print_ms) >= DIRECT_TOF_PRINT_PERIOD_MS) {
        direct_tof_last_print_ms = now_ms;
        direct_tof_print_status(now_ms);
    }
#endif
    if (lift_timed_active && (now_ms - lift_started_ms) >= lift_duration_ms) {
        lift_stop_outputs();
        dbg_printf("OK: LIFT complete -> M2/M5 stopped\r\n");
    }
    if (motor_coordinated_service()) {
        dbg_printf("OK: coordinated step run complete -> selected motors stopped\r\n");
    }
    hx711_parallel_service(now_ms);
    if (load_cell_stream_enabled &&
        (now_ms - load_cell_last_print_ms) >= LOAD_CELL_PRINT_PERIOD_MS) {
        load_cell_last_print_ms = now_ms;
        load_cell_print_status();
    }
    if (switch_stream_enabled &&
        (now_ms - switch_last_print_ms) >= SWITCH_PRINT_PERIOD_MS) {
        switch_last_print_ms = now_ms;
        switch_print_status();
    }
    if (terminal_log_enabled) {
        sensor_node_emit_status(HAL_GetTick(),
                                &sensor_last_print_ms,
                                &sensor_last_valid_frames,
                                &sensor_last_rx_bytes);
    }

    /* --- BNO055 poll. Target 100 Hz (10 ms period). Note: while the motor
     * test loop below is running its HAL_Delay()s, this block only fires
     * between cycles -- so during the test you'll see less than 100 Hz.
     * Once you remove the test code and re-enable the CLI, this becomes
     * the real 100 Hz polling path with no changes. */
    if (imu_ok && (HAL_GetTick() - imu_last_poll_ms) >= IMU_POLL_PERIOD_MS) {
        imu_last_poll_ms = HAL_GetTick();
        bool e_ok = bno055_read_euler(&imu_euler);
        bool c_ok = bno055_read_calib(&imu_calib);
        if (e_ok && c_ok) imu_read_count++;
        else              imu_err_count++;

        if (terminal_log_enabled &&
            (HAL_GetTick() - imu_last_print_ms) >= IMU_PRINT_PERIOD_MS) {
            imu_last_print_ms = HAL_GetTick();
            dbg_printf("IMU H=%.1f R=%.1f P=%.1f  calib S/G/A/M=%u/%u/%u/%u  reads=%lu errs=%lu\r\n",
                       (double)imu_euler.heading_deg,
                       (double)imu_euler.roll_deg,
                       (double)imu_euler.pitch_deg,
                       imu_calib.sys, imu_calib.gyro,
                       imu_calib.accel, imu_calib.mag,
                       (unsigned long)imu_read_count,
                       (unsigned long)imu_err_count);
        }
    }

    /* --- ROS drive command dispatch. The USART2 RX ISR sets cmd_ready once a
     * full '\n'-terminated line sits in cmd_buf. Copy it out, clear the flag
     * so the ISR can refill, then parse and drive. */
#if !DIRECT_TOF_TEST_MODE
    if (cmd_ready) {
        char line[CMD_BUF_SIZE];
        strncpy(line, (const char *)cmd_buf, sizeof line);
        line[sizeof line - 1] = '\0';
        cmd_ready = false;
#if Y_LIFT_TEST_MODE
        const char *lift_line = cli_skip_ws(line);
        if (cli_word_is(lift_line, "LIFT")) {
            cli_run_lift(lift_line + 4);
        } else if (*lift_line == 'S' || *lift_line == 's' ||
                   *lift_line == 'A' || *lift_line == 'a') {
            lift_stop_outputs();
            motor_stop_all();
            dbg_printf("OK: bounded Y lift stopped\r\n");
        } else {
            motor_stop_all();
            dbg_printf("ERR: bounded mode accepts only LIFT UP/DOWN or S/A\r\n");
        }
#else
        cli_apply_line(line);
#endif
    }

    /* --- Command watchdog. If the Pi stops streaming V lines (crashed bridge,
     * unplugged cable), stop both wheels so the robot can't run away. The Pi
     * also sends "V 0 0" on its own 0.5 s timeout; this is the backstop for
     * when even that doesn't arrive. */
    if (!drive_timed_out &&
        (HAL_GetTick() - drive_last_cmd_ms) > DRIVE_CMD_TIMEOUT_MS) {
        drive_stop_all();
        drive_timed_out = true;
        dbg_printf("!! cmd timeout (%u ms) -> wheels stopped\r\n",
                   (unsigned)DRIVE_CMD_TIMEOUT_MS);
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
#if DIRECT_TOF_LONG_CABLE_MODE
  /* The direct VL53L0X modules are approximately two metres from the MCU.
   * Slow the diagnostic bus to give the long cable substantially more rise
   * and settling time. */
  hi2c2.Init.ClockSpeed = 10000;
#else
  hi2c2.Init.ClockSpeed = 100000;
#endif
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
 
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
 
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
 
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* APB1 timer clock is 84 MHz. Divide to a 1 MHz counter. */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
#if LAC_RC_TEST_MODE
  htim2.Init.Period = 19999; /* 50 Hz RC frame */
#else
  htim2.Init.Period = 999;   /* 1 kHz VC PWM */
#endif
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
 
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
 
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
 
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
 
  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */
 
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
 
  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
 
  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */
 
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
 
  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
 
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
 
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* BLD-515C RS-485 link runs at 9600 baud. Override the CubeMX default
   * 115200 and re-init. Survives Generate Code because this is inside a USER
   * CODE block. Cleaner alternative: set USART1 baud to 9600 in the IOC,
   * regenerate, and delete this block. */
  huart1.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
 
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
 
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
 
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USART3 is no longer used for direct DYP-A02 polling. It remains a
   * 115200-baud RS485 fallback input for the ESP32 sensor node. */
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
 
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Hold both VL53L0X sensors in shutdown until they can be addressed one at
   * a time without colliding at the shared factory address. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC1 PC2 PC3
                           PC4 PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure D13/PA5 as the direct ToF XSHUT output. The onboard LD2 LED is
   * also attached to PA5 and will harmlessly mirror this line. */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure PD2 as the second direct ToF XSHUT output. */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB14 PB15
                           PB4 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
 
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
 
/* USART2 uses single-byte interrupt RX for the test command interface;
 * we re-arm immediately after every byte so the RX never stops. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        usart2_rx_count++;
        if (!cmd_ready) {
            char c = (char)usart2_rx_byte;
            if (c == '\n' || c == '\r') {
                cmd_buf[cmd_idx] = '\0';
                if (cmd_idx > 0) {
                    cmd_ready = true;
                    usart2_line_count++;
                }
                cmd_idx = 0;
            } else if (cmd_idx < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_idx++] = c;
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
    }
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
 
/* On any UART error (overrun, framing, noise) HAL aborts the active DMA.
 * Restart it so we don't go deaf permanently. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        usart3_rx_dma_read_pos = 0u;
        sensor_parser_usart3.idx = 0u;
        sensor_parser_usart3.expected_len = 0u;
        HAL_UART_Receive_DMA(&huart3, usart3_rx_dma_buf, USART3_RX_DMA_BUF_SIZE);
    } else if (huart->Instance == USART2) {
        cmd_idx = 0u;
        cmd_ready = false;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
    }
}
 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
