#include "vl53l0x_pair.h"

#include <string.h>

#include "vl53l0x_api.h"

#define VL53L0X_DEFAULT_ADDR_8BIT  (0x29u << 1)
#define VL53L0X_RIGHT_ADDR_8BIT    (0x30u << 1)
#define VL53L0X_MODEL_ID           0xEEAAu
#define VL53L0X_DEFAULT_BUDGET_US  33000u
#define VL53L0X_SHOT_TIMEOUT_MS    350u
#define VL53L0X_DEFAULT_GAP_MS     120u
#define VL53L0X_READY_POLL_MS       25u
#define VL53L0X_BOOT_DELAY_MS      50u
#define VL53L0X_PROBE_ATTEMPTS     4u
#define VL53L0X_PROBE_WAIT_MS      750u
#define VL53L0X_RUNTIME_MAX_ERRORS 3u
#define VL53L0X_AVERAGE_WINDOW_MS  1000u
#define VL53L0X_AVERAGE_CAPACITY     16u
#define VL53L0X_SINGLE_SENSOR_TEST 0

/* Legacy software names are retained to avoid a risky broad rewrite, but the
 * physical installation was verified in the robot as follows:
 *   TOF_LEFT  / D13 / PA5 = physical RIGHT conveyor sensor (working)
 *   TOF_RIGHT / PD2       = physical LEFT conveyor sensor (unreliable)
 */
#define LEFT_XSHUT_PORT GPIOA
#define LEFT_XSHUT_PIN  GPIO_PIN_5
#define RIGHT_XSHUT_PORT GPIOD
#define RIGHT_XSHUT_PIN  GPIO_PIN_2

/* Application override in main.c. With both sensors in XSHUT this clears a
 * stale controller BUSY state and verifies that neither cable is held low
 * before either VL53L0X is allowed back onto the long bus. */
int VL53L0X_I2C_Recover(I2C_HandleTypeDef *i2c);

typedef struct {
    VL53L0X_Dev_t dev;
    uint16_t distance_mm;
    uint16_t raw_distance_mm;
    uint8_t range_status;
    bool online;
    uint8_t consecutive_errors;
    uint32_t samples;
    uint32_t errors;
    uint32_t last_update_ms;
    uint16_t average_mm[VL53L0X_AVERAGE_CAPACITY];
    uint32_t average_time_ms[VL53L0X_AVERAGE_CAPACITY];
    uint8_t average_head;
    uint8_t average_count;
} tof_sensor_t;

enum {
    TOF_LEFT = 0,
    TOF_RIGHT = 1,
    TOF_COUNT = 2
};

static tof_sensor_t sensors[TOF_COUNT];
static int active_sensor = -1;
static uint8_t next_sensor = TOF_LEFT;
static uint32_t shot_started_ms;
static uint32_t next_shot_ms;
static uint32_t next_ready_poll_ms;
static uint32_t timing_budget_us = VL53L0X_DEFAULT_BUDGET_US;
static uint32_t intershot_gap_ms = VL53L0X_DEFAULT_GAP_MS;
static const char *init_stage = "not started";
static int init_error;

static void reset_measurement_state(void)
{
    active_sensor = -1;
    next_sensor = TOF_LEFT;
    shot_started_ms = 0u;
    next_shot_ms = 0u;
    next_ready_poll_ms = 0u;
}

static void reset_average(tof_sensor_t *sensor)
{
    sensor->distance_mm = 0xFFFFu;
    sensor->raw_distance_mm = 0xFFFFu;
    sensor->average_head = 0u;
    sensor->average_count = 0u;
}

static void recompute_average(tof_sensor_t *sensor)
{
    if (sensor->average_count == 0u) {
        sensor->distance_mm = 0xFFFFu;
        return;
    }

    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < sensor->average_count; ++i) {
        uint8_t index = (uint8_t)((sensor->average_head + i) %
                                  VL53L0X_AVERAGE_CAPACITY);
        sum += sensor->average_mm[index];
    }
    sensor->distance_mm = (uint16_t)(
        (sum + (sensor->average_count / 2u)) / sensor->average_count);
}

static void prune_average(tof_sensor_t *sensor, uint32_t now_ms)
{
    while (sensor->average_count > 0u) {
        uint8_t oldest = sensor->average_head;
        if ((now_ms - sensor->average_time_ms[oldest]) <=
            VL53L0X_AVERAGE_WINDOW_MS) {
            break;
        }
        sensor->average_head = (uint8_t)(
            (sensor->average_head + 1u) % VL53L0X_AVERAGE_CAPACITY);
        sensor->average_count--;
    }

    recompute_average(sensor);
}

static void update_average(tof_sensor_t *sensor, uint16_t distance_mm,
                           uint32_t now_ms)
{
    prune_average(sensor, now_ms);

    uint8_t insert;
    if (sensor->average_count < VL53L0X_AVERAGE_CAPACITY) {
        insert = (uint8_t)((sensor->average_head + sensor->average_count) %
                           VL53L0X_AVERAGE_CAPACITY);
        sensor->average_count++;
    } else {
        insert = sensor->average_head;
        sensor->average_head = (uint8_t)(
            (sensor->average_head + 1u) % VL53L0X_AVERAGE_CAPACITY);
    }

    sensor->average_mm[insert] = distance_mm;
    sensor->average_time_ms[insert] = now_ms;
    recompute_average(sensor);
}

static void record_measurement(tof_sensor_t *sensor,
                               const VL53L0X_RangingMeasurementData_t *measurement,
                               uint32_t now_ms)
{
    sensor->raw_distance_mm = measurement->RangeMilliMeter;
    sensor->range_status = measurement->RangeStatus;
    sensor->consecutive_errors = 0u;
    sensor->samples++;
    sensor->last_update_ms = now_ms;

    if (measurement->RangeStatus == 0u) {
        update_average(sensor, measurement->RangeMilliMeter, now_ms);
    } else {
        prune_average(sensor, now_ms);
    }
}

static bool status_allows_continue(VL53L0X_Error status)
{
    return status == VL53L0X_ERROR_NONE ||
           status == VL53L0X_ERROR_CALIBRATION_WARNING;
}

static bool device_acknowledges(tof_sensor_t *sensor, uint16_t address)
{
    sensor->dev.I2cDevAddr = address;
    uint32_t started_ms = HAL_GetTick();
    do {
        if (HAL_I2C_IsDeviceReady(sensor->dev.I2cHandle,
                                  address,
                                  VL53L0X_PROBE_ATTEMPTS,
                                  50u) == HAL_OK) {
            return true;
        }
        HAL_Delay(20u);
    } while ((HAL_GetTick() - started_ms) < VL53L0X_PROBE_WAIT_MS);

    return false;
}

static bool device_present(tof_sensor_t *sensor, uint16_t address)
{
    uint16_t id = 0u;
    if (!device_acknowledges(sensor, address)) {
        return false;
    }

    for (uint8_t attempt = 0u; attempt < VL53L0X_PROBE_ATTEMPTS; ++attempt) {
        if (VL53L0X_RdWord(&sensor->dev,
                           VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                           &id) == VL53L0X_ERROR_NONE) {
            if (id == VL53L0X_MODEL_ID) {
                return true;
            }
        }
        HAL_Delay(2u);
    }

    /* An ACK at 0x29/0x30 is enough to attempt the ST API. This keeps one
     * marginal model-ID read from masquerading as a physically absent part. */
    return true;
}

static bool drive_xshut(GPIO_TypeDef *port, uint16_t pin,
                         GPIO_PinState state)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
    HAL_GPIO_WritePin(port, pin, state);
    HAL_Delay(2u);

    return HAL_GPIO_ReadPin(port, pin) == state;
}

static bool drive_left_xshut(GPIO_PinState state)
{
    return drive_xshut(LEFT_XSHUT_PORT, LEFT_XSHUT_PIN, state);
}

static bool drive_right_xshut(GPIO_PinState state)
{
    return drive_xshut(RIGHT_XSHUT_PORT, RIGHT_XSHUT_PIN, state);
}

static bool assign_right_address(tof_sensor_t *sensor)
{
    for (uint8_t attempt = 0u; attempt < VL53L0X_PROBE_ATTEMPTS; ++attempt) {
        sensor->dev.I2cDevAddr = VL53L0X_DEFAULT_ADDR_8BIT;
        VL53L0X_Error status = VL53L0X_SetDeviceAddress(
            &sensor->dev, VL53L0X_RIGHT_ADDR_8BIT);

        HAL_Delay(5u);
        if (device_present(sensor, VL53L0X_RIGHT_ADDR_8BIT)) {
            return true;
        }

        init_error = (int)status;
        if (!device_acknowledges(sensor, VL53L0X_DEFAULT_ADDR_8BIT)) {
            break;
        }
    }

    sensor->dev.I2cDevAddr = VL53L0X_RIGHT_ADDR_8BIT;
    return false;
}

static bool configure_device(tof_sensor_t *sensor, const char *stage)
{
    VL53L0X_Error status;
    uint8_t vhv_settings = 0u;
    uint8_t phase_cal = 0u;
    uint32_t ref_spad_count = 0u;
    uint8_t is_aperture_spads = 0u;

    sensor->dev.comms_type = 1u;
    sensor->dev.comms_speed_khz = 10u;

    init_stage = stage;
    status = VL53L0X_DataInit(&sensor->dev);
    if (!status_allows_continue(status)) goto failed;

    init_stage = sensor == &sensors[TOF_LEFT]
                     ? "left StaticInit failed"
                     : "right StaticInit failed";
    status = VL53L0X_StaticInit(&sensor->dev);
    if (status != VL53L0X_ERROR_NONE) goto failed;

    init_stage = sensor == &sensors[TOF_LEFT]
                     ? "left reference calibration failed"
                     : "right reference calibration failed";
    status = VL53L0X_PerformRefCalibration(&sensor->dev,
                                            &vhv_settings,
                                            &phase_cal);
    if (status != VL53L0X_ERROR_NONE) goto failed;

    init_stage = sensor == &sensors[TOF_LEFT]
                     ? "left SPAD management failed"
                     : "right SPAD management failed";
    status = VL53L0X_PerformRefSpadManagement(&sensor->dev,
                                               &ref_spad_count,
                                               &is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE) goto failed;

    init_stage = sensor == &sensors[TOF_LEFT]
                     ? "left ranging-mode setup failed"
                     : "right ranging-mode setup failed";
    status = VL53L0X_SetDeviceMode(&sensor->dev,
                                    VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) goto failed;

    init_stage = sensor == &sensors[TOF_LEFT]
                     ? "left timing-budget setup failed"
                     : "right timing-budget setup failed";
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        &sensor->dev, timing_budget_us);
    if (status != VL53L0X_ERROR_NONE) goto failed;

    sensor->online = true;
    sensor->consecutive_errors = 0u;
    reset_average(sensor);
    sensor->range_status = 0xFFu;
    return true;

failed:
    sensor->online = false;
    sensor->errors++;
    init_error = (int)status;
    return false;
}

bool vl53l0x_pair_init(I2C_HandleTypeDef *i2c)
{
    reset_measurement_state();
    init_error = 0;

    sensors[TOF_LEFT].online = false;
    sensors[TOF_RIGHT].online = false;
    sensors[TOF_LEFT].consecutive_errors = 0u;
    sensors[TOF_RIGHT].consecutive_errors = 0u;
    sensors[TOF_LEFT].dev.I2cHandle = i2c;
    sensors[TOF_LEFT].dev.I2cDevAddr = VL53L0X_DEFAULT_ADDR_8BIT;
    sensors[TOF_RIGHT].dev.I2cHandle = i2c;
    sensors[TOF_RIGHT].dev.I2cDevAddr = VL53L0X_DEFAULT_ADDR_8BIT;

    /* A full hardware reset on every attempt makes retries deterministic:
     * both sensors always restart at the factory address 0x29. */
    init_stage = "holding both XSHUT lines low";
    if (!drive_left_xshut(GPIO_PIN_RESET)) {
        init_stage = "D13/left XSHUT did not reach LOW";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }
    if (!drive_right_xshut(GPIO_PIN_RESET)) {
        init_stage = "PD2/right XSHUT did not reach LOW";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }
    HAL_Delay(VL53L0X_BOOT_DELAY_MS);

    init_stage = "recovering I2C with both sensors in XSHUT";
    if (VL53L0X_I2C_Recover(i2c) != 0) {
        init_stage = "SDA/SCL held low with both sensors in XSHUT";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }
    HAL_Delay(5u);

    init_stage = "releasing PD2/right XSHUT";
    if (!drive_right_xshut(GPIO_PIN_SET)) {
        init_stage = "PD2/right XSHUT did not reach HIGH";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }
    HAL_Delay(VL53L0X_BOOT_DELAY_MS);

    bool right_at_default = device_present(&sensors[TOF_RIGHT],
                                            VL53L0X_DEFAULT_ADDR_8BIT);
#if VL53L0X_SINGLE_SENSOR_TEST
    if (right_at_default) {
        if (configure_device(&sensors[TOF_RIGHT],
                             "single sensor DataInit failed")) {
            init_stage = "single PD2 sensor ready at 0x29";
        }
    } else {
        init_stage = "single PD2 sensor not found at 0x29";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return sensors[TOF_RIGHT].online;
#else
    /* If only the controller reset while the remote sensor rail stayed up,
     * the PD2 device can still be answering at its previously assigned 0x30
     * address. Accept that state instead of unnecessarily rejecting a live
     * sensor on the long cable. A working XSHUT reset normally returns it to
     * 0x29, so this is strictly a recovery fallback. */
    bool right_at_assigned = false;
    if (!right_at_default) {
        right_at_assigned = device_present(&sensors[TOF_RIGHT],
                                            VL53L0X_RIGHT_ADDR_8BIT);
    }

    if (!right_at_default && !right_at_assigned) {
        /* Keep the missing PD2 branch shut down and bring up D13 by itself.
         * This preserves useful ranging while the application periodically
         * retries the missing branch. */
        (void)drive_right_xshut(GPIO_PIN_RESET);
        bool left_present = false;
        if (drive_left_xshut(GPIO_PIN_SET)) {
            HAL_Delay(VL53L0X_BOOT_DELAY_MS);
            left_present = device_present(&sensors[TOF_LEFT],
                                           VL53L0X_DEFAULT_ADDR_8BIT);
        }
        if (left_present && configure_device(&sensors[TOF_LEFT],
                                             "left fallback DataInit failed")) {
            init_stage = "left ready in fallback mode; PD2 sensor missing";
            return true;
        }

        (void)drive_left_xshut(GPIO_PIN_RESET);
        if (!left_present) {
            init_stage = "neither sensor responds when tested separately";
        }
        if (init_error == 0) {
            init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        }
        return false;
    }

    if (right_at_default) {
        init_stage = "assigning PD2/right sensor address 0x30";
        if (!assign_right_address(&sensors[TOF_RIGHT])) {
            sensors[TOF_RIGHT].online = false;
            init_stage = "right address assignment/verification failed";
            if (init_error == 0) {
                init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
            }
            return false;
        }
    } else {
        init_stage = "PD2/right recovered at retained address 0x30";
    }

    bool right_configured = configure_device(&sensors[TOF_RIGHT],
                                              "physical-left/PD2 DataInit failed");
    if (!right_configured) {
        /* PD2 is the unreliable physical-left branch. Do not let its failed
         * calibration prevent the known-good physical-right D13 sensor from
         * providing conveyor position feedback. Shut PD2 down and bring D13
         * up alone at the factory address. */
        (void)drive_right_xshut(GPIO_PIN_RESET);
        sensors[TOF_RIGHT].online = false;

        bool physical_right_present = false;
        if (drive_left_xshut(GPIO_PIN_SET)) {
            HAL_Delay(VL53L0X_BOOT_DELAY_MS);
            physical_right_present = device_present(
                &sensors[TOF_LEFT], VL53L0X_DEFAULT_ADDR_8BIT);
        }

        if (physical_right_present &&
            configure_device(&sensors[TOF_LEFT],
                             "physical-right/D13 DataInit failed")) {
            init_stage = "physical-right/D13 ready; physical-left/PD2 failed";
            return true;
        }

        (void)drive_left_xshut(GPIO_PIN_RESET);
        if (!physical_right_present) {
            init_stage = "physical-right/D13 missing after PD2 failure";
        }
        return false;
    }

    init_stage = "releasing D13/left XSHUT";
    if (!drive_left_xshut(GPIO_PIN_SET)) {
        sensors[TOF_LEFT].online = false;
        init_stage = "D13/left XSHUT did not reach HIGH";
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }
    HAL_Delay(VL53L0X_BOOT_DELAY_MS);

    bool left_at_default = device_present(&sensors[TOF_LEFT],
                                           VL53L0X_DEFAULT_ADDR_8BIT);
    if (!left_at_default && init_error == 0) {
        init_error = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    bool left_configured = false;
    if (left_at_default) {
        left_configured = configure_device(&sensors[TOF_LEFT],
                                            "left DataInit failed");
    }

    if (sensors[TOF_LEFT].online && sensors[TOF_RIGHT].online) {
        init_stage = "both sensors ready";
    } else if (sensors[TOF_LEFT].online) {
        init_stage = "left ready; right missing";
    } else if (sensors[TOF_RIGHT].online) {
        init_stage = "right ready; left missing or XSHUT not released";
    } else if (left_at_default && !left_configured) {
        /* configure_device() left the exact failed API step in init_stage. */
    } else if (!left_at_default) {
        init_stage = "no sensor at 0x29 after releasing D13 XSHUT";
    } else {
        init_stage = "both sensors failed initialization";
    }

    return sensors[TOF_LEFT].online && sensors[TOF_RIGHT].online;
#endif
}

bool vl53l0x_pair_set_profile(uint32_t requested_budget_us,
                             uint32_t requested_gap_ms)
{
    if (requested_budget_us < 20000u || requested_budget_us > 200000u ||
        requested_gap_ms > 1000u) {
        return false;
    }

    if (active_sensor >= 0) {
        (void)VL53L0X_StopMeasurement(&sensors[active_sensor].dev);
        active_sensor = -1;
    }

    timing_budget_us = requested_budget_us;
    intershot_gap_ms = requested_gap_ms;

    bool configured_any = false;
    for (uint8_t index = 0u; index < TOF_COUNT; ++index) {
        if (!sensors[index].online) continue;

        VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
            &sensors[index].dev, timing_budget_us);
        if (status != VL53L0X_ERROR_NONE) {
            init_stage = index == TOF_LEFT
                             ? "left timing-profile update failed"
                             : "right timing-profile update failed";
            init_error = (int)status;
            return false;
        }

        reset_average(&sensors[index]);
        configured_any = true;
    }

    reset_measurement_state();
    next_shot_ms = HAL_GetTick();
    return configured_any;
}

static int find_next_online_sensor(void)
{
    for (uint8_t checked = 0u; checked < TOF_COUNT; ++checked) {
        uint8_t index = (uint8_t)((next_sensor + checked) % TOF_COUNT);
        if (sensors[index].online) {
            next_sensor = (uint8_t)((index + 1u) % TOF_COUNT);
            return (int)index;
        }
    }
    return -1;
}

void vl53l0x_pair_service(uint32_t now_ms)
{
#if VL53L0X_SINGLE_SENSOR_TEST
    tof_sensor_t *sensor = &sensors[TOF_RIGHT];
    if (!sensor->online || (int32_t)(now_ms - next_shot_ms) < 0) return;

    VL53L0X_RangingMeasurementData_t measurement;
    VL53L0X_Error status = VL53L0X_PerformSingleRangingMeasurement(
        &sensor->dev, &measurement);
    if (status == VL53L0X_ERROR_NONE) {
        record_measurement(sensor, &measurement, HAL_GetTick());
    } else {
        sensor->errors++;
        if (sensor->consecutive_errors < UINT8_MAX) {
            sensor->consecutive_errors++;
        }
        if (sensor->consecutive_errors >= 3u) {
            sensor->online = false;
            init_stage = "single sensor lost during ranging";
            init_error = (int)status;
        }
    }
    next_shot_ms = HAL_GetTick() + 100u;
    return;
#else
    if (active_sensor >= 0) {
        if ((int32_t)(now_ms - next_ready_poll_ms) < 0) return;
        next_ready_poll_ms = now_ms + VL53L0X_READY_POLL_MS;

        tof_sensor_t *sensor = &sensors[active_sensor];
        uint8_t ready = 0u;
        VL53L0X_Error status = VL53L0X_GetMeasurementDataReady(
            &sensor->dev, &ready);

        if (status != VL53L0X_ERROR_NONE) {
            sensor->errors++;
            if (sensor->consecutive_errors < UINT8_MAX) {
                sensor->consecutive_errors++;
            }
            if (sensor->consecutive_errors >= VL53L0X_RUNTIME_MAX_ERRORS) {
                sensor->online = false;
                init_stage = active_sensor == TOF_LEFT
                                 ? "left sensor lost during ready polling"
                                 : "right sensor lost during ready polling";
                init_error = (int)status;
            }
            active_sensor = -1;
            next_shot_ms = now_ms + intershot_gap_ms;
            return;
        }

        if (ready != 0u) {
            VL53L0X_RangingMeasurementData_t measurement;
            status = VL53L0X_GetRangingMeasurementData(&sensor->dev,
                                                        &measurement);
            if (status == VL53L0X_ERROR_NONE) {
                record_measurement(sensor, &measurement, now_ms);
            } else {
                sensor->errors++;
                if (sensor->consecutive_errors < UINT8_MAX) {
                    sensor->consecutive_errors++;
                }
            }

            if (VL53L0X_ClearInterruptMask(&sensor->dev, 0u) !=
                VL53L0X_ERROR_NONE) {
                sensor->errors++;
                if (sensor->consecutive_errors < UINT8_MAX) {
                    sensor->consecutive_errors++;
                }
            }
            if (sensor->consecutive_errors >= VL53L0X_RUNTIME_MAX_ERRORS) {
                sensor->online = false;
                init_stage = active_sensor == TOF_LEFT
                                 ? "left sensor lost while reading range"
                                 : "right sensor lost while reading range";
                init_error = (int)status;
            }
            active_sensor = -1;
            next_shot_ms = now_ms + intershot_gap_ms;
            return;
        }

        if ((now_ms - shot_started_ms) > VL53L0X_SHOT_TIMEOUT_MS) {
            (void)VL53L0X_StopMeasurement(&sensor->dev);
            sensor->errors++;
            if (sensor->consecutive_errors < UINT8_MAX) {
                sensor->consecutive_errors++;
            }
            if (sensor->consecutive_errors >= VL53L0X_RUNTIME_MAX_ERRORS) {
                sensor->online = false;
                init_stage = active_sensor == TOF_LEFT
                                 ? "left sensor timed out"
                                 : "right sensor timed out";
                init_error = VL53L0X_ERROR_TIME_OUT;
            }
            active_sensor = -1;
            next_shot_ms = now_ms + intershot_gap_ms;
        }
        return;
    }

    if ((int32_t)(now_ms - next_shot_ms) < 0) return;

    int index = find_next_online_sensor();
    if (index < 0) return;

    VL53L0X_Error status = VL53L0X_StartMeasurement(&sensors[index].dev);
    if (status == VL53L0X_ERROR_NONE) {
        active_sensor = index;
        shot_started_ms = now_ms;
        next_ready_poll_ms = now_ms + VL53L0X_READY_POLL_MS;
    } else {
        sensors[index].errors++;
        if (sensors[index].consecutive_errors < UINT8_MAX) {
            sensors[index].consecutive_errors++;
        }
        if (sensors[index].consecutive_errors >= VL53L0X_RUNTIME_MAX_ERRORS) {
            sensors[index].online = false;
            init_stage = index == TOF_LEFT
                             ? "left sensor lost while starting range"
                             : "right sensor lost while starting range";
            init_error = (int)status;
        }
        next_shot_ms = now_ms + intershot_gap_ms;
    }
#endif
}

void vl53l0x_pair_get_snapshot(vl53l0x_pair_snapshot_t *out)
{
    if (out == NULL) return;

    out->left_mm = sensors[TOF_LEFT].distance_mm;
    out->right_mm = sensors[TOF_RIGHT].distance_mm;
    out->left_raw_mm = sensors[TOF_LEFT].raw_distance_mm;
    out->right_raw_mm = sensors[TOF_RIGHT].raw_distance_mm;
    out->left_average_samples = sensors[TOF_LEFT].average_count;
    out->right_average_samples = sensors[TOF_RIGHT].average_count;
    out->left_range_status = sensors[TOF_LEFT].range_status;
    out->right_range_status = sensors[TOF_RIGHT].range_status;
    out->left_online = sensors[TOF_LEFT].online;
    out->right_online = sensors[TOF_RIGHT].online;
    out->left_samples = sensors[TOF_LEFT].samples;
    out->right_samples = sensors[TOF_RIGHT].samples;
    out->left_errors = sensors[TOF_LEFT].errors;
    out->right_errors = sensors[TOF_RIGHT].errors;
    out->left_last_update_ms = sensors[TOF_LEFT].last_update_ms;
    out->right_last_update_ms = sensors[TOF_RIGHT].last_update_ms;
    out->timing_budget_us = timing_budget_us;
    out->intershot_gap_ms = intershot_gap_ms;
}

const char *vl53l0x_pair_init_stage(void)
{
    return init_stage;
}

int vl53l0x_pair_init_error(void)
{
    return init_error;
}
