/* ESP32 RS485 sensor-node ingest
 * ---------------------------------------------------------------------------
 * USART3 streams AA 55 LEN CMD PAYLOAD CS frames from two ESP32 nodes:
 *   CMD=0x01 -> four VL53L0X distances + one ultrasonic
 *   CMD=0x02 -> three VL53L5CX grids, fan duty and online/stop masks
 * CS is the low byte of the sum of LEN, CMD and all payload bytes.
 *
 * sensor_node_feed() builds a frame byte-by-byte; sensor_node_service_usart3()
 * drains the DMA ring; sensor_node_emit_status() rate-limits the two status
 * prints. */

#include "sensor_node_app.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "debug.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/* Owned by main.c -- USART3 is dedicated to the RS485 sensor bus. */
extern UART_HandleTypeDef huart3;

/* ===========================================================================
 *                               Frame format
 * ========================================================================= */

#define SENSOR_VL53L5_COUNT        3u
#define SENSOR_VL53L5_ZONES        16u
#define SENSOR_FRAME_MAX_PAYLOAD   (3u + (SENSOR_VL53L5_COUNT * SENSOR_VL53L5_ZONES * 2u))
#define SENSOR_FRAME_MAX_LEN       (SENSOR_FRAME_MAX_PAYLOAD + 5u)
#define SENSOR_CMD_VL53L0X_ULTRA   0x01u
#define SENSOR_CMD_VL53L5_GRID     0x02u
#define SENSOR_VL53L0X_PAYLOAD_LEN 10u
#define SENSOR_VL53L5_PAYLOAD_LEN  SENSOR_FRAME_MAX_PAYLOAD

#define SENSOR_NODE_TIMEOUT_MS     300u
#define SENSOR_NODE_INVALID_MM     9999u
#define SENSOR_GRID_INVALID_MM     0xFFFFu
#define SENSOR_STOP_THRESHOLD_MM   250u
#define SENSOR_PRINT_PERIOD_MS     250u

typedef enum {
    SENSOR_SRC_NONE = 0,
    SENSOR_SRC_USART3
} sensor_source_t;

typedef struct {
    uint16_t tof_mm[4];
    uint16_t ultra_mm;
    sensor_source_t source;
    uint32_t last_update_ms;
    uint32_t valid_frames;
    uint32_t bad_checksum_count;
    uint32_t bad_format_count;
    uint32_t rx_bytes;        /* raw bytes consumed -- 0 = RS485 wire dead */
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

typedef struct {
    uint8_t frame[SENSOR_FRAME_MAX_LEN];
    uint16_t idx;
    uint16_t expected_len;
} sensor_parser_t;

/* ===========================================================================
 *                                   State
 * ========================================================================= */

#define USART3_RX_DMA_BUF_SIZE  512u
static uint8_t  usart3_rx_dma_buf[USART3_RX_DMA_BUF_SIZE];
static uint32_t usart3_rx_dma_read_pos = 0u;

static sensor_node_t sensor_node = {
    { 9999u, 9999u, 9999u, 9999u },
    9999u,
    SENSOR_SRC_NONE,
    0u, 0u, 0u, 0u, 0u
};

static vl53l5_node_t vl53l5_node = {0};

static sensor_parser_t sensor_parser_usart3 = { {0}, 0u, 0u };

/* ===========================================================================
 *                                 Helpers
 * ========================================================================= */

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

/* ===========================================================================
 *                           Frame accept + parser
 * ========================================================================= */

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

static void sensor_node_feed(sensor_parser_t *parser,
                             sensor_source_t source,
                             uint8_t b)
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

/* ===========================================================================
 *                              Liveness + stop
 * ========================================================================= */

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

/* ===========================================================================
 *                               Status prints
 * ========================================================================= */

/* Print one SENSOR status line for the VL53L0X + ultrasonic node. Pre-WAIT
 * state (no good frame yet) prints a shorter line; once frames come in the
 * full line with age/fps is emitted. */
static void sensor_node_print_vl53l0x_status(uint32_t now, uint32_t dt,
                                             uint32_t last_valid_frames,
                                             uint32_t last_rx_bytes)
{
    uint32_t frames_delta = sensor_node.valid_frames - last_valid_frames;
    uint32_t rx_delta     = sensor_node.rx_bytes - last_rx_bytes;
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
}

/* Print one L5 status line for the VL53L5CX grid + fan-control node. */
static void sensor_node_print_vl53l5_status(uint32_t now)
{
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

/* ===========================================================================
 *                                Public API
 * ========================================================================= */

void sensor_node_start_dma(void)
{
    usart3_rx_dma_read_pos = 0u;
    sensor_parser_usart3.idx = 0u;
    sensor_parser_usart3.expected_len = 0u;
    (void)HAL_UART_Receive_DMA(&huart3, usart3_rx_dma_buf, USART3_RX_DMA_BUF_SIZE);
}

void sensor_node_restart_dma(void)
{
    usart3_rx_dma_read_pos = 0u;
    sensor_parser_usart3.idx = 0u;
    sensor_parser_usart3.expected_len = 0u;
    HAL_UART_Receive_DMA(&huart3, usart3_rx_dma_buf, USART3_RX_DMA_BUF_SIZE);
}

void sensor_node_service_usart3(void)
{
    uint32_t write_pos =
        USART3_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
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

void sensor_node_emit_status(uint32_t now,
                             uint32_t *last_print_ms,
                             uint32_t *last_valid_frames,
                             uint32_t *last_rx_bytes)
{
    if ((now - *last_print_ms) < SENSOR_PRINT_PERIOD_MS) {
        return;
    }

    uint32_t dt = (now > *last_print_ms) ? (now - *last_print_ms)
                                         : SENSOR_PRINT_PERIOD_MS;
    if (*last_print_ms == 0u) {
        dt = SENSOR_PRINT_PERIOD_MS;
    }

    sensor_node_print_vl53l0x_status(now, dt, *last_valid_frames,
                                     *last_rx_bytes);

    *last_print_ms     = now;
    *last_valid_frames = sensor_node.valid_frames;
    *last_rx_bytes     = sensor_node.rx_bytes;

    sensor_node_print_vl53l5_status(now);
}
