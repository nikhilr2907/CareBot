#ifndef SENSOR_NODE_APP_H
#define SENSOR_NODE_APP_H

#include <stdint.h>

/* ESP32 RS485 sensor-node ingest.
 *
 * USART3 (DMA RX) carries AA 55 LEN CMD PAYLOAD CS frames from two ESP32
 * nodes. One node streams four VL53L0X distances + one ultrasonic
 * (CMD=0x01); the other streams three VL53L5CX 4x4 grids plus fan/online/stop
 * flags (CMD=0x02). This module owns the DMA ring, the frame parser, and the
 * two latest-value accessors. */

/* Start the USART3 RX DMA. Call once from main() after MX_USART3_UART_Init. */
void sensor_node_start_dma(void);

/* Reset parser + DMA cursor and restart the DMA. Call from the UART error
 * callback when HAL has aborted the active transfer. */
void sensor_node_restart_dma(void);

/* Pull bytes from the DMA ring and advance the frame parser. Call every
 * main-loop iteration. */
void sensor_node_service_usart3(void);

/* Rate-limited status printer -- one SENSOR line (VL53L0X + ultrasonic) and
 * one L5 line (VL53L5CX grid). Caller owns the three cursor variables so the
 * print cadence stays independent across callers. */
void sensor_node_emit_status(uint32_t now,
                             uint32_t *last_print_ms,
                             uint32_t *last_valid_frames,
                             uint32_t *last_rx_bytes);

#endif /* SENSOR_NODE_APP_H */
