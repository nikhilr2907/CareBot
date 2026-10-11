#ifndef DYPA02_H
#define DYPA02_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   /* UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* DYP-A02 ultrasonic range sensor, Modbus RTU over RS-485.
 * UART must already be initialized at 9600 baud, 8N1, no parity.
 *
 * Holding registers (read with function 0x03):
 *   0x0100  processed distance, mm (uint16, high byte first) -- 500 ms response
 *   0x0101  real-time distance, mm (uint16, high byte first) -- ~100 ms response
 *   0x0102  temperature in 0.1 C units (uint16)              -- ~100 ms response
 *
 * Sensor's default slave address is 0x01. Bus can be multi-drop if you change
 * each sensor's slave address (register 0x0200, function 0x06).
 *
 * RS-485 direction control: this driver assumes the transceiver auto-toggles
 * direction based on TX activity (common for cheap "TTL-to-RS485" modules
 * marked CH340/MAX13487/etc.). If your converter has a manual DE/RE pin, set
 * DYPA02_DE_RE_PORT/PIN below to a GPIO output and the driver will drive it
 * high during transmit and low after the last byte clears the TX shift reg. */
/* #define DYPA02_DE_RE_PORT  GPIOX
 * #define DYPA02_DE_RE_PIN   GPIO_PIN_n */

/* Bind the driver to a UART handle. Call once during init, after
 * MX_USARTx_UART_Init() has run. The UART must be at 9600 baud. */
void dypa02_init(UART_HandleTypeDef *huart);

/* DYP-A02 *UART auto-output* mode (the variant on this build). The sensor
 * continuously streams 4-byte frames: [0xFF][DistHigh][DistLow][SUM] where
 * SUM = (0xFF + DistHigh + DistLow) & 0xFF, distance in mm. This listens for
 * one valid frame and returns the distance in *mm. No address/query needed.
 * Returns true on a checksum-valid frame. NOTE: the Modbus functions below are
 * for the RS-485 Modbus variant and don't apply to an auto-output sensor. */
bool dypa02_read_uart_auto(uint16_t *mm);

/* --- Background streaming (non-blocking) ------------------------------------
 * Receive the auto-output stream continuously via circular DMA and parse it in
 * the background, so the main loop never blocks waiting for a frame.
 *   dypa02_start_stream()  : start circular DMA RX into a caller-owned buffer.
 *   dypa02_service()       : drain new DMA bytes through the frame parser; call
 *                            often from the main loop (cheap, non-blocking).
 *   dypa02_get_distance()  : latest checksum-valid distance (mm) and its age in
 *                            ms; returns false until the first frame arrives.
 *   dypa02_restart_stream(): re-arm DMA after a UART error -- call this from
 *                            HAL_UART_ErrorCallback for USART3. */
void dypa02_start_stream(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
void dypa02_service(void);
bool dypa02_get_distance(uint16_t *mm, uint32_t *age_ms);
void dypa02_restart_stream(void);

/* Send the trigger byte so a "controlled mode" sensor emits one frame. Call it
 * periodically (e.g. each poll). No-op-safe for continuous auto-output sensors
 * (the byte is ignored). The reply is captured by the background DMA + parser. */
void dypa02_trigger(void);

/* Discard everything currently buffered and the last-parsed reading, so the
 * next dypa02_get_distance() only succeeds on a frame received AFTER this call.
 * Use it right after switching to a different sensor so you don't read the
 * previous sensor's stale frame. */
void dypa02_stream_reset(void);

/* Read the "processed" distance register (0x0100). Slower response (500 ms
 * on the sensor side) but the value is filtered/averaged. Returns true on
 * success; *mm holds the distance in millimetres. */
bool dypa02_read_distance_processed(uint8_t slave_addr, uint16_t *mm);

/* Read the "real-time" distance register (0x0101). ~100 ms sensor response,
 * suitable for ~5-10 Hz polling. Returns true on success. */
bool dypa02_read_distance_realtime(uint8_t slave_addr, uint16_t *mm);

/* Read the temperature register (0x0102), 0.1 C resolution. */
bool dypa02_read_temperature_01C(uint8_t slave_addr, uint16_t *temp_01C);

/* Bring-up diagnostic: send a 1-register read of `reg` and capture whatever
 * comes back (up to `max` raw bytes) regardless of framing. Returns the byte
 * count -- 0 means the bus was silent (no reply reaching USART3 RX). Use it to
 * tell a dead bus / swapped A-B / wrong baud apart from a good reply. */
int dypa02_probe(uint8_t slave_addr, uint16_t reg, uint8_t *out, int max);

/* USART3 loopback self-test. With a jumper shorting PC10<->PC11 (sensor/
 * converter unplugged), returns true iff every transmitted byte echoes back --
 * proving the USART3 peripheral, pin AF and baud are all healthy. */
bool dypa02_selftest_loopback(void);

#ifdef __cplusplus
}
#endif

#endif /* DYPA02_H */
