#ifndef BLD515C_H
#define BLD515C_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   /* UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * BLD-515C digital brushless DC motor driver (StepperOnline / OMC).
 * Modbus RTU over RS-485.  Wiring on this board:
 *
 *   USART1   TX=PA9, RX=PA10   (see HAL_UART_MspInit / MX_USART1_UART_Init)
 *   RS-485   via an external TTL<->RS485 converter on USART1.
 *
 * Data format: 8 data bits, 1 stop bit, NO parity (8N1).
 * Default slave/site address: 1 (user range 1..250, 0 = broadcast).
 *
 * BAUD: USART1 is configured for 9600 (overridden in MX_USART1_UART_Init's
 * USER CODE block). The BLD-515C's baud is a stored parameter; if the driver
 * doesn't answer, confirm its baud is also 9600 (factory default) or change
 * huart1 to match whatever it's set to. See the bring-up notes in main.c.
 *
 * RS-485 direction control: by default this driver assumes the converter
 * auto-toggles direction from TX activity (same assumption as the DYP-A02
 * driver on USART3). If your USART1 converter has a manual DE/RE pin, wire it
 * to a GPIO output and uncomment the two #defines below -- the driver will
 * then drive it high during TX and drop it after the TX shift register
 * drains. PB6 is already configured as a GPIO output by MX_GPIO_Init, so it's
 * the natural choice if you need manual direction control.
 * ===================================================================== */
/* #define BLD515C_DE_RE_PORT  GPIOB
 * #define BLD515C_DE_RE_PIN   GPIO_PIN_6 */

#define BLD515C_DEFAULT_ADDR   0x01u

/* --- Register map (Modbus holding registers, function 0x03 read / 0x06 write)
 * Addresses below are CRC-verified against the worked frame examples in the
 * BLD-515C manual:
 *   "01 06 81 06 07 00 43 C7"  -> 0x8106 = 0x0700  (internal ctrl, stopped)
 *   "01 06 81 06 07 01 82 07"  -> 0x8106 = 0x0701  (internal ctrl + start)
 *   "01 06 81 10 05 DC A2 FA"  -> 0x8110 = 0x05DC  (target speed = 1500 rpm)
 */
#define BLD515C_REG_DEVTYPE    0x8000u   /* device type (R/O), reads 0x0192  */
#define BLD515C_REG_CONTROL    0x8106u   /* control word (R/W)               */
#define BLD515C_REG_SPEED      0x8110u   /* target speed in RPM (R/W)        */
#define BLD515C_REG_SAVE       0x81FFu   /* write 0xFFFF here to persist cfg */

/* Control-word (0x8106) bit fields.
 *   Low byte  -- motion command:                                            */
#define BLD515C_CTL_START      0x0001u   /* bit0: 1 = run, 0 = stop          */
#define BLD515C_CTL_REVERSE    0x0002u   /* bit1: 1 = reverse/CCW, 0 = fwd   */
#define BLD515C_CTL_BRAKE      0x0004u   /* bit2: 1 = brake                  */
/*   High byte -- operating mode:                                            */
#define BLD515C_CTL_INTERNAL   0x0100u   /* bit8:  1 = internal (comms) ctrl */
#define BLD515C_CTL_SPD_COMMS  0x0200u   /* bit9:  speed source = comms      */
#define BLD515C_CTL_SENSORED   0x0400u   /* bit10: 1 = Hall sensored         */

/* Standard mode bits for driving over RS-485 with a Hall-sensored motor.
 * Matches the manual's "set internal control" example (high byte 0x07). If
 * your motor is sensorless, drop BLD515C_CTL_SENSORED. */
#define BLD515C_MODE_DEFAULT   (BLD515C_CTL_INTERNAL | BLD515C_CTL_SPD_COMMS \
                                | BLD515C_CTL_SENSORED)

/* Bind the driver to a UART handle (call once after MX_USART1_UART_Init). */
void bld515c_init(UART_HandleTypeDef *huart);

/* Low-level Modbus primitives. Both return true on a valid reply, false on
 * timeout / framing / CRC error.
 *   write_reg: function 0x06, verifies the driver's echo.
 *   read_regs: function 0x03, reads `count` consecutive 16-bit registers into
 *              out[] (big-endian on the wire -> native uint16). count <= 8. */
bool bld515c_write_reg(uint8_t addr, uint16_t reg, uint16_t val);
bool bld515c_read_regs(uint8_t addr, uint16_t reg, uint16_t *out, uint8_t count);

/* Bring-up diagnostic: send a 1-register read of `reg`, then capture whatever
 * comes back (up to `max` bytes, `per_byte_ms` gap timeout). Returns the raw
 * byte count -- 0 means the bus was silent. Unlike read_regs() this never
 * cares about frame length, so it surfaces a silent bus, line noise (A/B
 * swapped), or a Modbus exception reply rather than lumping them together. */
int bld515c_probe(uint8_t addr, uint16_t reg, uint8_t *out, int max, uint32_t per_byte_ms);

/* UART loopback self-test. Transmits a few bytes on USART1 TX and checks they
 * come back on USART1 RX. With a jumper wire shorting PA9<->PA10 (the TTL side,
 * BEFORE the RS-485 converter), success proves the USART1 peripheral + pin AF
 * config + baud are all working, isolating the fault to the converter/bus/
 * driver. Returns true if every byte echoed back correctly. */
bool bld515c_selftest_loopback(void);

/* High-level helpers. Each is a single 0x8106 / 0x8110 register write.
 * run()/stop()/brake() send the full control word (mode bits included) so a
 * single call both configures internal-comms mode AND issues the command. */
bool bld515c_set_speed_rpm(uint8_t addr, uint16_t rpm);
bool bld515c_run(uint8_t addr, bool reverse);   /* 0x8106 = MODE | START[|REV] */
bool bld515c_stop(uint8_t addr);                /* 0x8106 = MODE (start bit 0) */
bool bld515c_brake(uint8_t addr);               /* 0x8106 = MODE | BRAKE       */

#ifdef __cplusplus
}
#endif

#endif /* BLD515C_H */
