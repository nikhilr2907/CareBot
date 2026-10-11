#include "bld515c.h"
#include <stddef.h>

/* Modbus RTU master for the BLD-515C, same RS-485 handling as dypa02.c
 * (which is proven on USART3 of this board): flush stale RX, raise DE (if a
 * manual transceiver is used), transmit, wait for TC before dropping DE, then
 * read the reply. CRC-16/Modbus, low byte first on the wire. */

#define MB_FUNC_READ    0x03u
#define MB_FUNC_WRITE   0x06u

#define TX_TIMEOUT_MS   100u
#define RX_FIRST_MS     100u   /* wait this long for the reply to START        */
#define RX_GAP_MS       8u     /* end-of-frame: >3.5 char times @9600 (~3.6ms) */

static UART_HandleTypeDef *s_huart = NULL;

/* CRC-16 / MODBUS: poly 0xA001, init 0xFFFF. Identical to dypa02.c. */
static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)buf[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

#if defined(BLD515C_DE_RE_PORT) && defined(BLD515C_DE_RE_PIN)
  #define DE_RE_SET()    HAL_GPIO_WritePin(BLD515C_DE_RE_PORT, BLD515C_DE_RE_PIN, GPIO_PIN_SET)
  #define DE_RE_CLEAR()  HAL_GPIO_WritePin(BLD515C_DE_RE_PORT, BLD515C_DE_RE_PIN, GPIO_PIN_RESET)
  #define DE_RE_WAIT_TX_DONE()  while (!__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC)) { /* spin */ }
#else
  #define DE_RE_SET()           ((void)0)
  #define DE_RE_CLEAR()         ((void)0)
  #define DE_RE_WAIT_TX_DONE()  ((void)0)
#endif

void bld515c_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    DE_RE_CLEAR();   /* default to receive */
}

/* Raw RX capture, overrun-tolerant. HAL_UART_Receive aborts the instant the
 * ORE flag latches -- which it does on a half-duplex bus that echoes our own
 * TX, because the F4 USART has no RX FIFO -- so it would deliver only the first
 * byte and quit. Here we poll SR and read DR directly: reading DR clears both
 * RXNE and ORE, so reception simply continues. Waits up to RX_FIRST_MS for the
 * first byte, then ends the frame after RX_GAP_MS of silence. Returns count. */
static int raw_recv(uint8_t *out, int max)
{
    int n = 0;
    uint32_t deadline = HAL_GetTick() + RX_FIRST_MS;
    while (n < max) {
        if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE) ||
            __HAL_UART_GET_FLAG(s_huart, UART_FLAG_ORE)) {
            out[n++] = (uint8_t)(s_huart->Instance->DR & 0xFFu);
            deadline = HAL_GetTick() + RX_GAP_MS;
        } else if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
            break;
        }
    }
    return n;
}

/* Send a CRC-complete frame, then capture the whole reply (echo + response)
 * into buf[]. Returns the number of raw bytes captured (0 on TX failure). */
static int txn_capture(const uint8_t *req, uint16_t req_len, uint8_t *buf, int bufmax)
{
    if (s_huart == NULL) return 0;

    /* Drop anything stale before we key the bus. */
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        (void)s_huart->Instance->DR;
    }

    DE_RE_SET();
    HAL_StatusTypeDef tx = HAL_UART_Transmit(s_huart, (uint8_t *)req, req_len, TX_TIMEOUT_MS);
    DE_RE_WAIT_TX_DONE();
    DE_RE_CLEAR();
    if (tx != HAL_OK) return 0;

    return raw_recv(buf, bufmax);
}

bool bld515c_write_reg(uint8_t addr, uint16_t reg, uint16_t val)
{
    uint8_t req[8];
    req[0] = addr;
    req[1] = MB_FUNC_WRITE;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFFu);
    req[4] = (uint8_t)(val >> 8);
    req[5] = (uint8_t)(val & 0xFFu);
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);   /* CRC low byte first on the wire */
    req[7] = (uint8_t)(crc >> 8);

    /* The function-0x06 reply is the request echoed back verbatim. On a
     * half-duplex auto-converter our own TX may also loop back, so scan the
     * capture for an 8-byte frame that matches our request (with valid CRC)
     * rather than assuming it lands at offset 0. */
    uint8_t buf[24];
    int n = txn_capture(req, 8, buf, sizeof buf);
    for (int off = 0; off + 8 <= n; ++off) {
        bool match = true;
        for (int i = 0; i < 8; ++i) {
            if (buf[off + i] != req[i]) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

bool bld515c_read_regs(uint8_t addr, uint16_t reg, uint16_t *out, uint8_t count)
{
    if (out == NULL || count == 0 || count > 8) return false;

    uint8_t req[8];
    req[0] = addr;
    req[1] = MB_FUNC_READ;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFFu);
    req[4] = 0x00;
    req[5] = count;
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);
    req[7] = (uint8_t)(crc >> 8);

    /* Capture echo + response, then scan for a valid read reply:
     *   addr, 0x03, bytecount(=2*count), data..., crc_lo, crc_hi
     * Scanning past a leading echo of our own request (which begins
     * addr,0x03,reg_hi,... and so won't match bytecount==2*count) makes this
     * robust on half-duplex converters. */
    uint8_t buf[40];
    int n = txn_capture(req, 8, buf, sizeof buf);
    int want = 5 + 2 * count;
    for (int off = 0; off + want <= n; ++off) {
        if (buf[off]     != addr)              continue;
        if (buf[off + 1] != MB_FUNC_READ)      continue;
        if (buf[off + 2] != (uint8_t)(2 * count)) continue;

        uint16_t got  = (uint16_t)buf[off + want - 2]
                      | ((uint16_t)buf[off + want - 1] << 8);
        if (crc16_modbus(&buf[off], (size_t)(want - 2)) != got) continue;

        for (uint8_t i = 0; i < count; ++i) {
            out[i] = ((uint16_t)buf[off + 3 + i * 2] << 8) | buf[off + 4 + i * 2];
        }
        return true;
    }
    return false;
}

int bld515c_probe(uint8_t addr, uint16_t reg, uint8_t *out, int max, uint32_t per_byte_ms)
{
    (void)per_byte_ms;   /* timing now fixed by RX_FIRST_MS / RX_GAP_MS */
    if (s_huart == NULL || out == NULL || max <= 0) return 0;

    uint8_t req[8];
    req[0] = addr;
    req[1] = MB_FUNC_READ;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFFu);
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);
    req[7] = (uint8_t)(crc >> 8);

    return txn_capture(req, 8, out, max);
}

bool bld515c_selftest_loopback(void)
{
    if (s_huart == NULL) return false;

    static const uint8_t pattern[4] = { 0x55, 0xAA, 0x01, 0xFE };
    int matched = 0;

    /* One byte at a time: the F4 USART has no RX FIFO, so transmitting a burst
     * without reading concurrently would overrun. TX-then-RX per byte avoids
     * that and is plenty fast for a one-shot self-test. */
    for (int i = 0; i < 4; ++i) {
        while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
            (void)s_huart->Instance->DR;
        }
        DE_RE_SET();
        (void)HAL_UART_Transmit(s_huart, (uint8_t *)&pattern[i], 1, TX_TIMEOUT_MS);
        DE_RE_WAIT_TX_DONE();
        DE_RE_CLEAR();

        uint8_t got = 0;
        if (HAL_UART_Receive(s_huart, &got, 1, 20) == HAL_OK && got == pattern[i]) {
            matched++;
        }
    }
    return matched == 4;
}

bool bld515c_set_speed_rpm(uint8_t addr, uint16_t rpm)
{
    return bld515c_write_reg(addr, BLD515C_REG_SPEED, rpm);
}

bool bld515c_run(uint8_t addr, bool reverse)
{
    uint16_t ctl = BLD515C_MODE_DEFAULT | BLD515C_CTL_START;
    if (reverse) ctl |= BLD515C_CTL_REVERSE;
    return bld515c_write_reg(addr, BLD515C_REG_CONTROL, ctl);
}

bool bld515c_stop(uint8_t addr)
{
    return bld515c_write_reg(addr, BLD515C_REG_CONTROL, BLD515C_MODE_DEFAULT);
}

bool bld515c_brake(uint8_t addr)
{
    return bld515c_write_reg(addr, BLD515C_REG_CONTROL,
                             BLD515C_MODE_DEFAULT | BLD515C_CTL_BRAKE);
}
