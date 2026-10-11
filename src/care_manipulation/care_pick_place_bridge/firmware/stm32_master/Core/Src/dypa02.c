#include "dypa02.h"
#include <stddef.h>

#define MB_FUNC_READ   0x03u

/* Single-register read request is 8 bytes; response for 1 register is 7 bytes
 * (addr + func + bytecount=2 + 2 data + 2 CRC). Plenty of headroom in the
 * timeouts -- sensor's worst-case real-time response is ~100 ms; we wait up
 * to 200 ms. Polling at 5 Hz from main leaves another ~100 ms slack. */
#define REQ_LEN        8u
#define RSP_LEN        7u
#define TX_TIMEOUT_MS  100u
#define RX_FIRST_MS    200u   /* wait this long for the reply to START        */
#define RX_GAP_MS      8u     /* end-of-frame: >3.5 char times @9600 (~3.6ms) */
/* DYP-A02 UART "controlled" mode emits one [FF][H][L][SUM] frame per trigger
 * byte. 0x55 is the documented trigger. Harmless if the sensor is actually in
 * continuous auto-output mode (it just ignores RX). */
#define DYPA02_TRIGGER_BYTE  0x55u

static UART_HandleTypeDef *s_huart = NULL;

/* CRC-16 / MODBUS: poly 0xA001 (right-shifting form of 0x8005), init 0xFFFF.
 * Verified against the PDF examples -- "01 03 01 00 00 01" -> 0xF685 -> CRC
 * bytes 0x85 0xF6 (low byte first on the wire). */
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

#if defined(DYPA02_DE_RE_PORT) && defined(DYPA02_DE_RE_PIN)
  #define DE_RE_SET()    HAL_GPIO_WritePin(DYPA02_DE_RE_PORT, DYPA02_DE_RE_PIN, GPIO_PIN_SET)
  #define DE_RE_CLEAR()  HAL_GPIO_WritePin(DYPA02_DE_RE_PORT, DYPA02_DE_RE_PIN, GPIO_PIN_RESET)
  /* After HAL_UART_Transmit returns, the last byte may still be sitting in
   * the TX shift register. Drop DE only after TC (transmission complete)
   * goes high, otherwise the trailing byte gets garbled on the RS-485 bus. */
  #define DE_RE_WAIT_TX_DONE()  while (!__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC)) { /* spin */ }
#else
  #define DE_RE_SET()           ((void)0)
  #define DE_RE_CLEAR()         ((void)0)
  #define DE_RE_WAIT_TX_DONE()  ((void)0)
#endif

void dypa02_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    DE_RE_CLEAR();   /* default to receive */
}

/* Overrun-tolerant raw RX capture. HAL_UART_Receive aborts the instant the
 * ORE flag latches -- which it does on a half-duplex converter that echoes our
 * own TX (the F4 USART has no RX FIFO) -- so it would drop the sensor's reply
 * after one byte. We poll SR and read DR directly (reading DR clears RXNE+ORE)
 * so reception continues. Waits RX_FIRST_MS for the first byte, then ends the
 * frame after RX_GAP_MS of silence. Returns the raw byte count. */
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

/* Build the 8-byte "read 1 register" request for reg into req[8]. */
static void build_read_req(uint8_t *req, uint8_t slave, uint16_t reg)
{
    req[0] = slave;
    req[1] = MB_FUNC_READ;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFFu);
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);     /* CRC low byte first on the wire */
    req[7] = (uint8_t)(crc >> 8);
}

static bool read_one_register(uint8_t slave, uint16_t reg, uint16_t *out)
{
    if (s_huart == NULL) return false;

    uint8_t req[REQ_LEN];
    build_read_req(req, slave, reg);

    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        (void)s_huart->Instance->DR;
    }

    DE_RE_SET();
    HAL_StatusTypeDef tx = HAL_UART_Transmit(s_huart, req, REQ_LEN, TX_TIMEOUT_MS);
    DE_RE_WAIT_TX_DONE();
    DE_RE_CLEAR();
    if (tx != HAL_OK) return false;

    /* Capture echo + reply, then scan for a valid 7-byte read response
     * (slave, 0x03, 0x02, hi, lo, crc_lo, crc_hi), skipping any leading echo
     * of our own request that a half-duplex converter loops back. */
    uint8_t buf[24];
    int n = raw_recv(buf, sizeof buf);
    for (int off = 0; off + 7 <= n; ++off) {
        if (buf[off]     != slave)        continue;
        if (buf[off + 1] != MB_FUNC_READ) continue;
        if (buf[off + 2] != 0x02)         continue;   /* 2 data bytes */
        uint16_t got = (uint16_t)buf[off + 5] | ((uint16_t)buf[off + 6] << 8);
        if (crc16_modbus(&buf[off], 5) != got) continue;
        *out = ((uint16_t)buf[off + 3] << 8) | buf[off + 4];   /* big-endian */
        return true;
    }
    return false;
}

bool dypa02_read_uart_auto(uint16_t *mm)
{
    if (s_huart == NULL || mm == NULL) return false;

    /* Drop partial/stale bytes once, then capture. Two attempts: if the first
     * capture started mid-frame, the second begins in the inter-frame gap and
     * lands a whole frame. Each raw_recv returns within ~one frame period. */
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        (void)s_huart->Instance->DR;
    }

    uint8_t buf[32];
    for (int attempt = 0; attempt < 2; ++attempt) {
        int n = raw_recv(buf, sizeof buf);
        /* Scan for a valid 4-byte frame: 0xFF, H, L, SUM where
         * SUM = (0xFF + H + L) & 0xFF. Distance = (H<<8 | L) in mm. */
        for (int i = 0; i + 3 < n; ++i) {
            if (buf[i] != 0xFFu) continue;
            uint8_t sum = (uint8_t)(buf[i] + buf[i + 1] + buf[i + 2]);
            if (sum != buf[i + 3]) continue;
            *mm = ((uint16_t)buf[i + 1] << 8) | buf[i + 2];
            return true;
        }
    }
    return false;
}

/* ---- Background streaming via circular DMA --------------------------------*/
static UART_HandleTypeDef *s_stream     = NULL;
static uint8_t            *s_ring       = NULL;
static uint16_t            s_ring_sz    = 0;
static uint16_t            s_rd         = 0;       /* parser read cursor       */
/* 4-byte frame parser: 0=wait header, 1=have FF, 2=have H, 3=have L (need SUM) */
static uint8_t             s_fidx       = 0;
static uint8_t             s_fh, s_fl;
static uint16_t            s_latest_mm  = 0;
static uint32_t            s_latest_ms  = 0;
static bool                s_have       = false;

static void stream_feed(uint8_t b)
{
    switch (s_fidx) {
    case 0: if (b == 0xFFu) s_fidx = 1; break;          /* sync on header     */
    case 1: s_fh = b;       s_fidx = 2; break;
    case 2: s_fl = b;       s_fidx = 3; break;
    default: {                                          /* case 3: SUM byte   */
        uint8_t sum = (uint8_t)(0xFFu + s_fh + s_fl);
        if (sum == b) {
            s_latest_mm = ((uint16_t)s_fh << 8) | s_fl;
            s_latest_ms = HAL_GetTick();
            s_have = true;
            s_fidx = 0;
        } else {
            /* Bad checksum -> resync; if this byte is itself a header, keep it. */
            s_fidx = (b == 0xFFu) ? 1 : 0;
        }
        break;
    }
    }
}

void dypa02_start_stream(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    s_stream  = huart;
    s_ring    = buf;
    s_ring_sz = size;
    s_rd      = 0;
    s_fidx    = 0;
    s_have    = false;
    if (huart && buf && size) {
        HAL_UART_Receive_DMA(huart, buf, size);
    }
}

void dypa02_restart_stream(void)
{
    if (s_stream && s_ring && s_ring_sz) {
        s_rd   = 0;
        s_fidx = 0;
        HAL_UART_Receive_DMA(s_stream, s_ring, s_ring_sz);
    }
}

void dypa02_service(void)
{
    if (s_stream == NULL || s_ring == NULL) return;
    DMA_HandleTypeDef *hdma = s_stream->hdmarx;
    if (hdma == NULL) return;

    /* DMA write cursor = size - remaining-transfer-count (circular). */
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(hdma);
    uint16_t wr = (uint16_t)(s_ring_sz - remaining);

    while (s_rd != wr) {
        stream_feed(s_ring[s_rd]);
        if (++s_rd >= s_ring_sz) s_rd = 0;
    }
}

bool dypa02_get_distance(uint16_t *mm, uint32_t *age_ms)
{
    if (!s_have) return false;
    if (mm)     *mm     = s_latest_mm;
    if (age_ms) *age_ms = HAL_GetTick() - s_latest_ms;
    return true;
}

void dypa02_stream_reset(void)
{
    if (s_stream && s_stream->hdmarx) {
        /* Jump the parser cursor to the DMA's current write position so any
         * bytes buffered before now are skipped. */
        uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(s_stream->hdmarx);
        s_rd = (uint16_t)(s_ring_sz - remaining);
    }
    s_fidx = 0;
    s_have = false;
}

void dypa02_trigger(void)
{
    if (s_stream == NULL) return;
    /* Poke the sensor so it emits one frame (controlled mode). TX runs fine
     * alongside the circular DMA RX -- they use independent state. Any echo of
     * this byte on a half-duplex bus is ignored by the frame parser. */
    uint8_t cmd = DYPA02_TRIGGER_BYTE;
    HAL_UART_Transmit(s_stream, &cmd, 1, 10);
}

bool dypa02_selftest_loopback(void)
{
    if (s_huart == NULL) return false;

    static const uint8_t pattern[4] = { 0x55, 0xAA, 0x01, 0xFE };
    int matched = 0;

    /* One byte at a time (the F4 USART has no RX FIFO). With PC10<->PC11
     * jumpered (sensor unplugged) every byte must echo back -- proving USART3
     * TX, RX, pin AF and baud all work, isolating any fault to the converter
     * or the sensor side. */
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

int dypa02_probe(uint8_t slave, uint16_t reg, uint8_t *out, int max)
{
    if (s_huart == NULL || out == NULL || max <= 0) return 0;

    uint8_t req[REQ_LEN];
    build_read_req(req, slave, reg);

    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        (void)s_huart->Instance->DR;
    }

    DE_RE_SET();
    HAL_UART_Transmit(s_huart, req, REQ_LEN, TX_TIMEOUT_MS);
    DE_RE_WAIT_TX_DONE();
    DE_RE_CLEAR();

    return raw_recv(out, max);
}

bool dypa02_read_distance_processed(uint8_t slave_addr, uint16_t *mm)
{
    return read_one_register(slave_addr, 0x0100u, mm);
}

bool dypa02_read_distance_realtime(uint8_t slave_addr, uint16_t *mm)
{
    return read_one_register(slave_addr, 0x0101u, mm);
}

bool dypa02_read_temperature_01C(uint8_t slave_addr, uint16_t *temp_01C)
{
    return read_one_register(slave_addr, 0x0102u, temp_01C);
}
