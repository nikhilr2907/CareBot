#include "bno055.h"
#include "main.h"

/* I2C handle lives in main.c (CubeMX-generated). */
extern I2C_HandleTypeDef hi2c2;

/* 7-bit address 0x28, shifted left for the HAL's 8-bit convention.
 * If the BNO055 module has COM3/ADR pulled HIGH, change to (0x29 << 1). */
#define BNO055_I2C_ADDR       (0x28u << 1)
#define BNO055_CHIP_ID_VALUE  0xA0u

/* Register addresses (page 0). */
#define REG_CHIP_ID           0x00u
#define REG_PAGE_ID           0x07u
#define REG_EUL_HEADING_LSB   0x1Au
#define REG_CALIB_STAT        0x35u
#define REG_SYS_STATUS        0x39u
#define REG_OPR_MODE          0x3Du
#define REG_PWR_MODE          0x3Eu
#define REG_SYS_TRIGGER       0x3Fu

#define OPR_MODE_CONFIG       0x00u
#define OPR_MODE_NDOF         0x0Cu
#define BNO055_PWR_MODE_NORMAL 0x00u

/* Reset pin -- configured as GPIO_Output by MX_GPIO_Init() in main.c and
 * driven LOW at boot, so the chip starts in reset. bno055_init() brings
 * it out by driving HIGH and waiting for the boot timer. */
#define BNO055_RST_PORT       GPIOC
#define BNO055_RST_PIN        GPIO_PIN_3

/* All HAL_I2C calls use a 50 ms timeout. At 100 kHz, the longest burst we
 * do is 6 bytes (Euler) which takes < 1 ms, so 50 ms is huge headroom that
 * still keeps the main loop responsive if the bus is misbehaving. */
#define I2C_TIMEOUT_MS        50u

static HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c2, BNO055_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2, BNO055_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS);
}

bool bno055_init(void)
{
    /* Hold reset low for >5 ms (10 used here for safety), release, then
     * wait 650 ms for the chip to boot. The datasheet calls this the
     * power-on / reset time -- I2C transactions before this will NACK. */
    HAL_GPIO_WritePin(BNO055_RST_PORT, BNO055_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BNO055_RST_PORT, BNO055_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(700);

    /* Sanity-check the chip is alive and is actually a BNO055. */
    uint8_t id = 0;
    if (read_burst(REG_CHIP_ID, &id, 1) != HAL_OK) return false;
    if (id != BNO055_CHIP_ID_VALUE)                return false;

    /* Force CONFIG mode before reconfiguring anything; ignore failures here
     * since the chip may already be in CONFIG and the write is harmless. */
    if (write_reg(REG_PAGE_ID, 0)                  != HAL_OK) return false;
    if (write_reg(REG_OPR_MODE, OPR_MODE_CONFIG)   != HAL_OK) return false;
    HAL_Delay(25);

    /* Normal power, internal oscillator (SYS_TRIGGER bit 7 = 0). */
    if (write_reg(REG_PWR_MODE, BNO055_PWR_MODE_NORMAL) != HAL_OK) return false;
    if (write_reg(REG_SYS_TRIGGER, 0x00)           != HAL_OK) return false;

    /* Engage NDOF -- 9-DOF fusion with continuous self-calibration. The
     * mode switch takes ~7 ms; wait 20 to be conservative. */
    if (write_reg(REG_OPR_MODE, OPR_MODE_NDOF)     != HAL_OK) return false;
    HAL_Delay(20);

    return true;
}

bool bno055_read_euler(bno055_euler_t *out)
{
    uint8_t buf[6];
    if (read_burst(REG_EUL_HEADING_LSB, buf, sizeof buf) != HAL_OK) return false;

    /* All three values are little-endian signed 16-bit. Resolution is
     * 1 LSB = 1/16 degree (datasheet table 3-29). */
    int16_t h = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t r = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t p = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    out->heading_deg = (float)h * (1.0f / 16.0f);
    out->roll_deg    = (float)r * (1.0f / 16.0f);
    out->pitch_deg   = (float)p * (1.0f / 16.0f);
    return true;
}

bool bno055_read_calib(bno055_calib_t *out)
{
    uint8_t v;
    if (read_burst(REG_CALIB_STAT, &v, 1) != HAL_OK) return false;
    out->sys   = (uint8_t)((v >> 6) & 0x03u);
    out->gyro  = (uint8_t)((v >> 4) & 0x03u);
    out->accel = (uint8_t)((v >> 2) & 0x03u);
    out->mag   = (uint8_t)((v >> 0) & 0x03u);
    return true;
}
