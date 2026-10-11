#include "debug.h"

#include <stdarg.h>
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

void dbg_printf(const char *fmt, ...)
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
