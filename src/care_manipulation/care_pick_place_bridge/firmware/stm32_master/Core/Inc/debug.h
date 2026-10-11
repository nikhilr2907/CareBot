#ifndef DEBUG_H
#define DEBUG_H

/* Lightweight printf-over-USART2 used by every module for status prints.
 * Blocks for up to 100 ms while the TX line drains -- fine for boot banners
 * and periodic logs, DO NOT call from ISRs. */
void dbg_printf(const char *fmt, ...);

#endif /* DEBUG_H */
