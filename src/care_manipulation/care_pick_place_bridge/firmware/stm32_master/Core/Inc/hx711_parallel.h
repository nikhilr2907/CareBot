#ifndef HX711_PARALLEL_H
#define HX711_PARALLEL_H

#include <stdbool.h>
#include <stdint.h>

#define HX711_PARALLEL_CHANNEL_COUNT 4u
#define HX711_PARALLEL_ALL_MASK      ((1u << HX711_PARALLEL_CHANNEL_COUNT) - 1u)

/* Kept as a public debug structure so the values can also be inspected over
 * STLink/SWD if the virtual serial port is unavailable. */
typedef struct {
    uint32_t magic;
    int32_t raw[HX711_PARALLEL_CHANNEL_COUNT];
    int32_t zero[HX711_PARALLEL_CHANNEL_COUNT];
    int32_t min_delta[HX711_PARALLEL_CHANNEL_COUNT];
    int32_t max_delta[HX711_PARALLEL_CHANNEL_COUNT];
    uint32_t sample_count;
    uint32_t last_update_ms;
    uint32_t timeout_count;
    uint8_t ready_mask;
    uint8_t last_timeout_ready_mask;
    uint8_t valid_mask;
    uint8_t reserved;
} hx711_parallel_state_t;

extern volatile hx711_parallel_state_t g_hx711_parallel_state;

void hx711_parallel_init(void);
bool hx711_parallel_service(uint32_t now_ms);
bool hx711_parallel_tare(void);
void hx711_parallel_reset_span(void);
void hx711_parallel_get_state(hx711_parallel_state_t *state);

#endif /* HX711_PARALLEL_H */
