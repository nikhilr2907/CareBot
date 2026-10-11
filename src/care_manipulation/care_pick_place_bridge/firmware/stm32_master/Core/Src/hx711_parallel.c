#include "hx711_parallel.h"

#include "stm32f4xx_hal.h"

/* Central pin map for the four-HX711 harness.
 *
 * All boards share SCK. DOUT remains separate so a missing board can be
 * identified from the readiness mask. Channel order is electronic only until
 * the four physical load-cell positions have been mapped. */
#define HX711_SCK_PORT GPIOC
#define HX711_SCK_PIN  GPIO_PIN_0

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} hx711_dout_pin_t;

static const hx711_dout_pin_t hx711_dout[HX711_PARALLEL_CHANNEL_COUNT] = {
    { GPIOB, GPIO_PIN_0 }, /* channel 0: Nucleo A3 */
    { GPIOB, GPIO_PIN_5 }, /* channel 1: Nucleo D4 */
    { GPIOC, GPIO_PIN_7 }, /* channel 2: Nucleo D9 */
    { GPIOA, GPIO_PIN_8 }, /* channel 3: Nucleo D7 */
};

#define HX711_STATE_MAGIC        0x48583731u /* "HX71" */
#define HX711_READY_TIMEOUT_MS   350u
#define HX711_STALE_MS           500u

volatile hx711_parallel_state_t g_hx711_parallel_state;

static uint32_t wait_started_ms;

static void hx711_short_delay(void)
{
    /* At the configured 84 MHz core clock this is comfortably above the
     * HX711 0.2 us clock minimum while remaining far below power-down time. */
    for (volatile uint32_t i = 0u; i < 16u; ++i) {
        __NOP();
    }
}

static uint8_t hx711_read_ready_mask(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0u; i < HX711_PARALLEL_CHANNEL_COUNT; ++i) {
        if ((hx711_dout[i].port->IDR & hx711_dout[i].pin) == 0u) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

static void hx711_read_parallel(int32_t values[HX711_PARALLEL_CHANNEL_COUNT])
{
    uint32_t bits[HX711_PARALLEL_CHANNEL_COUNT] = {0u, 0u, 0u, 0u};
    uint32_t primask = __get_PRIMASK();

    /* An interrupt during SCK high could accidentally exceed the HX711's
     * power-down threshold. The protected section is only about 30 us. */
    __disable_irq();

    for (uint8_t bit = 0u; bit < 24u; ++bit) {
        HX711_SCK_PORT->BSRR = HX711_SCK_PIN;
        hx711_short_delay();

        for (uint8_t channel = 0u;
             channel < HX711_PARALLEL_CHANNEL_COUNT;
             ++channel) {
            bits[channel] <<= 1;
            if ((hx711_dout[channel].port->IDR & hx711_dout[channel].pin) != 0u) {
                bits[channel] |= 1u;
            }
        }

        HX711_SCK_PORT->BSRR = (uint32_t)HX711_SCK_PIN << 16u;
        hx711_short_delay();
    }

    /* Pulse 25 selects Channel A, gain 128, for the next conversion. */
    HX711_SCK_PORT->BSRR = HX711_SCK_PIN;
    hx711_short_delay();
    HX711_SCK_PORT->BSRR = (uint32_t)HX711_SCK_PIN << 16u;
    hx711_short_delay();

    if (primask == 0u) {
        __enable_irq();
    }

    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        if ((bits[channel] & 0x00800000u) != 0u) {
            bits[channel] |= 0xFF000000u;
        }
        values[channel] = (int32_t)bits[channel];
    }
}

void hx711_parallel_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Drive SCK low before making it an output so no spurious edge is sent. */
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    gpio.Pin = HX711_SCK_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HX711_SCK_PORT, &gpio);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        gpio.Pin = hx711_dout[channel].pin;
        HAL_GPIO_Init(hx711_dout[channel].port, &gpio);
    }

    g_hx711_parallel_state.magic = HX711_STATE_MAGIC;
    g_hx711_parallel_state.sample_count = 0u;
    g_hx711_parallel_state.last_update_ms = 0u;
    g_hx711_parallel_state.timeout_count = 0u;
    g_hx711_parallel_state.ready_mask = 0u;
    g_hx711_parallel_state.last_timeout_ready_mask = 0u;
    g_hx711_parallel_state.valid_mask = 0u;
    g_hx711_parallel_state.reserved = 0u;
    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        g_hx711_parallel_state.raw[channel] = 0;
        g_hx711_parallel_state.zero[channel] = 0;
        g_hx711_parallel_state.min_delta[channel] = 0;
        g_hx711_parallel_state.max_delta[channel] = 0;
    }

    wait_started_ms = HAL_GetTick();
}

bool hx711_parallel_service(uint32_t now_ms)
{
    uint8_t ready_mask = hx711_read_ready_mask();
    g_hx711_parallel_state.ready_mask = ready_mask;

    if (ready_mask == HX711_PARALLEL_ALL_MASK) {
        int32_t values[HX711_PARALLEL_CHANNEL_COUNT];
        hx711_read_parallel(values);

        bool first_sample = (g_hx711_parallel_state.sample_count == 0u);
        for (uint8_t channel = 0u;
             channel < HX711_PARALLEL_CHANNEL_COUNT;
             ++channel) {
            int32_t delta = values[channel] - g_hx711_parallel_state.zero[channel];
            g_hx711_parallel_state.raw[channel] = values[channel];
            if (first_sample) {
                g_hx711_parallel_state.min_delta[channel] = delta;
                g_hx711_parallel_state.max_delta[channel] = delta;
            } else {
                if (delta < g_hx711_parallel_state.min_delta[channel]) {
                    g_hx711_parallel_state.min_delta[channel] = delta;
                }
                if (delta > g_hx711_parallel_state.max_delta[channel]) {
                    g_hx711_parallel_state.max_delta[channel] = delta;
                }
            }
        }

        g_hx711_parallel_state.sample_count++;
        g_hx711_parallel_state.last_update_ms = now_ms;
        g_hx711_parallel_state.valid_mask = HX711_PARALLEL_ALL_MASK;
        wait_started_ms = now_ms;
        return true;
    }

    if ((now_ms - wait_started_ms) >= HX711_READY_TIMEOUT_MS) {
        g_hx711_parallel_state.timeout_count++;
        g_hx711_parallel_state.last_timeout_ready_mask = ready_mask;
        wait_started_ms = now_ms;
    }

    if (g_hx711_parallel_state.sample_count == 0u ||
        (now_ms - g_hx711_parallel_state.last_update_ms) >= HX711_STALE_MS) {
        g_hx711_parallel_state.valid_mask = 0u;
    }

    return false;
}

bool hx711_parallel_tare(void)
{
    if (g_hx711_parallel_state.valid_mask != HX711_PARALLEL_ALL_MASK) {
        return false;
    }

    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        g_hx711_parallel_state.zero[channel] = g_hx711_parallel_state.raw[channel];
        g_hx711_parallel_state.min_delta[channel] = 0;
        g_hx711_parallel_state.max_delta[channel] = 0;
    }
    return true;
}

void hx711_parallel_reset_span(void)
{
    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        int32_t delta = g_hx711_parallel_state.raw[channel] -
                        g_hx711_parallel_state.zero[channel];
        g_hx711_parallel_state.min_delta[channel] = delta;
        g_hx711_parallel_state.max_delta[channel] = delta;
    }
}

void hx711_parallel_get_state(hx711_parallel_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->magic = g_hx711_parallel_state.magic;
    for (uint8_t channel = 0u;
         channel < HX711_PARALLEL_CHANNEL_COUNT;
         ++channel) {
        state->raw[channel] = g_hx711_parallel_state.raw[channel];
        state->zero[channel] = g_hx711_parallel_state.zero[channel];
        state->min_delta[channel] = g_hx711_parallel_state.min_delta[channel];
        state->max_delta[channel] = g_hx711_parallel_state.max_delta[channel];
    }
    state->sample_count = g_hx711_parallel_state.sample_count;
    state->last_update_ms = g_hx711_parallel_state.last_update_ms;
    state->timeout_count = g_hx711_parallel_state.timeout_count;
    state->ready_mask = g_hx711_parallel_state.ready_mask;
    state->last_timeout_ready_mask = g_hx711_parallel_state.last_timeout_ready_mask;
    state->valid_mask = g_hx711_parallel_state.valid_mask;
    state->reserved = g_hx711_parallel_state.reserved;
}
