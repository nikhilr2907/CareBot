#include "lac_actuators.h"

#if LAC_RC_TEST_MODE
#define LAC_PWM_MIN_COUNTS    1000u
#define LAC_PWM_SPAN_COUNTS   1000u
#else
#define LAC_PWM_MIN_COUNTS      10u
#define LAC_PWM_SPAN_COUNTS    980u
#endif

static TIM_HandleTypeDef *lac_timer;
static uint8_t commanded_percent[LAC_ACTUATOR_COUNT];

static const uint32_t lac_channels[LAC_ACTUATOR_COUNT] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
};

static uint32_t percent_to_compare(uint8_t percent)
{
    /* RC test mode maps 0..100% to 1..2 ms; normal mode maps to 1..99%. */
    return LAC_PWM_MIN_COUNTS +
           (((uint32_t)percent * LAC_PWM_SPAN_COUNTS) / 100u);
}

bool lac_actuators_init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM2) {
        return false;
    }

    lac_timer = timer;
    for (uint8_t i = 0u; i < LAC_ACTUATOR_COUNT; ++i) {
        commanded_percent[i] = 0u;
        __HAL_TIM_SET_COMPARE(lac_timer, lac_channels[i],
                              percent_to_compare(0u));
        if (HAL_TIM_PWM_Start(lac_timer, lac_channels[i]) != HAL_OK) {
            while (i > 0u) {
                --i;
                HAL_TIM_PWM_Stop(lac_timer, lac_channels[i]);
            }
            lac_timer = NULL;
            return false;
        }
    }

    return true;
}

bool lac_actuator_set_percent(uint8_t actuator, uint8_t percent)
{
    if (lac_timer == NULL || actuator < 1u ||
        actuator > LAC_ACTUATOR_COUNT || percent > 100u) {
        return false;
    }

    uint8_t index = actuator - 1u;
    __HAL_TIM_SET_COMPARE(lac_timer, lac_channels[index],
                          percent_to_compare(percent));
    commanded_percent[index] = percent;
    return true;
}

bool lac_actuators_set_both_percent(uint8_t percent)
{
    if (percent > 100u) {
        return false;
    }

    return lac_actuator_set_percent(1u, percent) &&
           lac_actuator_set_percent(2u, percent);
}

uint8_t lac_actuator_get_percent(uint8_t actuator)
{
    if (actuator < 1u || actuator > LAC_ACTUATOR_COUNT) {
        return 0u;
    }

    return commanded_percent[actuator - 1u];
}
