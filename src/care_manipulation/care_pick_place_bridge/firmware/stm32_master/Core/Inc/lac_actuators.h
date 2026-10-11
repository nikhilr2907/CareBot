#ifndef LAC_ACTUATORS_H
#define LAC_ACTUATORS_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define LAC_ACTUATOR_COUNT 2u

/* Temporary bench mode: TIM2 outputs standard 50 Hz RC servo pulses. */
#define LAC_RC_TEST_MODE 1

bool lac_actuators_init(TIM_HandleTypeDef *timer);
bool lac_actuator_set_percent(uint8_t actuator, uint8_t percent);
bool lac_actuators_set_both_percent(uint8_t percent);
uint8_t lac_actuator_get_percent(uint8_t actuator);

#endif
