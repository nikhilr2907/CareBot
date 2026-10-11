#include "motors.h"
#include "main.h"

/* Timer handles live in main.c (CubeMX-generated). */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

/* All four timers run at 84 MHz: TIM3 is on APB1 (PCLK1=42 MHz, *2 for timer
 * clock because the APB1 prescaler is /2); TIM1/10/11 are on APB2 (PCLK2=84
 * MHz, *1). If the clock tree in SystemClock_Config ever changes, update
 * this constant or the step frequencies will be off by the same ratio. */
#define MOTOR_TIM_CLK_HZ  84000000u
#define MOTOR_MAX_ARR     65535u
#define MOTOR_MIN_HZ      13u       /* below this, period_total > 0xFFFF * 0xFFFF */
#define MOTOR_MAX_HZ      1000000u  /* sanity ceiling -- well above DM542 limit */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;     /* TIM_CHANNEL_1..4                  */
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
} motor_cfg_t;

static motor_cfg_t motors[MOTOR_COUNT];

#define MOTOR_COORDINATED_MAX_HZ 4000u

static volatile bool coordinated_active = false;
static volatile bool coordinated_completed_pending = false;
static volatile bool coordinated_high_phase = false;
static volatile uint8_t coordinated_mask = 0u;
static volatile uint8_t coordinated_high_mask = 0u;
static volatile uint32_t coordinated_slot = 0u;
static volatile uint32_t coordinated_slot_count = 0u;
static volatile uint32_t coordinated_steps[MOTOR_COUNT];
static volatile uint32_t coordinated_emitted[MOTOR_COUNT];
static volatile uint32_t coordinated_accumulator[MOTOR_COUNT];
static uint8_t coordinated_restore_mask = 0u;

static void compute_psc_arr(uint32_t hz, uint32_t *psc, uint32_t *arr)
{
    if (hz < MOTOR_MIN_HZ) hz = MOTOR_MIN_HZ;
    if (hz > MOTOR_MAX_HZ) hz = MOTOR_MAX_HZ;

    uint32_t period_total = MOTOR_TIM_CLK_HZ / hz;
    /* Smallest prescaler that makes ARR fit in 16 bits => max ARR resolution. */
    uint32_t p = (period_total + MOTOR_MAX_ARR) / (MOTOR_MAX_ARR + 1u);
    if (p == 0u) p = 1u;
    *psc = p - 1u;
    *arr = (period_total / p) - 1u;
}

void motor_init(void)
{
    /* PUL = PWM channel, DIR = plain GPIO output. DIR pins are configured as
     * outputs by MX_GPIO_Init() in main.c -- they're driven low here at boot
     * and toggled later by motor_set_dir(). */
    motors[0] = (motor_cfg_t){ &htim3,  TIM_CHANNEL_1, GPIOB, GPIO_PIN_12 };  /* M1 PUL=PA6  DIR=PB12 */
    motors[1] = (motor_cfg_t){ &htim3,  TIM_CHANNEL_2, GPIOB, GPIO_PIN_14 };  /* M2 PUL=PA7  DIR=PB14 */
    motors[2] = (motor_cfg_t){ &htim3,  TIM_CHANNEL_3, GPIOB, GPIO_PIN_15 };  /* M3 PUL=PC8  DIR=PB15 */
    motors[3] = (motor_cfg_t){ &htim3,  TIM_CHANNEL_4, GPIOB, GPIO_PIN_2  };  /* M4 PUL=PB1  DIR=PB2  */
    motors[4] = (motor_cfg_t){ &htim1,  TIM_CHANNEL_4, GPIOC, GPIO_PIN_4  };  /* M5 PUL=PA11 DIR=PC4  */
    motors[5] = (motor_cfg_t){ &htim10, TIM_CHANNEL_1, GPIOC, GPIO_PIN_5  };  /* M6 PUL=PB8  DIR=PC5  */
    motors[6] = (motor_cfg_t){ &htim11, TIM_CHANNEL_1, GPIOC, GPIO_PIN_1  };  /* M7 PUL=PB9  DIR=PC1  */

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        HAL_GPIO_WritePin(motors[i].dir_port, motors[i].dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
    }

    /* TIM3 normally runs PWM without interrupts. Coordinated conveyor/clamp
     * moves temporarily use its update interrupt as a bounded pulse scheduler. */
    HAL_NVIC_SetPriority(TIM3_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
}

void motor_set_dir(int idx, motor_dir_t dir)
{
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    HAL_GPIO_WritePin(motors[idx].dir_port, motors[idx].dir_pin,
                      (dir == MOTOR_DIR_FWD) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void motor_stop(int idx)
{
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    if (coordinated_active || coordinated_completed_pending) {
        motor_coordinated_stop();
    }
    __HAL_TIM_SET_COMPARE(motors[idx].htim, motors[idx].channel, 0u);
}

void motor_stop_all(void)
{
    if (coordinated_active || coordinated_completed_pending) {
        motor_coordinated_stop();
    }
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
    }
}

void motor_set_speed_hz(int idx, uint32_t hz)
{
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    if (coordinated_active || coordinated_completed_pending) {
        motor_coordinated_stop();
    }
    if (hz == 0u) { motor_stop(idx); return; }

    uint32_t psc, arr;
    compute_psc_arr(hz, &psc, &arr);

    TIM_HandleTypeDef *htim      = motors[idx].htim;
    uint32_t           new_pulse = (arr + 1u) / 2u;

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);

    /* Restore 50%% duty for every active channel on this timer. Without this,
     * setting motor 1's speed would shrink ARR below motor 0's CCR and pin
     * motor 0's output permanently high (no more stepping). "Active" here =
     * CCR is currently non-zero (i.e. not motor_stop()'d). */
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (motors[i].htim != htim) continue;
        uint32_t cur_ccr = __HAL_TIM_GET_COMPARE(htim, motors[i].channel);
        if (i == idx || cur_ccr != 0u) {
            __HAL_TIM_SET_COMPARE(htim, motors[i].channel, new_pulse);
        }
    }

    /* Latch new PSC immediately (PSC is shadowed -- without UG it would only
     * take effect at the next natural update event, which on the *old* ARR
     * could be milliseconds away). Briefly resets the counter, which causes
     * at most one slightly-off pulse on already-running channels. */
    htim->Instance->EGR = TIM_EGR_UG;
}

/* STM32_LIVE_MOTOR_STEP_PATCH_V2
 * Exact step pulses for manual terminal testing.
 * The PWM pins are temporarily switched from timer alternate-function mode
 * to plain GPIO output, pulsed, then restored to the timer alternate function.
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      af;
} motor_pulse_pin_t;

static const motor_pulse_pin_t motor_pulse_pins[MOTOR_COUNT] = {
    { GPIOA, GPIO_PIN_6,  GPIO_AF2_TIM3  },  /* M1 PUL=PA6  */
    { GPIOA, GPIO_PIN_7,  GPIO_AF2_TIM3  },  /* M2 PUL=PA7  */
    { GPIOC, GPIO_PIN_8,  GPIO_AF2_TIM3  },  /* M3 PUL=PC8  */
    { GPIOB, GPIO_PIN_1,  GPIO_AF2_TIM3  },  /* M4 PUL=PB1  */
    { GPIOA, GPIO_PIN_11, GPIO_AF1_TIM1  },  /* M5 PUL=PA11 */
    { GPIOB, GPIO_PIN_8,  GPIO_AF3_TIM10 },  /* M6 PUL=PB8  */
    { GPIOB, GPIO_PIN_9,  GPIO_AF3_TIM11 }   /* M7 PUL=PB9  */
};

static void motor_delay_us(uint32_t us)
{
    if (us == 0u) return;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    if (cycles_per_us == 0u) cycles_per_us = 1u;

    uint32_t cycles = us * cycles_per_us;
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

static void motor_pulse_pin_as_gpio(int idx)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = motor_pulse_pins[idx].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(motor_pulse_pins[idx].port, &GPIO_InitStruct);
}

static void motor_pulse_pin_as_timer(int idx)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = motor_pulse_pins[idx].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = motor_pulse_pins[idx].af;
    HAL_GPIO_Init(motor_pulse_pins[idx].port, &GPIO_InitStruct);
}

static void motor_coordinated_restore_pins(uint8_t mask)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if ((mask & (uint8_t)(1u << i)) == 0u) continue;
        HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                          motor_pulse_pins[i].pin,
                          GPIO_PIN_RESET);
        motor_pulse_pin_as_timer(i);
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
    }
}

bool motor_coordinated_start(const uint32_t steps[MOTOR_COUNT],
                             const motor_dir_t directions[MOTOR_COUNT],
                             uint32_t slot_hz)
{
    if (steps == NULL || directions == NULL || slot_hz == 0u ||
        slot_hz > MOTOR_COORDINATED_MAX_HZ) {
        return false;
    }

    uint32_t max_steps = 0u;
    uint8_t mask = 0u;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (steps[i] == 0u) continue;
        mask |= (uint8_t)(1u << i);
        if (steps[i] > max_steps) max_steps = steps[i];
    }
    if (max_steps == 0u) return false;

    motor_coordinated_stop();
    motor_stop_all();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    coordinated_mask = mask;
    coordinated_restore_mask = mask;
    coordinated_high_mask = 0u;
    coordinated_slot = 0u;
    coordinated_slot_count = max_steps;
    coordinated_high_phase = false;
    coordinated_completed_pending = false;

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        coordinated_steps[i] = steps[i];
        coordinated_emitted[i] = 0u;
        coordinated_accumulator[i] = 0u;
        if ((mask & (uint8_t)(1u << i)) == 0u) continue;
        motor_set_dir(i, directions[i]);
        HAL_TIM_PWM_Stop(motors[i].htim, motors[i].channel);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
        motor_pulse_pin_as_gpio(i);
        HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                          motor_pulse_pins[i].pin,
                          GPIO_PIN_RESET);
    }

    uint32_t psc, arr;
    compute_psc_arr(slot_hz * 2u, &psc, &arr);
    __HAL_TIM_SET_PRESCALER(&htim3, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    coordinated_active = true;
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
    if (!primask) __enable_irq();
    return true;
}

void motor_coordinated_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
    uint8_t mask = coordinated_restore_mask;
    coordinated_active = false;
    coordinated_completed_pending = false;
    coordinated_high_phase = false;
    coordinated_high_mask = 0u;
    coordinated_mask = 0u;
    coordinated_restore_mask = 0u;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if ((mask & (uint8_t)(1u << i)) != 0u) {
            HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                              motor_pulse_pins[i].pin,
                              GPIO_PIN_RESET);
        }
    }
    if (!primask) __enable_irq();
    motor_coordinated_restore_pins(mask);
}

void motor_coordinated_tim3_irq(void)
{
    if (!coordinated_active) return;

    if (coordinated_high_phase) {
        uint8_t high_mask = coordinated_high_mask;
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            if ((high_mask & (uint8_t)(1u << i)) != 0u) {
                HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                                  motor_pulse_pins[i].pin,
                                  GPIO_PIN_RESET);
            }
        }
        coordinated_high_mask = 0u;
        coordinated_high_phase = false;
        if (coordinated_slot >= coordinated_slot_count) {
            coordinated_active = false;
            coordinated_completed_pending = true;
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        }
        return;
    }

    uint8_t pulse_mask = 0u;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1u << i);
        if ((coordinated_mask & bit) == 0u ||
            coordinated_emitted[i] >= coordinated_steps[i]) {
            continue;
        }
        coordinated_accumulator[i] += coordinated_steps[i];
        if (coordinated_accumulator[i] >= coordinated_slot_count) {
            coordinated_accumulator[i] -= coordinated_slot_count;
            coordinated_emitted[i]++;
            pulse_mask |= bit;
            HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                              motor_pulse_pins[i].pin,
                              GPIO_PIN_SET);
        }
    }
    coordinated_high_mask = pulse_mask;
    coordinated_slot++;
    coordinated_high_phase = true;
}

bool motor_coordinated_service(void)
{
    if (!coordinated_completed_pending) return false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t mask = coordinated_restore_mask;
    coordinated_completed_pending = false;
    coordinated_restore_mask = 0u;
    coordinated_mask = 0u;
    if (!primask) __enable_irq();
    motor_coordinated_restore_pins(mask);
    return true;
}

void motor_step_pulses(int idx, motor_dir_t dir, uint32_t steps, uint32_t pulse_us)
{
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    if (steps == 0u) return;

    if (pulse_us < 2u) pulse_us = 2u;
    if (pulse_us > 100000u) pulse_us = 100000u;
    if (steps > 10000u) steps = 10000u;

    motor_stop_all();
    motor_set_dir(idx, dir);
    motor_delay_us(50u);

    HAL_TIM_PWM_Stop(motors[idx].htim, motors[idx].channel);
    __HAL_TIM_SET_COMPARE(motors[idx].htim, motors[idx].channel, 0u);

    motor_pulse_pin_as_gpio(idx);
    HAL_GPIO_WritePin(motor_pulse_pins[idx].port, motor_pulse_pins[idx].pin, GPIO_PIN_RESET);
    motor_delay_us(20u);

    for (uint32_t i = 0u; i < steps; ++i) {
        HAL_GPIO_WritePin(motor_pulse_pins[idx].port, motor_pulse_pins[idx].pin, GPIO_PIN_SET);
        motor_delay_us(pulse_us);
        HAL_GPIO_WritePin(motor_pulse_pins[idx].port, motor_pulse_pins[idx].pin, GPIO_PIN_RESET);
        motor_delay_us(pulse_us);
    }

    motor_pulse_pin_as_timer(idx);
    HAL_TIM_PWM_Start(motors[idx].htim, motors[idx].channel);
    motor_stop(idx);
}

void motor_step_all_pulses(motor_dir_t dir, uint32_t steps, uint32_t pulse_us)
{
    if (steps == 0u) return;

    if (pulse_us < 2u) pulse_us = 2u;
    if (pulse_us > 100000u) pulse_us = 100000u;
    if (steps > 10000u) steps = 10000u;

    motor_stop_all();

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_set_dir(i, dir);
        HAL_TIM_PWM_Stop(motors[i].htim, motors[i].channel);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
        motor_pulse_pin_as_gpio(i);
        HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_RESET);
    }

    motor_delay_us(50u);

    for (uint32_t step = 0u; step < steps; ++step) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_SET);
        }
        motor_delay_us(pulse_us);
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_RESET);
        }
        motor_delay_us(pulse_us);
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_pulse_pin_as_timer(i);
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
        motor_stop(i);
    }
}

void motor_step_all_pulses_active_low(motor_dir_t dir, uint32_t steps, uint32_t pulse_us)
{
    if (steps == 0u) return;

    if (pulse_us < 2u) pulse_us = 2u;
    if (pulse_us > 100000u) pulse_us = 100000u;
    if (steps > 10000u) steps = 10000u;

    motor_stop_all();

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_set_dir(i, dir);
        HAL_TIM_PWM_Stop(motors[i].htim, motors[i].channel);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
        motor_pulse_pin_as_gpio(i);
        HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_SET);
    }

    motor_delay_us(50u);

    for (uint32_t step = 0u; step < steps; ++step) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_RESET);
        }
        motor_delay_us(pulse_us);
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_SET);
        }
        motor_delay_us(pulse_us);
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_RESET);
        motor_pulse_pin_as_timer(i);
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
        motor_stop(i);
    }
}

void motor_force_all_pul_gpio(bool high)
{
    motor_stop_all();

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        HAL_TIM_PWM_Stop(motors[i].htim, motors[i].channel);
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0u);
        motor_pulse_pin_as_gpio(i);
        HAL_GPIO_WritePin(motor_pulse_pins[i].port,
                          motor_pulse_pins[i].pin,
                          high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void motor_release_all_pul_timer(void)
{
    motor_coordinated_stop();
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        HAL_GPIO_WritePin(motor_pulse_pins[i].port, motor_pulse_pins[i].pin, GPIO_PIN_RESET);
        motor_pulse_pin_as_timer(i);
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
        motor_stop(i);
    }
}
