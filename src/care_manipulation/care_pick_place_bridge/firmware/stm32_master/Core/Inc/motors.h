#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_COUNT 7

typedef enum {
    MOTOR_DIR_FWD = 0,
    MOTOR_DIR_REV = 1,
} motor_dir_t;

/* Initialize all 7 motors. Call AFTER MX_GPIO_Init() and the MX_TIMx_Init()
 * functions in main(). Starts each PWM channel with 0%% duty (pulses gated
 * off) and DIR low. After this, motor_set_speed_hz() begins emitting pulses.
 *
 * IMPORTANT: motors 0..3 (Motor 1..4 in the schematic) all share TIM3. PSC
 * and ARR are timer-wide, so calling motor_set_speed_hz() for any of them
 * changes the step rate for all four. Each channel can still be stopped
 * independently via motor_stop(). Motors 4, 5, 6 (Motor 5, 6, 7) live on
 * TIM1, TIM10, TIM11 respectively and can be driven at independent rates. */
void motor_init(void);

/* Set the step pulse rate for one motor in Hz.
 *   hz == 0  -> stop this motor (sets CCR=0, timer keeps running)
 *   hz >  0  -> emit a 50%% duty square wave at approximately `hz`
 * Range is roughly 13 Hz .. 1 MHz; out-of-range values are clamped. The
 * actual frequency may differ by < 0.1%% from the request due to integer
 * PSC/ARR rounding. */
void motor_set_speed_hz(int idx, uint32_t hz);

/* Set the DIR pin for one motor. FWD = pin low, REV = pin high; the
 * mechanical direction those map to depends on how you've wired the
 * motor phases at the DM542. */
void motor_set_dir(int idx, motor_dir_t dir);

/* Gate pulses for one motor without disabling its timer. */
void motor_stop(int idx);

/* Convenience: stop every motor. */
void motor_stop_all(void);

/* Run several steppers over one shared time window with independent, exact
 * pulse totals. TIM3 is used as a software pulse distributor so M1/M3/M4 can
 * move at different average rates even though their normal PWM channels share
 * one timer period. `slot_hz` is the rate of the motor with the largest step
 * count; all selected motors start together and finish together. */
bool motor_coordinated_start(const uint32_t steps[MOTOR_COUNT],
                             const motor_dir_t directions[MOTOR_COUNT],
                             uint32_t slot_hz);

/* Abort a coordinated run and return its PUL pins to normal timer mode. */
void motor_coordinated_stop(void);

/* Called by TIM3_IRQHandler; not intended for application code. */
void motor_coordinated_tim3_irq(void);

/* Restore normal PWM mode after a coordinated run completes. Returns true
 * once for each completed run. */
bool motor_coordinated_service(void);

/* Emit a fixed number of exact PUL pulses using GPIO bit-banging. This is for
 * manual bring-up/jog testing where a tiny repeatable movement is easier to
 * reason about than a continuous PWM run. */
void motor_step_pulses(int idx, motor_dir_t dir, uint32_t steps, uint32_t pulse_us);

/* Diagnostic only: pulse every PUL output together using GPIO bit-banging.
 * Useful when only one external driver/motor is connected and the matching
 * STM32 motor number is unknown. */
void motor_step_all_pulses(motor_dir_t dir, uint32_t steps, uint32_t pulse_us);

/* Diagnostic only: same as motor_step_all_pulses(), but PUL idles high and
 * each pulse pulls low. This helps test common-anode opto-input wiring. */
void motor_step_all_pulses_active_low(motor_dir_t dir, uint32_t steps, uint32_t pulse_us);

/* Diagnostic only: force every PUL pin to GPIO output high/low so it can be
 * checked with a multimeter at the driver input terminal. */
void motor_force_all_pul_gpio(bool high);

/* Return every PUL pin to timer/PWM mode after motor_force_all_pul_gpio(). */
void motor_release_all_pul_timer(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
