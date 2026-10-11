/* Manual CLI terminal on USART2
 * ---------------------------------------------------------------------------
 * Line-oriented command interface. HAL_UART_RxCpltCallback(USART2) feeds each
 * byte to cli_handle_usart2_rx_cplt(); once a '\n' or '\r' arrives the module
 * marks the buffer ready, and cli_service() dispatches it on the next
 * main-loop tick. cli_service() also owns the lift-timeout check and the
 * optional load-cell / switch streaming prints.
 *
 * Commands:
 *   M <motor> <hz> [F|R]              continuous run, one stepper only
 *   G <list|ALL> <hz> [F|R]           continuous run, motor group
 *   D <m> <hz> <F|R> [...]            run N motors with independent dirs
 *   C <slot_hz> <m> <steps> <F|R>...  exact coordinated multi-motor move
 *   LIFT <UP|DOWN> <m2_hz> <m5_hz> <ms>   timed independent Y-axis run
 *   J <motor> <steps> [F|R] <pulse_us>    exact step pulses, one motor
 *   X|Y <steps> [F|R] <pulse_us>      diagnostic: pulse all PUL pins
 *   P <0|1> / N                       diagnostic: force/release all PUL pins
 *   S / A                             stop all steppers
 *   LAC <0|1> <0-100>                 linear-actuator percent
 *   LC STATUS|TARE|STREAM|RESET       load-cell diagnostics
 *   SW STATUS                         limit-switch state
 *   LOG 0|1                           quiet / verbose terminal logs
 *   ARM / DISARM                      enable / force-stop all motion
 * V <left_rad_s> <right_rad_s>  stays wired to drive_apply_v() in drive_app.c. */

#include "cli_app.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug.h"
#include "direct_tof_app.h"
#include "drive_app.h"
#include "hx711_parallel.h"
#include "lac_actuators.h"
#include "main.h"
#include "motors.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

/* ===========================================================================
 *                                   State
 * ========================================================================= */

/* USART2 single-byte RX ISR state. The ISR builds lines into cmd_buf and sets
 * cmd_ready when it sees '\n' or '\r'. While cmd_ready is set the ISR drops
 * new chars so cli_service() has a stable buffer to copy. */
#define CMD_BUF_SIZE 64
static volatile char    cmd_buf[CMD_BUF_SIZE];
static volatile uint8_t cmd_idx   = 0u;
static volatile bool    cmd_ready = false;
static volatile uint8_t usart2_rx_byte;
static volatile uint32_t usart2_rx_count = 0u;
static volatile uint32_t usart2_line_count = 0u;
static volatile uint32_t disarmed_reject_count = 0u;

/* LOG flag -- gates verbose terminal prints across modules via
 * cli_terminal_log_enabled(). */
static bool terminal_log_enabled = false;

/* Raw load-cell diagnostics are separately rate-limited so they can be used
 * without enabling the much noisier range-sensor and IMU log stream. */
#define LOAD_CELL_PRINT_PERIOD_MS 250u
static bool     load_cell_stream_enabled = false;
static uint32_t load_cell_last_print_ms  = 0u;

/* Two active-low mechanical/limit-switch inputs. The internal pull-ups keep
 * open contacts at a defined HIGH level; a closed contact pulls the pin LOW. */
#define SWITCH_1_GPIO_PORT       GPIOC
#define SWITCH_1_GPIO_PIN        GPIO_PIN_6
#define SWITCH_2_GPIO_PORT       GPIOC
#define SWITCH_2_GPIO_PIN        GPIO_PIN_9
#define SWITCH_PRINT_PERIOD_MS   250u
static bool     switch_stream_enabled = false;
static uint32_t switch_last_print_ms  = 0u;
static uint32_t switch_pull_mode      = GPIO_PULLUP;

/* Timed two-motor lift command used by the local calibration console. M2 is
 * the right Y axis and M5 is the left Y axis. The board owns the timeout so
 * the outputs stop even if the browser, host process, or USB link disappears. */
#define LIFT_M2_INDEX             1
#define LIFT_M5_INDEX             4
#define LIFT_MIN_HZ             100u
#define LIFT_MAX_HZ            5000u
#define LIFT_MIN_DURATION_MS     100u
#define LIFT_MAX_DURATION_MS   10000u
static bool     lift_timed_active = false;
static uint32_t lift_started_ms   = 0u;
static uint32_t lift_duration_ms  = 0u;

static char cli_upper(char c)
{
    if (c >= 'a' && c <= 'z') return (char)(c - ('a' - 'A'));
    return c;
}

static const char *cli_skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static const char *cli_skip_alpha(const char *p)
{
    while ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) p++;
    return p;
}

// Advance past the command keyword so the dispatcher can hand remaining args
// to the handler. Lets "M 1 F 500" and "MOTOR 1 F 500" both resolve the same.
static const char *cli_skip_cmd_word(const char *p)
{
    p++;
    if ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) p = cli_skip_alpha(p);
    return p;
}

static bool cli_parse_long(const char **pp, long *out)
{
    const char *p = cli_skip_ws(*pp);
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    *out = v;
    *pp = end;
    return true;
}

static bool cli_word_is(const char *p, const char *word)
{
    p = cli_skip_ws(p);
    while (*word != '\0') {
        if (cli_upper(*p) != cli_upper(*word)) return false;
        p++;
        word++;
    }
    return (*p == '\0' || *p == ' ' || *p == '\t');
}

static bool cli_parse_dir_token(const char **pp, motor_dir_t *dir)
{
    const char *p = cli_skip_ws(*pp);
    if (*p == '\0') {
        *pp = p;
        return true;
    }

    char c = cli_upper(*p);
    if (c == 'F' || c == '+') {
        *dir = MOTOR_DIR_FWD;
    } else if (c == 'R' || c == '-') {
        *dir = ((motor_dir_t)1);
    } else {
        return false;
    }

    while (*p != '\0' && *p != ' ' && *p != '\t') p++;
    *pp = p;
    return true;
}

/* Range-check a parsed motor index (1..MOTOR_COUNT). Prints the standard
 * error and returns false if out of range so the caller can early-exit.      */
static bool cli_validate_motor_num(long motor_num)
{
    if (motor_num < 1 || motor_num > MOTOR_COUNT) {
        dbg_printf("ERR: motor must be 1..%d\r\n", MOTOR_COUNT);
        return false;
    }
    return true;
}

/* Try to parse one optional long argument. If no number is present the
 * input cursor is left untouched and *out is set to fallback.                */
static void cli_parse_optional_long(const char **pp, long *out, long fallback)
{
    const char *saved = *pp;
    if (!cli_parse_long(pp, out)) {
        *pp = saved;
        *out = fallback;
    }
}

/* Normalise a signed Hz speed into unsigned magnitude + direction.
 * Negative Hz means reverse. Caller may override dir later with an explicit
 * F/R token.                                                                 */
static uint32_t cli_hz_to_unsigned(long hz_signed, motor_dir_t *out_dir)
{
    if (hz_signed < 0) {
        *out_dir = MOTOR_DIR_REV;
        return (uint32_t)(-hz_signed);
    }
    *out_dir = MOTOR_DIR_FWD;
    return (uint32_t)hz_signed;
}

static void lift_cancel_timeout(void)
{
    lift_timed_active = false;
}

static void lift_stop_outputs(void)
{
    motor_stop(LIFT_M2_INDEX);
    motor_stop(LIFT_M5_INDEX);
    lift_timed_active = false;
}

/* Common pre-run sequence shared by every CLI motion command: drop any armed
 * lift timeout, release PUL pins from any manual-force state, then stop all
 * steppers. Callers are expected to arm their own motion right after.        */
static void motor_prepare_run(void)
{
    lift_cancel_timeout();
    motor_release_all_pul_timer();
    motor_stop_all();
}

static void cli_print_help(void)
{
    dbg_printf("\r\nManual motor commands on USART2 @ 115200:\r\n");
    dbg_printf("  ARM / DISARM                enable motion / force every output safe\r\n");
    dbg_printf("  M <1-%d> <hz> [F|R]       continuous run one stepper only\r\n", MOTOR_COUNT);
    dbg_printf("  G <list|ALL> <hz> [F|R]  continuous run motor group, e.g. G 1,5 100 F\r\n");
    dbg_printf("  D <motor> <hz> <F|R> [...]  run 2-7 motors with independent directions\r\n");
    dbg_printf("  C <slot_hz> <motor> <steps> <F|R> [...]  exact coordinated run\r\n");
    dbg_printf("  LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>  timed independent Y-axis run\r\n");
    dbg_printf("  J <1-%d> <steps> [F|R] <pulse_us>  exact step pulses\r\n", MOTOR_COUNT);
    dbg_printf("  X <steps> [F|R] <pulse_us>  diagnostic: pulse all PUL pins together\r\n");
    dbg_printf("  Y <steps> [F|R] <pulse_us>  diagnostic: active-low all-PUL pulse\r\n");
    dbg_printf("  P 0 / P 1                 diagnostic: hold every PUL pin low/high\r\n");
    dbg_printf("  N                         restore PUL pins to normal timer mode\r\n");
    dbg_printf("  S or A                    stop all steppers\r\n");
    dbg_printf("  LOG 0 / LOG 1             quiet / verbose board messages\r\n");
    dbg_printf("  LC STATUS                  one raw four-load-cell snapshot\r\n");
    dbg_printf("  LC TARE                    zero all four currently unloaded cells\r\n");
    dbg_printf("  LC STREAM 0|1              stop/start raw load-cell stream\r\n");
    dbg_printf("  LC RESET                   reset observed min/max signal span\r\n");
    dbg_printf("  SW STATUS                  read PC6 and PC9 switch levels\r\n");
    dbg_printf("  SW STREAM 0|1              stop/start switch-level stream\r\n");
    dbg_printf("  SW PULL NONE|UP|DOWN       configure input pull resistors\r\n");
    dbg_printf("  TOF STATUS                  read physical-right D13/PA5 ToF only\r\n");
    dbg_printf("  LAC <1|2> <0-100>          set one actuator position percent\r\n");
    dbg_printf("  LAC BOTH <0-100>           set both actuator positions\r\n");
    dbg_printf("  LAC HOME / LAC STATUS      retract both / show commanded positions\r\n");
    dbg_printf("Example: M 1 500 F, G ALL 100 F, then S.  Example step: J 1 1 F 500\r\n\r\n");
}

static void cli_lac(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        dbg_printf("LAC commanded: 1=%u%% LEFT on A0/PA0, "
                   "2=%u%% RIGHT on A1/PA1; signal="
#if LAC_RC_TEST_MODE
                   "temporary RC 50Hz (1-2ms); "
#else
                   "3.3V 1kHz PWM (logical 0-100%% maps to 1-99%% duty); "
#endif
                   "command only, no feedback\r\n",
                   (unsigned)lac_actuator_get_percent(1u),
                   (unsigned)lac_actuator_get_percent(2u));
        return;
    }

    if (cli_word_is(args, "HOME")) {
        if (!lac_actuators_set_both_percent(0u)) {
            dbg_printf("ERR: LAC outputs are not initialized\r\n");
            return;
        }
        dbg_printf("OK: both LAC actuators commanded HOME (0%%)\r\n");
        return;
    }

    if (cli_word_is(args, "BOTH")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long percent = 0;
        if (!cli_parse_long(&args, &percent) || percent < 0 || percent > 100) {
            dbg_printf("ERR: use LAC BOTH <0-100>\r\n");
            return;
        }
        if (!lac_actuators_set_both_percent((uint8_t)percent)) {
            dbg_printf("ERR: LAC outputs are not initialized\r\n");
            return;
        }
        dbg_printf("OK: both LAC actuators commanded %ld%%\r\n", percent);
        return;
    }

    long actuator = 0;
    long percent = 0;
    if (!cli_parse_long(&args, &actuator) ||
        !cli_parse_long(&args, &percent) ||
        actuator < 1 || actuator > (long)LAC_ACTUATOR_COUNT ||
        percent < 0 || percent > 100) {
        dbg_printf("ERR: use LAC <1|2> <0-100>, LAC BOTH <0-100>, "
                   "LAC HOME, or LAC STATUS\r\n");
        return;
    }

    if (!lac_actuator_set_percent((uint8_t)actuator, (uint8_t)percent)) {
        dbg_printf("ERR: LAC outputs are not initialized\r\n");
        return;
    }
    dbg_printf("OK: LAC %ld commanded %ld%%\r\n", actuator, percent);
}

static void cli_run_stepper(const char *args)
{
    long motor_num = 0;
    long hz_signed = 0;

    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: missing motor number. Example: M 1 500 F\r\n");
        return;
    }
    if (!cli_validate_motor_num(motor_num)) return;
    if (!cli_parse_long(&args, &hz_signed)) {
        dbg_printf("ERR: missing speed in Hz. Example: M %ld 500 F\r\n", motor_num);
        return;
    }

    motor_dir_t dir;
    uint32_t hz = cli_hz_to_unsigned(hz_signed, &dir);

    const char *dir_args = args;
    if (*cli_skip_ws(dir_args) != '\0' && !cli_parse_dir_token(&dir_args, &dir)) {
        dbg_printf("ERR: direction must be F or R. Example: M %ld %lu R\r\n",
                   motor_num, (unsigned long)hz);
        return;
    }

    motor_prepare_run();

    if (hz == 0u) {
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    int idx = (int)motor_num - 1;
    motor_set_dir(idx, dir);
    motor_set_speed_hz(idx, hz);
    dbg_printf("OK: M%ld continuous %lu Hz DIR=%s\r\n",
               motor_num, (unsigned long)hz,
               (dir == MOTOR_DIR_FWD) ? "F" : "R");
}

static bool cli_parse_motor_mask(const char **pp, uint8_t *mask)
{
    const char *p = cli_skip_ws(*pp);
    uint8_t parsed = 0u;

    if (*p == '*') {
        *mask = (uint8_t)((1u << MOTOR_COUNT) - 1u);
        *pp = p + 1;
        return true;
    }

    if ((p[0] == 'A' || p[0] == 'a') &&
        (p[1] == 'L' || p[1] == 'l') &&
        (p[2] == 'L' || p[2] == 'l') &&
        (p[3] == '\0' || p[3] == ' ' || p[3] == '\t')) {
        *mask = (uint8_t)((1u << MOTOR_COUNT) - 1u);
        *pp = p + 3;
        return true;
    }

    while (true) {
        long motor_num = 0;

        p = cli_skip_ws(p);
        if (*p < '0' || *p > '9') {
            return false;
        }

        while (*p >= '0' && *p <= '9') {
            motor_num = motor_num * 10 + (*p - '0');
            p++;
        }

        if (motor_num < 1 || motor_num > MOTOR_COUNT) {
            return false;
        }
        parsed |= (uint8_t)(1u << (motor_num - 1));

        const char *q = cli_skip_ws(p);
        if (*q == ',' || *q == '+' || *q == '/') {
            p = q + 1;
            continue;
        }

        *mask = parsed;
        *pp = q;
        return parsed != 0u;
    }
}

static void cli_run_stepper_group(const char *args)
{
    uint8_t mask = 0u;
    long hz_signed = 0;

    if (!cli_parse_motor_mask(&args, &mask)) {
        dbg_printf("ERR: missing motor list. Example: G 1,5 100 F or G ALL 100 F\r\n");
        return;
    }
    if (!cli_parse_long(&args, &hz_signed)) {
        dbg_printf("ERR: missing speed in Hz. Example: G 1,5 100 F\r\n");
        return;
    }

    motor_dir_t dir;
    uint32_t hz = cli_hz_to_unsigned(hz_signed, &dir);

    const char *dir_args = args;
    if (*cli_skip_ws(dir_args) != '\0' && !cli_parse_dir_token(&dir_args, &dir)) {
        dbg_printf("ERR: direction must be F or R. Example: G 1,5 %lu R\r\n",
                   (unsigned long)hz);
        return;
    }

    motor_prepare_run();

    if (hz == 0u) {
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) == 0u) continue;
        motor_set_dir(idx, dir);
        motor_set_speed_hz(idx, hz);
    }

    dbg_printf("OK: group mask 0x%02X continuous %lu Hz DIR=%s\r\n",
               mask, (unsigned long)hz,
               (dir == MOTOR_DIR_FWD) ? "F" : "R");
}

static void cli_run_stepper_pair(const char *args)
{
    uint32_t speeds[MOTOR_COUNT] = {0u};
    motor_dir_t directions[MOTOR_COUNT];
    uint8_t mask = 0u;
    int count = 0;

    while (*cli_skip_ws(args) != '\0') {
        long motor_num = 0;
        long hz = 0;
        motor_dir_t dir = MOTOR_DIR_FWD;

        if (!cli_parse_long(&args, &motor_num) ||
            !cli_parse_long(&args, &hz) ||
            *cli_skip_ws(args) == '\0' ||
            !cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: use D <motor> <hz> <F|R> repeated 2-%d times\r\n",
                       MOTOR_COUNT);
            return;
        }

        if (motor_num < 1 || motor_num > MOTOR_COUNT || hz < 0) {
            dbg_printf("ERR: motors must be 1-%d and speeds must be non-negative\r\n",
                       MOTOR_COUNT);
            return;
        }

        int idx = (int)motor_num - 1;
        uint8_t bit = (uint8_t)(1u << idx);
        if ((mask & bit) != 0u) {
            dbg_printf("ERR: M%ld was listed more than once\r\n", motor_num);
            return;
        }

        mask |= bit;
        speeds[idx] = (uint32_t)hz;
        directions[idx] = dir;
        count++;
    }

    if (count < 2) {
        dbg_printf("ERR: D requires at least two different motors\r\n");
        return;
    }

    motor_prepare_run();

    /* Set every direction before emitting the first pulse, then start all
     * selected channels back-to-back. Motors sharing TIM3 must use the same
     * frequency; motor_set_speed_hz() keeps its active channels synchronized. */
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            motor_set_dir(idx, directions[idx]);
        }
    }
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            motor_set_speed_hz(idx, speeds[idx]);
        }
    }

    dbg_printf("OK: D started %d motors, mask=0x%02X\r\n", count, mask);
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if ((mask & (uint8_t)(1u << idx)) != 0u) {
            dbg_printf("  M%d %lu Hz DIR=%s\r\n", idx + 1,
                       (unsigned long)speeds[idx],
                       (directions[idx] == MOTOR_DIR_FWD) ? "F" : "R");
        }
    }
}

static void cli_run_coordinated(const char *args)
{
    long slot_hz_signed = 0;
    uint32_t steps[MOTOR_COUNT] = {0u};
    motor_dir_t directions[MOTOR_COUNT];
    uint8_t mask = 0u;
    int count = 0;

    if (!cli_parse_long(&args, &slot_hz_signed) ||
        slot_hz_signed < 1 || slot_hz_signed > 4000) {
        dbg_printf("ERR: coordinated slot rate must be 1..4000 Hz\r\n");
        return;
    }

    while (*cli_skip_ws(args) != '\0') {
        long motor_num = 0;
        long motor_steps = 0;
        motor_dir_t dir = MOTOR_DIR_FWD;
        if (!cli_parse_long(&args, &motor_num) ||
            !cli_parse_long(&args, &motor_steps) ||
            *cli_skip_ws(args) == '\0' ||
            !cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: use C <slot_hz> <motor> <steps> <F|R> repeated\r\n");
            return;
        }
        if (motor_num < 1 || motor_num > MOTOR_COUNT ||
            motor_steps < 1 || motor_steps > 100000) {
            dbg_printf("ERR: motors must be 1-%d and steps must be 1..100000\r\n",
                       MOTOR_COUNT);
            return;
        }
        int idx = (int)motor_num - 1;
        uint8_t bit = (uint8_t)(1u << idx);
        if ((mask & bit) != 0u) {
            dbg_printf("ERR: M%ld was listed more than once\r\n", motor_num);
            return;
        }
        mask |= bit;
        steps[idx] = (uint32_t)motor_steps;
        directions[idx] = dir;
        count++;
    }

    if (count < 2) {
        dbg_printf("ERR: C requires at least two different motors\r\n");
        return;
    }

    motor_prepare_run();
    if (!motor_coordinated_start(steps, directions, (uint32_t)slot_hz_signed)) {
        dbg_printf("ERR: coordinated motion could not start\r\n");
        return;
    }

    uint32_t max_steps = 0u;
    for (int idx = 0; idx < MOTOR_COUNT; ++idx) {
        if (steps[idx] > max_steps) max_steps = steps[idx];
    }
    dbg_printf("OK: C started %d motors, slot=%ld Hz, max=%lu steps, about %lums\r\n",
               count, slot_hz_signed, (unsigned long)max_steps,
               (unsigned long)((max_steps * 1000u) /
                               (uint32_t)slot_hz_signed));
}

static void cli_run_lift(const char *args)
{
    args = cli_skip_ws(args);

    motor_dir_t dir;
    const char *direction_name;
    if (cli_word_is(args, "UP")) {
        dir = MOTOR_DIR_REV;
        direction_name = "UP";
    } else if (cli_word_is(args, "DOWN")) {
        dir = MOTOR_DIR_FWD;
        direction_name = "DOWN";
    } else {
        dbg_printf("ERR: use LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>\r\n");
        return;
    }
    args = cli_skip_alpha(args);

    long m2_hz = 0;
    long m5_hz = 0;
    long duration_ms = 0;
    if (!cli_parse_long(&args, &m2_hz) ||
        !cli_parse_long(&args, &m5_hz) ||
        !cli_parse_long(&args, &duration_ms)) {
        dbg_printf("ERR: use LIFT <UP|DOWN> <M2_hz> <M5_hz> <ms>\r\n");
        return;
    }

    bool m2_valid = (m2_hz == 0) ||
                    (m2_hz >= (long)LIFT_MIN_HZ && m2_hz <= (long)LIFT_MAX_HZ);
    bool m5_valid = (m5_hz == 0) ||
                    (m5_hz >= (long)LIFT_MIN_HZ && m5_hz <= (long)LIFT_MAX_HZ);
    if (!m2_valid || !m5_valid || (m2_hz == 0 && m5_hz == 0)) {
        dbg_printf("ERR: each lift speed must be 0 or %u..%u Hz; at least one must run\r\n",
                   (unsigned)LIFT_MIN_HZ, (unsigned)LIFT_MAX_HZ);
        return;
    }
    if (duration_ms < (long)LIFT_MIN_DURATION_MS ||
        duration_ms > (long)LIFT_MAX_DURATION_MS) {
        dbg_printf("ERR: lift duration must be %u..%u ms\r\n",
                   (unsigned)LIFT_MIN_DURATION_MS,
                   (unsigned)LIFT_MAX_DURATION_MS);
        return;
    }

    motor_prepare_run();
    motor_set_dir(LIFT_M2_INDEX, dir);
    motor_set_dir(LIFT_M5_INDEX, dir);
    motor_set_speed_hz(LIFT_M2_INDEX, (uint32_t)m2_hz);
    motor_set_speed_hz(LIFT_M5_INDEX, (uint32_t)m5_hz);

    lift_started_ms = HAL_GetTick();
    lift_duration_ms = (uint32_t)duration_ms;
    lift_timed_active = true;

    dbg_printf("OK: LIFT %s M2=%ldHz M5=%ldHz duration=%ldms\r\n",
               direction_name, m2_hz, m5_hz, duration_ms);
}

static void cli_jog_stepper(const char *args)
{
    long motor_num = 0;
    long steps = 1;
    long pulse_us = 500;
    motor_dir_t dir = MOTOR_DIR_FWD;

    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: missing motor number. Example: J 1 1 F 500\r\n");
        return;
    }
    if (!cli_validate_motor_num(motor_num)) return;

    cli_parse_optional_long(&args, &steps, 1);

    const char *p = cli_skip_ws(args);
    if (*p != '\0' && (cli_upper(*p) == 'F' || cli_upper(*p) == 'R' || *p == '+' || *p == '-')) {
        if (!cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: direction must be F or R. Example: J %ld %ld R 500\r\n", motor_num, steps);
            return;
        }
    }

    cli_parse_optional_long(&args, &pulse_us, 500);

    if (steps < 1) steps = 1;
    if (steps > 10000) steps = 10000;
    if (pulse_us < 2) pulse_us = 2;
    if (pulse_us > 100000) pulse_us = 100000;

    motor_prepare_run();
    motor_step_pulses((int)motor_num - 1, dir, (uint32_t)steps, (uint32_t)pulse_us);
    motor_stop_all();

    dbg_printf("OK: M%ld step/jog %ld step(s), DIR=%s, pulse=%ld us\r\n",
               motor_num, steps,
               (dir == MOTOR_DIR_FWD) ? "F" : "R",
               pulse_us);
}

static void cli_stop_stepper(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "ALL") || *args == '*') {
        motor_prepare_run();
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    long motor_num = 0;
    if (!cli_parse_long(&args, &motor_num)) {
        dbg_printf("ERR: use S, S ALL, or S <1-%d>\r\n", MOTOR_COUNT);
        return;
    }
    if (!cli_validate_motor_num(motor_num)) return;

    if (motor_num == (LIFT_M2_INDEX + 1) || motor_num == (LIFT_M5_INDEX + 1)) {
        lift_cancel_timeout();
    }
    motor_stop((int)motor_num - 1);
    dbg_printf("OK: M%ld stopped\r\n", motor_num);
}

static void cli_force_pul_pins(const char *args)
{
    long high = 0;
    if (!cli_parse_long(&args, &high)) {
        dbg_printf("ERR: use P 0 or P 1\r\n");
        return;
    }

    lift_cancel_timeout();
    motor_force_all_pul_gpio(high != 0);
    dbg_printf("OK: all PUL pins forced %s. Use N or S to restore normal mode.\r\n",
               high ? "HIGH" : "LOW");
}

static void cli_pulse_all_steppers(const char *args, bool active_low)
{
    long steps = 1000;
    long pulse_us = 500;
    motor_dir_t dir = MOTOR_DIR_FWD;

    cli_parse_optional_long(&args, &steps, 1000);

    const char *p = cli_skip_ws(args);
    if (*p != '\0' && (cli_upper(*p) == 'F' || cli_upper(*p) == 'R' || *p == '+' || *p == '-')) {
        if (!cli_parse_dir_token(&args, &dir)) {
            dbg_printf("ERR: direction must be F or R. Example: X 1000 F 500\r\n");
            return;
        }
    }

    cli_parse_optional_long(&args, &pulse_us, 500);

    if (steps < 1) steps = 1;
    if (steps > 10000) steps = 10000;
    if (pulse_us < 2) pulse_us = 2;
    if (pulse_us > 100000) pulse_us = 100000;

    lift_cancel_timeout();
    motor_stop_all();
    if (active_low) {
        motor_step_all_pulses_active_low(dir, (uint32_t)steps, (uint32_t)pulse_us);
    } else {
        motor_step_all_pulses(dir, (uint32_t)steps, (uint32_t)pulse_us);
    }
    motor_stop_all();

    dbg_printf("OK: all PUL pins pulsed %ld step(s), DIR=%s, pulse=%ld us, mode=%s\r\n",
               steps,
               (dir == MOTOR_DIR_FWD) ? "F" : "R",
               pulse_us,
               active_low ? "active-low" : "active-high");
}

static void cli_set_log(const char *args)
{
    long enable = 0;
    if (!cli_parse_long(&args, &enable)) {
        dbg_printf("LOG is %s. Use LOG 0 or LOG 1.\r\n",
                   terminal_log_enabled ? "ON" : "OFF");
        return;
    }
    terminal_log_enabled = (enable != 0);
    dbg_printf("OK: periodic terminal logs %s\r\n",
               terminal_log_enabled ? "ON" : "OFF");
}

static void load_cell_print_status(void)
{
    hx711_parallel_state_t state;
    hx711_parallel_get_state(&state);

    if (state.sample_count == 0u) {
        dbg_printf("LC WAIT samples=0 valid=0x%X ready_now=0x%X "
                   "last_timeout_ready=0x%X timeouts=%lu\r\n",
                   state.valid_mask,
                   state.ready_mask,
                   state.last_timeout_ready_mask,
                   (unsigned long)state.timeout_count);
        dbg_printf("LC wiring bits: ch0=PB0 ch1=PB5 ch2=PC7 ch3=PA8; shared SCK=PC0/A5\r\n");
        return;
    }

    uint32_t now = HAL_GetTick();
    int32_t delta[HX711_PARALLEL_CHANNEL_COUNT];
    int32_t span[HX711_PARALLEL_CHANNEL_COUNT];
    for (uint8_t i = 0u; i < HX711_PARALLEL_CHANNEL_COUNT; ++i) {
        delta[i] = state.raw[i] - state.zero[i];
        span[i] = state.max_delta[i] - state.min_delta[i];
    }

    dbg_printf("LC %s samples=%lu age=%lums valid=0x%X ready_now=0x%X "
               "timeouts=%lu last_timeout_ready=0x%X\r\n",
               (state.valid_mask == HX711_PARALLEL_ALL_MASK) ? "LIVE" : "STALE",
               (unsigned long)state.sample_count,
               (unsigned long)(now - state.last_update_ms),
               state.valid_mask,
               state.ready_mask,
               (unsigned long)state.timeout_count,
               state.last_timeout_ready_mask);
    dbg_printf("LC raw=[%ld %ld %ld %ld] delta=[%ld %ld %ld %ld]\r\n",
               (long)state.raw[0], (long)state.raw[1],
               (long)state.raw[2], (long)state.raw[3],
               (long)delta[0], (long)delta[1],
               (long)delta[2], (long)delta[3]);
    dbg_printf("LC span=[%ld %ld %ld %ld] (max-min counts since tare/reset)\r\n",
               (long)span[0], (long)span[1],
               (long)span[2], (long)span[3]);
}

static void cli_load_cell(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        load_cell_print_status();
        return;
    }

    if (cli_word_is(args, "TARE")) {
        if (hx711_parallel_tare()) {
            dbg_printf("OK: all four load-cell channels tared; no load should be applied\r\n");
            load_cell_print_status();
        } else {
            dbg_printf("ERR: cannot tare until all four HX711 channels are LIVE\r\n");
            load_cell_print_status();
        }
        return;
    }

    if (cli_word_is(args, "RESET")) {
        hx711_parallel_reset_span();
        dbg_printf("OK: load-cell observed spans reset\r\n");
        return;
    }

    if (cli_word_is(args, "STREAM")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long enable = 0;
        if (!cli_parse_long(&args, &enable)) {
            dbg_printf("LC stream is %s. Use LC STREAM 0 or LC STREAM 1.\r\n",
                       load_cell_stream_enabled ? "ON" : "OFF");
            return;
        }
        load_cell_stream_enabled = (enable != 0);
        load_cell_last_print_ms = 0u;
        dbg_printf("OK: load-cell stream %s\r\n",
                   load_cell_stream_enabled ? "ON" : "OFF");
        return;
    }

    dbg_printf("ERR: use LC STATUS, LC TARE, LC STREAM 0|1, or LC RESET\r\n");
}

static const char *switch_pull_name(void)
{
    if (switch_pull_mode == GPIO_PULLUP) return "UP";
    if (switch_pull_mode == GPIO_PULLDOWN) return "DOWN";
    return "NONE";
}

void switch_configure_inputs(uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SWITCH_1_GPIO_PIN | SWITCH_2_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    switch_pull_mode = pull;
}

static void switch_print_status(void)
{
    unsigned pc6 = (HAL_GPIO_ReadPin(SWITCH_1_GPIO_PORT, SWITCH_1_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    unsigned pc9 = (HAL_GPIO_ReadPin(SWITCH_2_GPIO_PORT, SWITCH_2_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    dbg_printf("SW PC6=%u(%s) PC9=%u(%s) pull=%s\r\n",
               pc6, pc6 ? "HIGH" : "LOW",
               pc9, pc9 ? "HIGH" : "LOW",
               switch_pull_name());
}

static void cli_switch(const char *args)
{
    args = cli_skip_ws(args);

    if (*args == '\0' || cli_word_is(args, "STATUS")) {
        switch_print_status();
        return;
    }

    if (cli_word_is(args, "STREAM")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        long enable = 0;
        if (!cli_parse_long(&args, &enable)) {
            dbg_printf("SW stream is %s. Use SW STREAM 0 or SW STREAM 1.\r\n",
                       switch_stream_enabled ? "ON" : "OFF");
            return;
        }
        switch_stream_enabled = (enable != 0);
        switch_last_print_ms = 0u;
        dbg_printf("OK: switch stream %s\r\n",
                   switch_stream_enabled ? "ON" : "OFF");
        return;
    }

    if (cli_word_is(args, "PULL")) {
        args = cli_skip_alpha(cli_skip_ws(args));
        args = cli_skip_ws(args);
        if (cli_word_is(args, "UP")) {
            switch_configure_inputs(GPIO_PULLUP);
        } else if (cli_word_is(args, "DOWN")) {
            switch_configure_inputs(GPIO_PULLDOWN);
        } else if (cli_word_is(args, "NONE")) {
            switch_configure_inputs(GPIO_NOPULL);
        } else {
            dbg_printf("ERR: use SW PULL NONE, SW PULL UP, or SW PULL DOWN\r\n");
            return;
        }
        dbg_printf("OK: PC6/PC9 input pull=%s\r\n", switch_pull_name());
        switch_print_status();
        return;
    }

    dbg_printf("ERR: use SW STATUS, SW STREAM 0|1, or SW PULL NONE|UP|DOWN\r\n");
}

/* ---- CLI line dispatcher --------------------------------------------------
 * One line in, one handler called. The first character selects the family;
 * multi-letter words (LIFT, LAC, LC, LOG, SW, ARM, DISARM) are matched with
 * cli_word_is(). All handlers above are self-contained — this function has
 * no logic beyond routing.                                                   */
static void cli_apply_line(const char *line)
{
    const char *p = cli_skip_ws(line);
    if (*p == '\0') return;

    if (cli_word_is(p, "DISARM")) {
        drive_disarm_all();
        dbg_printf("OK: DISARMED; all motion outputs stopped\r\n");
        return;
    }

    if (cli_word_is(p, "ARM")) {
        drive_stop_motors_preserve_lac();
        drive_set_armed(true);
        dbg_printf("OK: ARMED; motor outputs stopped, LAC targets preserved\r\n");
        return;
    }

    char cmd = cli_upper(*p);

    if (cmd == '?' || cmd == 'H') {
        cli_print_help();
        return;
    }

    if (cli_word_is(p, "TOF") || cli_word_is(p, "TOF STATUS")) {
        direct_tof_print_physical_right(HAL_GetTick());
        return;
    }

    if (!drive_is_armed()) {
        if (cli_word_is(p, "LAC STATUS")) {
            cli_lac(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "SW") || cli_word_is(p, "LC") ||
            cli_word_is(p, "LOG")) {
            dbg_printf("LOCKED: diagnostic command available after ARM; type H for help\r\n");
            return;
        }
        disarmed_reject_count++;
        dbg_printf("LOCKED: motion is DISARMED; send ARM deliberately\r\n");
        return;
    }

    if (cmd == 'V') {
        drive_apply_v(line);
        return;
    }

    if (cmd == 'M') {
        cli_run_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'G') {
        cli_run_stepper_group(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'D') {
        cli_run_stepper_pair(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'C') {
        cli_run_coordinated(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'J' || cmd == 'T') {
        cli_jog_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'S') {
        if (cli_word_is(p, "SW")) {
            cli_switch(cli_skip_cmd_word(p));
            return;
        }
        cli_stop_stepper(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'X') {
        cli_pulse_all_steppers(cli_skip_cmd_word(p), false);
        return;
    }

    if (cmd == 'Y') {
        cli_pulse_all_steppers(cli_skip_cmd_word(p), true);
        return;
    }

    if (cmd == 'P') {
        cli_force_pul_pins(cli_skip_cmd_word(p));
        return;
    }

    if (cmd == 'N') {
        lift_cancel_timeout();
        motor_release_all_pul_timer();
        dbg_printf("OK: PUL pins restored to normal timer mode\r\n");
        return;
    }

    if (cmd == 'A') {
        motor_prepare_run();
        dbg_printf("OK: all steppers stopped\r\n");
        return;
    }

    if (cmd == 'L') {
        if (cli_word_is(p, "LIFT")) {
            cli_run_lift(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "LAC")) {
            cli_lac(cli_skip_cmd_word(p));
            return;
        }
        if (cli_word_is(p, "LC")) {
            cli_load_cell(cli_skip_cmd_word(p));
            return;
        }
        cli_set_log(cli_skip_cmd_word(p));
        return;
    }

    dbg_printf("ERR: unknown command '%c'. Type H for help.\r\n", cmd);


/* ===========================================================================
 *                     Public API: USART2 ISR hooks + service
 * ========================================================================= */

/* Clear line-assembly state, flush any stale byte sitting in the USART2 data
 * register, enable the NVIC line, arm single-byte interrupt RX, and print the
 * boot banner so a human on the VCP sees the terminal come up. */
void cli_terminal_init(void)
{
#if DIRECT_TOF_TEST_MODE
    dbg_printf("\r\n==== DIRECT ToF SENSOR FEED (read-only) ====\r\n");
    dbg_printf(">> Motion commands are disabled in this diagnostic build.\r\n");
#else
    cmd_idx = 0u;
    cmd_ready = false;
    usart2_rx_count = 0u;
    usart2_line_count = 0u;
    disarmed_reject_count = 0u;
    volatile uint32_t uart2_flush = huart2.Instance->SR;
    uart2_flush = huart2.Instance->DR;
    (void)uart2_flush;
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
    dbg_printf("\r\n==== STM32 motor terminal ready ====\r\n");
#if Y_LIFT_TEST_MODE
    dbg_printf(">> Bounded mode: only LIFT UP/DOWN and S/A stop commands accepted.\r\n");
#else
    dbg_printf(">> Manual stepper terminal on USART2 @ 115200. Type H for help.\r\n");
    dbg_printf(">> Mac tester uses M for continuous, J for exact step pulses, S to stop.\r\n");
    dbg_printf(">> Existing ROS drive command still works: V <left_rad_s> <right_rad_s>\\n\r\n");
    dbg_printf(">>   e.g. 'V 1.0 1.0' (fwd), 'V -1.908 1.908' (turn), 'V 0 0' (stop)\r\n");
    dbg_printf(">>   no valid V line for %u ms -> both wheels auto-stop.\r\n",
               (unsigned)DRIVE_CMD_TIMEOUT_MS);
    dbg_printf(">> BOOT STATE: DISARMED. Send ARM before any motion command.\r\n");
#endif
#endif
}

void cli_handle_usart2_rx_cplt(void)
{
    usart2_rx_count++;
    if (!cmd_ready) {
        char c = (char)usart2_rx_byte;
        if (c == '\n' || c == '\r') {
            cmd_buf[cmd_idx] = '\0';
            if (cmd_idx > 0) {
                cmd_ready = true;
                usart2_line_count++;
            }
            cmd_idx = 0;
        } else if (cmd_idx < CMD_BUF_SIZE - 1) {
            cmd_buf[cmd_idx++] = c;
        }
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
}

void cli_handle_usart2_error(void)
{
    cmd_idx = 0u;
    cmd_ready = false;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
}

bool cli_terminal_log_enabled(void) { return terminal_log_enabled; }

void cli_service(uint32_t now_ms)
{
    if (lift_timed_active && (now_ms - lift_started_ms) >= lift_duration_ms) {
        lift_stop_outputs();
        dbg_printf("OK: LIFT complete -> M2/M5 stopped\r\n");
    }

    if (load_cell_stream_enabled &&
        (now_ms - load_cell_last_print_ms) >= LOAD_CELL_PRINT_PERIOD_MS) {
        load_cell_last_print_ms = now_ms;
        load_cell_print_status();
    }

    if (switch_stream_enabled &&
        (now_ms - switch_last_print_ms) >= SWITCH_PRINT_PERIOD_MS) {
        switch_last_print_ms = now_ms;
        switch_print_status();
    }

    if (!cmd_ready) return;

    char line[CMD_BUF_SIZE];
    strncpy(line, (const char *)cmd_buf, sizeof line);
    line[sizeof line - 1] = '\0';
    cmd_ready = false;

#if Y_LIFT_TEST_MODE
    const char *lift_line = cli_skip_ws(line);
    if (cli_word_is(lift_line, "LIFT")) {
        cli_run_lift(lift_line + 4);
    } else if (*lift_line == 'S' || *lift_line == 's' ||
               *lift_line == 'A' || *lift_line == 'a') {
        lift_stop_outputs();
        motor_stop_all();
        dbg_printf("OK: bounded Y lift stopped\r\n");
    } else {
        motor_stop_all();
        dbg_printf("ERR: bounded mode accepts only LIFT UP/DOWN or S/A\r\n");
    }
#else
    cli_apply_line(line);
#endif
}
