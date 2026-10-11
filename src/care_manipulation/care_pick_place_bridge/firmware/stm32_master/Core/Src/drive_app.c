/* ROS /cmd_vel drive interface
 * ---------------------------------------------------------------------------
 * The Pi streams "V <left_rad_s> <right_rad_s>" lines on USART2. This module
 * converts each wheel's signed rad/s into a BLD-515C RPM+direction and pushes
 * it over RS-485 on USART1. Steady-state cruising re-sends nothing (per-wheel
 * cache). A main-loop watchdog stops the wheels if no fresh V line arrives in
 * DRIVE_CMD_TIMEOUT_MS. */

#include "drive_app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "bld515c.h"
#include "debug.h"
#include "lac_actuators.h"
#include "main.h"
#include "motors.h"
#include "stm32f4xx_hal.h"

#define DRIVE_LEFT_ADDR      0x01u   /* BLD-515C slave id - LEFT wheel  */
#define DRIVE_RIGHT_ADDR     0x02u   /* BLD-515C slave id - RIGHT wheel */

/* Motor revolutions per WHEEL revolution (total gearbox reduction).
 * Drivetrain: F6B250-24GU-30S BLDC (3000 rpm rated) + 6GU20K 20:1 spur gearbox.
 * Combined with the 150 rpm motor minimum, this 20:1 ratio gives a usable
 * robot-speed band of ~0.047 m/s (min) .. ~0.94 m/s (max) at the 0.06 m wheel
 * radius. A /cmd_vel below ~0.047 m/s maps under the motor minimum, so the
 * wheel is held stopped (that's why 0.03 m/s did nothing -- it's ~95 motor
 * RPM, below 150). */
#define DRIVE_GEAR_RATIO     20.0f

/* Drive wheels are usually mirror-mounted, so one side must spin the opposite
 * way to move the robot "forward". Flip these (0/1) to match your mechanics. */
#define DRIVE_LEFT_INVERT    0
#define DRIVE_RIGHT_INVERT   1

#define DRIVE_MIN_RPM        150u     /* below this we just stop (mfr minimum)  */
#define DRIVE_MAX_RPM        3000u    /* safety clamp                           */
/* DRIVE_CMD_TIMEOUT_MS: published in drive_app.h so the boot banner can
 * quote it. No valid V line in that window -> both wheels auto-stop. */
#define RADS_TO_RPM          9.54929659f   /* 60 / (2*pi) */

typedef struct { uint16_t rpm; bool reverse; bool running; bool valid; } wheel_state_t;
static wheel_state_t drive_wheel[2] = {0};       /* [0]=left, [1]=right */
static uint32_t      drive_last_cmd_ms = 0u;
static bool          drive_timed_out   = true;   /* start safe (stopped) */
static bool          motion_armed      = false;  /* explicit ARM after boot */

/* Convert one signed wheel velocity (rad/s) to a BLD-515C command and send it,
 * skipping the write when nothing changed. `side` selects the cache slot. */
static bool drive_set_wheel(int side, uint8_t addr, float wheel_rad_s, int invert)
{
    float rpm_f   = wheel_rad_s * RADS_TO_RPM * DRIVE_GEAR_RATIO;
    bool  reverse = (rpm_f < 0.0f);
    if (invert) reverse = !reverse;
    long  mag = (long)(rpm_f < 0.0f ? -rpm_f : rpm_f);

    wheel_state_t *w = &drive_wheel[side];
    bool ok = true;

    if (mag < (long)DRIVE_MIN_RPM) {          /* too slow to run -> stop */
        if (!w->valid || w->running) ok = bld515c_stop(addr);
        w->running = false; w->rpm = 0; w->valid = true;
        return ok;
    }
    if (mag > (long)DRIVE_MAX_RPM) mag = (long)DRIVE_MAX_RPM;
    uint16_t rpm = (uint16_t)mag;

    if (!w->valid || w->rpm != rpm)
        ok = bld515c_set_speed_rpm(addr, rpm) && ok;
    if (!w->valid || !w->running || w->reverse != reverse)
        ok = bld515c_run(addr, reverse) && ok;

    w->rpm = rpm; w->reverse = reverse; w->running = true; w->valid = true;
    return ok;
}

void drive_stop_all(void)
{
    bld515c_stop(DRIVE_LEFT_ADDR);
    bld515c_stop(DRIVE_RIGHT_ADDR);
    drive_wheel[0] = (wheel_state_t){ 0, false, false, true };
    drive_wheel[1] = (wheel_state_t){ 0, false, false, true };
}

void drive_stop_motors_preserve_lac(void)
{
    motor_stop_all();
    drive_stop_all();
    drive_timed_out = true;
}

void drive_disarm_all(void)
{
    drive_stop_motors_preserve_lac();
    (void)lac_actuators_set_both_percent(0u);
    motion_armed = false;
}

void drive_apply_v(const char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line != 'V' && *line != 'v') {
        dbg_printf(">> ignore (no leading V): \"%s\"\r\n", line);
        return;
    }

    const char *p = line + 1;
    char *end;
    float left  = strtof(p, &end);
    if (end == p) { dbg_printf(">> bad cmd: missing left value\r\n");  return; }
    p = end;
    float right = strtof(p, &end);
    if (end == p) { dbg_printf(">> bad cmd: missing right value\r\n"); return; }

    bool okL = drive_set_wheel(0, DRIVE_LEFT_ADDR,  left,  DRIVE_LEFT_INVERT);
    bool okR = drive_set_wheel(1, DRIVE_RIGHT_ADDR, right, DRIVE_RIGHT_INVERT);

    drive_last_cmd_ms = HAL_GetTick();
    drive_timed_out   = false;

    /* Show the mapped motor RPM (pre-clamp) so a "received but too slow to
     * spin" command is obvious rather than looking like no response. */
    int lrpm = (int)(left  * RADS_TO_RPM * DRIVE_GEAR_RATIO);
    int rrpm = (int)(right * RADS_TO_RPM * DRIVE_GEAR_RATIO);
    dbg_printf("V L=%.3f R=%.3f rad/s -> map L=%d R=%d rpm | L:%s[%s] R:%s[%s]\r\n",
               (double)left, (double)right, lrpm, rrpm,
               drive_wheel[0].running ? (drive_wheel[0].reverse ? "REV" : "FWD") : "STOP",
               okL ? "OK" : "ERR",
               drive_wheel[1].running ? (drive_wheel[1].reverse ? "REV" : "FWD") : "STOP",
               okR ? "OK" : "ERR");

    if ((left  != 0.0f && !drive_wheel[0].running) ||
        (right != 0.0f && !drive_wheel[1].running)) {
        dbg_printf("   note: |rpm| < %u min -> wheel HELD STOPPED. Raise DRIVE_GEAR_RATIO\r\n"
                   "   to your gearbox, or command a higher speed.\r\n",
                   (unsigned)DRIVE_MIN_RPM);
    }
}

void drive_timeout_service(uint32_t now_ms)
{
    if (drive_timed_out ||
        (now_ms - drive_last_cmd_ms) <= DRIVE_CMD_TIMEOUT_MS) {
        return;
    }
    drive_stop_all();
    drive_timed_out = true;
    dbg_printf("!! cmd timeout (%u ms) -> wheels stopped\r\n",
               (unsigned)DRIVE_CMD_TIMEOUT_MS);
}

bool drive_is_armed(void)      { return motion_armed; }
void drive_set_armed(bool armed) { motion_armed = armed; }
