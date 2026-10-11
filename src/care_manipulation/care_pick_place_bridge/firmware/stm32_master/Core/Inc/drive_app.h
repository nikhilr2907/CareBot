#ifndef DRIVE_APP_H
#define DRIVE_APP_H

#include <stdbool.h>
#include <stdint.h>

/* Watchdog window: no fresh V line within this many ms triggers an auto-stop.
 * Published here so the boot banner can quote the exact value. */
#define DRIVE_CMD_TIMEOUT_MS 500u

/* ROS /cmd_vel wheel driver.
 *
 * The Raspberry Pi (via USART2) streams one ASCII line per command:
 *      V <left_rad_s> <right_rad_s>\n
 * drive_apply_v() parses that line and pushes each wheel's RPM + direction to
 * the BLD-515C BLDC driver over USART1 RS-485. A per-wheel cache suppresses
 * writes while the command stays the same, and a watchdog stops the wheels if
 * no fresh V line arrives within DRIVE_CMD_TIMEOUT_MS.
 *
 * ARM semantics: on boot and after DISARM the robot refuses motion commands;
 * ARM clears the stop state and lets the CLI route V lines through. */

/* Parse "V <left> <right>" and drive both wheels. Rejects malformed lines. */
void drive_apply_v(const char *line);

/* Force both wheels stopped and reset the per-wheel cache. */
void drive_stop_all(void);

/* Stop every motor output without touching the LAC position commands. ARM
 * uses this so re-enabling drive/stepper commands can never reposition a
 * linear actuator. */
void drive_stop_motors_preserve_lac(void);

/* Stop motors, zero both LAC outputs, and clear the armed flag. */
void drive_disarm_all(void);

/* Main-loop watchdog -- stops the wheels if no fresh V line arrived within
 * DRIVE_CMD_TIMEOUT_MS. Prints a timeout banner once per edge. */
void drive_timeout_service(uint32_t now_ms);

/* Armed flag -- read by the CLI (which rejects motion commands when the robot
 * is not armed) and set by the ARM / DISARM commands. */
bool drive_is_armed(void);
void drive_set_armed(bool armed);

#endif /* DRIVE_APP_H */
