#ifndef CLI_APP_H
#define CLI_APP_H

#include <stdbool.h>
#include <stdint.h>

/* Y_LIFT_TEST_MODE restricts the CLI to the LIFT UP/DOWN + S/A commands and
 * disables unrelated bring-up paths. Published here so main.c sees the same
 * value cli_app.c compiles against. */
#define Y_LIFT_TEST_MODE 0

/* Manual CLI terminal on USART2.
 *
 * HAL_UART_RxCpltCallback(USART2) hands bytes to this module via
 * cli_handle_usart2_rx_cplt(); the module assembles '\n'-terminated lines in
 * its own buffer and dispatches them from cli_service(), which main() calls
 * every tick. cli_service() also owns the per-tick glue for the lift
 * timeout and the load-cell / switch streaming prints. */

/* Boot banner + USART2 single-byte interrupt RX arm. Call once from main(). */
void cli_terminal_init(void);

/* Per-tick service: lift timeout check, streaming prints, dispatch any
 * pending command line. Pass HAL_GetTick(). */
void cli_service(uint32_t now_ms);

/* HAL callback hooks. The HAL ISRs in main.c call these for the USART2
 * interrupt-RX path. */
void cli_handle_usart2_rx_cplt(void);
void cli_handle_usart2_error(void);

/* Configure the two limit-switch inputs. Called from main() during init,
 * with GPIO_PULLUP / GPIO_PULLDOWN / GPIO_NOPULL. */
void switch_configure_inputs(uint32_t pull);

/* True while LOG 1 is in effect -- IMU and sensor status prints throttle on
 * this flag so they do not flood the terminal while a user is driving. */
bool cli_terminal_log_enabled(void);

#endif /* CLI_APP_H */
