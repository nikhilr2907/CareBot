/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "motors.h"
#include "bld515c.h"
#include "bno055.h"
#include "hx711_parallel.h"
#include "lac_actuators.h"
#include "cli_app.h"
#include "debug.h"
#include "direct_tof_app.h"
#include "drive_app.h"
#include "sensor_node_app.h"
#include "vl53l0x_pair.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Sensor-node ingest   -> Core/Src/sensor_node_app.c
 * ROS /cmd_vel drive   -> Core/Src/drive_app.c
 * Manual CLI terminal  -> Core/Src/cli_app.c (owns the USART2 RX ISR state,
 *                         the lift timeout, and the LC / SW streaming prints) */

/* BNO055 IMU state. Polled from the main loop when the poll period has
 * elapsed. imu_ok is set by bno055_init(); if false, the poll is skipped
 * so a missing IMU doesn't generate hundreds of failed I2C transactions
 * per second. Print is rate-limited separately so logs don't flood. */
#define IMU_POLL_PERIOD_MS  10u    /* 10 ms = 100 Hz target               */
#define IMU_PRINT_PERIOD_MS 500u   /* print one summary line every 500 ms */
static bno055_euler_t imu_euler         = {0};
static bno055_calib_t imu_calib         = {0};
static bool           imu_ok            = false;
static uint32_t       imu_read_count    = 0;
static uint32_t       imu_err_count     = 0;
static uint32_t       imu_last_poll_ms  = 0;
static uint32_t       imu_last_print_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
void MX_I2C2_Init(void);  /* non-static: used by direct_tof_app.c for bus recovery */
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
/* Direct VL53L0X ToF application code lives in Core/Src/direct_tof_app.c. */

/* ESP32 RS485 sensor-node ingest lives in Core/Src/sensor_node_app.c. */


/* ROS /cmd_vel drive interface lives in Core/Src/drive_app.c. */



/* Manual CLI terminal lives in Core/Src/cli_app.c. */

/* ===========================================================================
 *                   SECTION  Boot bring-up helpers
 * ---------------------------------------------------------------------------
 * Each function here is called exactly once from main() during startup, in
 * the order they appear. Keeping them separate makes the boot sequence read
 * top-to-bottom in main() as a list of named phases.
 * ========================================================================= */

/* BLD-515C BLDC bring-up test on USART1 / RS-485. Round-trips a read of the
 * device-type register to prove the bus/baud/address are correct before any
 * spin command. If the probe gets no reply, suspect baud (driver should be
 * 9600), slave address, or A/B swap. Set BLD515C_DE_RE_PORT/PIN in bld515c.h
 * (PB6 is free) if your converter needs manual DE/RE.                        */
static void bldc_bringup_probe(void)
{
    const uint8_t bldc_addr = BLD515C_DEFAULT_ADDR;

    bld515c_init(&huart1);
    drive_stop_all();
    dbg_printf(">> boot interlock: STOP sent to both drive addresses before probe\r\n");
    dbg_printf("\r\n>> BLD-515C bring-up on USART1 @ 9600 (slave 0x%02X)...\r\n", bldc_addr);

    /* Raw probe of the read-only device-type reg (0x8000) -- dumps every byte
     * that comes back for human eyes. A correct reply contains the sequence
     * 01 03 02 01 92 <crc> <crc> (possibly preceded by an echo of our own
     * request on a half-duplex converter). */
    uint8_t raw[24];
    int n = bld515c_probe(bldc_addr, BLD515C_REG_DEVTYPE, raw, sizeof raw, 0);
    dbg_printf(">> probe read 0x8000: %d byte(s) back:", n);
    for (int i = 0; i < n; ++i) dbg_printf(" %02X", raw[i]);
    dbg_printf("\r\n");

    /* Authoritative check: read_regs() scans the capture for a CRC-valid
     * device-type response (0x0192), skipping any leading bus echo. */
    uint16_t devtype = 0;
    bool online = bld515c_read_regs(bldc_addr, BLD515C_REG_DEVTYPE, &devtype, 1);

    if (!online) {
        /* MCU self-check. The byte-by-byte loopback is reliable (unlike the
         * burst probe above). It only means something when PA9<->PA10 are
         * jumpered together with the converter UNPLUGGED; with the converter
         * wired up this is expected to FAIL and says nothing about the MCU. */
        bool loop = bld515c_selftest_loopback();
        dbg_printf(">> USART1 loopback self-test: %s  (valid only with PA9<->PA10\r\n"
                   "   jumpered & converter unplugged)\r\n",
                   loop ? "PASS -> USART1 TX+RX are healthy; you do NOT need another UART"
                        : "FAIL");

        if (n == 0) {
            dbg_printf(">> SILENT BUS: nothing came back. If NOT in loopback, check\r\n"
                       "   converter power, A/B polarity, GND, and TTL TX/RX orientation\r\n"
                       "   (PA9->conv RXD, PA10->conv TXD).\r\n");
        } else if (raw[0] != bldc_addr) {
            dbg_printf(">> Got bytes but the first isn't our address (0x%02X) -> A/B\r\n"
                       "   likely swapped, baud mismatch (driver not at 9600), or wrong\r\n"
                       "   slave addr.\r\n", bldc_addr);
        } else if (n >= 2 && (raw[1] & 0x80u)) {
            dbg_printf(">> Driver replied with a Modbus EXCEPTION (code 0x%02X) -- it's\r\n"
                       "   alive and addressed correctly; the register/function was\r\n"
                       "   rejected. Wiring & baud are GOOD.\r\n", n >= 3 ? raw[2] : 0);
        } else {
            dbg_printf(">> Got %d byte(s) but no CRC-valid frame -> partial/garbled reply.\r\n"
                       "   Suspect baud mismatch (driver not exactly 9600) or noise on A/B.\r\n", n);
        }
        dbg_printf(">> Skipping spin-up until the link is verified.\r\n");
    } else {
        dbg_printf(">> BLD-515C ONLINE. device type 0x8000 = 0x%04X\r\n", devtype);

        /* Boot-time motor movement test intentionally disabled. Keep the
         * bus probe above, but never run canned wheel commands at startup. */
        drive_stop_all();
        dbg_printf(">> boot motor movement test disabled; wheels commanded stopped.\r\n");
        dbg_printf(">> now listening for live V commands.\r\n");
    }
}


/* Direct VL53L0X pair bring-up and per-tick service live in direct_tof_app.c
 * and are called via direct_tof_bringup() / direct_tof_service(). */

/* ---- Steppers + linear actuators bring-up ---------------------------------
 * The three test-mode branches all end with safe state: DIRECT_TOF skips
 * motion entirely, Y_LIFT only arms steppers (M2/M5 lift), full-mode brings
 * up both the stepper array and the LAC PWM outputs.                         */
static void actuators_bringup(void)
{
#if DIRECT_TOF_TEST_MODE
    dbg_printf("\r\n>> DIRECT TOF TEST MODE: stepper, BLDC and LAC movement disabled\r\n");
#elif Y_LIFT_TEST_MODE
    motor_init();
    motor_stop_all();
    dbg_printf("\r\n>> Y LIFT TEST MODE: only timed M2/M5 lift commands enabled\r\n");
    dbg_printf(">> M1/M3/M4/M6/M7, BLDC and LAC movement disabled\r\n");
#else
    motor_init();
    motor_stop_all();
    if (!lac_actuators_init(&htim2)) {
        Error_Handler();
    }
#endif
}

/* ---- BNO055 IMU bring-up on I2C2 ------------------------------------------
 * Blocks ~750 ms while the chip boots; records online/offline into imu_ok
 * for the main loop status prints.                                           */
static void imu_bringup(void)
{
#if DIRECT_TOF_TEST_MODE || Y_LIFT_TEST_MODE
    dbg_printf("\r\n>> BNO055 skipped so I2C2 carries only ToF diagnostic traffic\r\n");
#else
    dbg_printf("\r\n>> Bringing up BNO055 IMU...\r\n");
    imu_ok = bno055_init();
    dbg_printf(">> BNO055 %s\r\n", imu_ok ? "READY (NDOF, polled @ ~100 Hz)"
                                          : "NOT FOUND (check PB3/PB10/PC3 wiring & pullups)");
#endif
}

/* ---- Static peripheral-topology banner ------------------------------------
 * Pure informational prints: LAC pinout + mode, ESP32 sensor-node framing on
 * USART3, and the USART2 role note. No hardware touched.                     */
static void print_actuator_banner(void)
{
#if Y_LIFT_TEST_MODE
    dbg_printf(">> LAC actuators disabled during bounded Y lift test\r\n");
#else
    dbg_printf(">> LAC actuators: LEFT=A0/PA0, RIGHT=A1/PA1, "
#if LAC_RC_TEST_MODE
               "temporary RC 50Hz (1-2ms); startup command=HOME\r\n");
#else
               "3.3V 1kHz PWM (1-99%% duty); startup command=HOME\r\n");
#endif
#endif
    dbg_printf(">> ESP32 sensor nodes on USART3/RS485 @ 115200\r\n");
    dbg_printf(">> cmd=0x01: 4x VL53L0X + 1x ultrasonic; cmd=0x02: 3x VL53L5CX grids + fans\r\n");
    dbg_printf(">> USART2 is terminal/Raspberry Pi only; sensor bytes stay on RS485 UARTs.\r\n");
}


/* cli_terminal_init() lives in Core/Src/cli_app.c (declared in cli_app.h). */

/* USER CODE END 0 */

/* ===========================================================================
 *                   SECTION  main() — init sequence + idle loop
 * ---------------------------------------------------------------------------
 * CubeMX fills in the HAL init and USER CODE markers. All application-level
 * bringup is routed through the helpers above, so the body of main() reads
 * as a flat list of named phases followed by the forever loop.
 * ========================================================================= */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
 
  sensor_node_start_dma();
  actuators_bringup();
  hx711_parallel_init();
  switch_configure_inputs(GPIO_PULLUP);
  dbg_printf(">> HX711 load cells: SCK=PC0/A5, DOUT=[PB0 PB5 PC7 PA8], waiting for all four\r\n");
  dbg_printf(">> limit switches: PC6 and PC9 active-low inputs, pull=UP; use SW STATUS\r\n");

#if Y_LIFT_TEST_MODE
  dbg_printf("\r\n>> Direct ToF initialization skipped during bounded Y lift test\r\n");
#else
  direct_tof_bringup();
#endif
  imu_bringup();
  print_actuator_banner();

#if !DIRECT_TOF_TEST_MODE && !Y_LIFT_TEST_MODE
  bldc_bringup_probe();
#else
  dbg_printf(">> BLDC probe skipped in direct ToF test mode\r\n");
#endif

  cli_terminal_init();


  uint32_t sensor_last_print_ms    = 0u;
  uint32_t sensor_last_valid_frames = 0u;
  uint32_t sensor_last_rx_bytes     = 0u;
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    sensor_node_service_usart3();
    uint32_t now_ms = HAL_GetTick();
#if !Y_LIFT_TEST_MODE
    vl53l0x_pair_service(now_ms);
#endif
    direct_tof_service(now_ms);
    if (motor_coordinated_service()) {
        dbg_printf("OK: coordinated step run complete -> selected motors stopped\r\n");
    }
    hx711_parallel_service(now_ms);
    cli_service(now_ms);
    if (cli_terminal_log_enabled()) {
        sensor_node_emit_status(HAL_GetTick(),
                                &sensor_last_print_ms,
                                &sensor_last_valid_frames,
                                &sensor_last_rx_bytes);
    }

    /* --- BNO055 poll. Target 100 Hz (10 ms period). Note: while the motor
     * test loop below is running its HAL_Delay()s, this block only fires
     * between cycles -- so during the test you'll see less than 100 Hz.
     * Once you remove the test code and re-enable the CLI, this becomes
     * the real 100 Hz polling path with no changes. */
    if (imu_ok && (HAL_GetTick() - imu_last_poll_ms) >= IMU_POLL_PERIOD_MS) {
        imu_last_poll_ms = HAL_GetTick();
        bool e_ok = bno055_read_euler(&imu_euler);
        bool c_ok = bno055_read_calib(&imu_calib);
        if (e_ok && c_ok) imu_read_count++;
        else              imu_err_count++;

        if (cli_terminal_log_enabled() &&
            (HAL_GetTick() - imu_last_print_ms) >= IMU_PRINT_PERIOD_MS) {
            imu_last_print_ms = HAL_GetTick();
            dbg_printf("IMU H=%.1f R=%.1f P=%.1f  calib S/G/A/M=%u/%u/%u/%u  reads=%lu errs=%lu\r\n",
                       (double)imu_euler.heading_deg,
                       (double)imu_euler.roll_deg,
                       (double)imu_euler.pitch_deg,
                       imu_calib.sys, imu_calib.gyro,
                       imu_calib.accel, imu_calib.mag,
                       (unsigned long)imu_read_count,
                       (unsigned long)imu_err_count);
        }
    }

    /* --- Command watchdog. If the Pi stops streaming V lines (crashed bridge,
     * unplugged cable), stop both wheels so the robot can't run away. The Pi
     * also sends "V 0 0" on its own 0.5 s timeout; this is the backstop for
     * when even that doesn't arrive. */
#if !DIRECT_TOF_TEST_MODE
    drive_timeout_service(HAL_GetTick());
#endif
  }
  /* USER CODE END 3 */
}

/* ===========================================================================
 *                   SECTION  CubeMX-generated HAL peripheral init
 * ---------------------------------------------------------------------------
 * Everything below is regenerated by STM32CubeMX from stm32f446LowerLevel.ioc.
 * Hand edits must stay inside USER CODE BEGIN/END pairs or they will be wiped.
 * ========================================================================= */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
#if DIRECT_TOF_LONG_CABLE_MODE
  /* The direct VL53L0X modules are approximately two metres from the MCU.
   * Slow the diagnostic bus to give the long cable substantially more rise
   * and settling time. */
  hi2c2.Init.ClockSpeed = 10000;
#else
  hi2c2.Init.ClockSpeed = 100000;
#endif
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
 
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
 
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
 
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* APB1 timer clock is 84 MHz. Divide to a 1 MHz counter. */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
#if LAC_RC_TEST_MODE
  htim2.Init.Period = 19999; /* 50 Hz RC frame */
#else
  htim2.Init.Period = 999;   /* 1 kHz VC PWM */
#endif
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
 
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
 
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
 
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
 
  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */
 
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
 
  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
 
  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */
 
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
 
  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
 
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
 
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* BLD-515C RS-485 link runs at 9600 baud. Override the CubeMX default
   * 115200 and re-init. Survives Generate Code because this is inside a USER
   * CODE block. Cleaner alternative: set USART1 baud to 9600 in the IOC,
   * regenerate, and delete this block. */
  huart1.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
 
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
 
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
 
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USART3 is no longer used for direct DYP-A02 polling. It remains a
   * 115200-baud RS485 fallback input for the ESP32 sensor node. */
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
 
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Hold both VL53L0X sensors in shutdown until they can be addressed one at
   * a time without colliding at the shared factory address. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC1 PC2 PC3
                           PC4 PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure D13/PA5 as the direct ToF XSHUT output. The onboard LD2 LED is
   * also attached to PA5 and will harmlessly mirror this line. */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure PD2 as the second direct ToF XSHUT output. */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB14 PB15
                           PB4 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
 
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===========================================================================
 *                   SECTION  HAL ISR callbacks + fatal handlers
 * ---------------------------------------------------------------------------
 * HAL_UART_* callbacks fire from the USARTx IRQ. They must stay short and
 * MUST NOT call dbg_printf (which blocks on TX). Error_Handler and
 * assert_failed are terminal panics — the status LED on PA5 blinks via a
 * fast toggle loop.
 * ========================================================================= */

/* USART2 uses single-byte interrupt RX for the test command interface. The
 * actual byte-accumulation lives in cli_app.c; we just route the HAL callback
 * to the right owner so the ISR path stays short. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        cli_handle_usart2_rx_cplt();
    }
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }

/* On any UART error (overrun, framing, noise) HAL aborts the active transfer.
 * Each owner restarts its own transfer so we don't go deaf permanently. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        sensor_node_restart_dma();
    } else if (huart->Instance == USART2) {
        cli_handle_usart2_error();
    }
}
 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
