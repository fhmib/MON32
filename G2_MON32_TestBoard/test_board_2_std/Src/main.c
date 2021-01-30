/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "cmd.h"
#include "function.h"
#include "ring_buffer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VER "146#C120_MON32_testboard_1.0.1"
#define CMD_LENGTH 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t terminal_buf[2048];
uint8_t communication_buf[TRANS_MAX_LENGTH];
uint32_t tim_counter = 0;
uint32_t tim_counter_max = 1;
uint32_t tim2_counter = 0;

extern usart_tr_stu *usart1_tr;
uint8_t uart1_data;
uint8_t uart1_irq_sel;
uint8_t print_trans_data;

uint8_t pro_ctl_c122 = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t ch, r_flag = 0;
  uint8_t cmd[CMD_LENGTH];
  uint32_t cmd_len = 0;

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
  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  print_trans_data = 0;
  CLEAR_BIT(TERMINAL_UART.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&TERMINAL_UART);
  rb_init(usart1_tr->rb, USART_RX_BUF_SIZE, usart1_tr->rx_buf);
  rb_init(usart1_tr->wb, USART_TX_BUF_SIZE, usart1_tr->tx_buf);
  uart1_irq_sel = 1;
  __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);

  HAL_GPIO_WritePin(SW1_MODE_SEL_GPIO_Port, SW1_MODE_SEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW2_MODE_SEL_GPIO_Port, SW2_MODE_SEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW1_STROBE_GPIO_Port, SW1_STROBE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW2_STROBE_GPIO_Port, SW2_STROBE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW1_BLOCK_GPIO_Port, SW1_BLOCK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PRO_DIS_N_GPIO_Port, PRO_DIS_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PRO_CTL_GPIO_Port, PRO_CTL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PRO_CTL_2_GPIO_Port, PRO_CTL_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HARD_RESET_GPIO_Port, HARD_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MASTER_RESET_GPIO_Port, MASTER_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

  PRINT("\r\nCOM is ready\r\n");
  PRINT("Firmware Version: " VER "\r\n");
  PRINT(">");
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (serial_available() > 0) {
      ch = serial_read();

      if (ch == '\r' || ch == '\n') {
        if (ch == '\r') {
          r_flag = 1;
        } else {
          if (r_flag) {
            r_flag = 0;
            continue;
          }
        }
        PRINT("\r\n");
        cmd[cmd_len] = 0;
        process_cmd((char*)cmd);
        PRINT(">");
        cmd_len = 0;
      } else if (ch == '\b') {
        if (cmd_len > 0) {
          HAL_UART_Transmit(&TERMINAL_UART, &ch, 1, 0xFFFFFFFF);
          PRINT(" ");
          HAL_UART_Transmit(&TERMINAL_UART, &ch, 1, 0xFFFFFFFF);
          --cmd_len;
        } 
        r_flag = 0;
      } else {
        if (cmd_len < CMD_LENGTH - 1)
          cmd[cmd_len++] = ch;
        HAL_UART_Transmit(&TERMINAL_UART, &ch, 1, 0xFFFFFFFF);
        r_flag = 0;
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
console_cmd cmdlist[] = {
  {"poweron", cmd_power, "Command about power", "poweron", "poweron"},
  {"poweroff", cmd_power, "Command about power", "poweroff", "poweroff"},
  {"sn", cmd_get_sn, "Get Serial Number", "sn", "sn"},
  {"mdate", cmd_get_date, "Get Menufacture Date", "date", "date"},
  {"pn", cmd_get_pn, "Get Part Number", "pn", "pn"},
  {"vendor", cmd_get_vendor, "Get Vendor", "vendor", "vendor"},
  {"version", cmd_version, "Get version", "version", "version"},
  {"txsw", cmd_switch, "Command about tx switch", "txsw write <in> to <out> | read", "txsw write 1 to 32 | txsw read"},
  {"rxsw", cmd_switch, "Command about rx switch", "rxsw write <channel> | read", "rxsw write 32 | rxsw read"},
  {"temp", cmd_temp, "Get device temperature", "temp", "temp"},
  {"il", cmd_IL, "Get Optical Switch Insertion Loss", "il <0/1>", "il 0"},
  {"tosa", cmd_tosa, "Command about tosa", "tosa set <high> <low> | get | thr <idx>", "tosa set 0 -4 | tosa get | tosa thr 0"},
  {"rxpd", cmd_rx_pd_cali, "Get RX PD calibration data", "rxpd <idx>", "rxpd 0"},
  {"tappd", cmd_tap_pd_cali, "Get TAP PD calibration data", "tappd <idx>", "tappd 0"},
  {"voltage", cmd_voltage, "Get device voltage", "voltage get", "voltage get"},
  {"loopback", cmd_loopback, "Command about loopback check", "loopback check | status", "loopback check | status"},
  {"alarm", cmd_alarm, "Get alarm", "alarm", "alarm"},
  {"modulation", cmd_modulation, "Command about tosa", "modulation on | off | mode", "modulation on | off | mode"},
  {"status", cmd_device_status, "Get device status", "status", "status"},
  {"history_alarm", cmd_history_alarm, "Get history alarm", "history_alarm", "history_alarm"},
  {"upgrade", cmd_upgrade, "Command about upgrade", "upgrade init | file | run", "upgrade init | file | run"},
  {"reset", cmd_reset, "Reset the device", "reset <soft | hard | master>", "reset soft"},
  {"date", cmd_time, "Set/Get module time", "date get | set y m d h m s", "date get | set 2020 7 29 15 55 50"},
  {"performance", cmd_performance, "Get performance", "performance [id1] [id2] ... | all", "performance 1 5 | performance all"},
  {"threshold", cmd_threshold, "Set/Get threshold", "threshold get <id> | set <id> <low> <high>", "threshold get 0 | set 0 -100 800"},
  {"log", cmd_log, "Command about log", "log size <type> | get <type> <offset> <length>", "log size 1 | get 1 0x100 128"},
  {"tosa_status", cmd_tosa_status, "Get tosa status", "tosa_status", "tosa_status"},
  {"waveset", cmd_set_tosa_wavelength, "Set the wavelength", "waveset 0(C98)/1(C122)", "waveset 0"},
  {"debug", cmd_for_debug, "Command for debug", "debug", "debug"},
  {"help", cmd_help, "Help information", "help | <cmd> help", "help | upgrade help"},
};
const uint32_t cmd_count = sizeof(cmdlist) / sizeof(cmdlist[0]);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == TERMINAL_UART.Instance) {
    if (rb_is_full(usart1_tr->rb)) {
        rb_remove(usart1_tr->rb);
    }
    rb_insert(usart1_tr->rb, uart1_data);

    HAL_UART_Receive_IT(&TERMINAL_UART, &uart1_data, 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    if (++tim_counter >= tim_counter_max) {
      if (pro_ctl_c122 == 0) {
        HAL_GPIO_TogglePin(PRO_CTL_GPIO_Port, PRO_CTL_Pin);
      } else {
        HAL_GPIO_TogglePin(PRO_CTL_2_GPIO_Port, PRO_CTL_2_Pin);
      }
      tim_counter = 0;
    }
  } else if (htim->Instance == TIM2) {
    tim2_counter++;
  }
}
#if 0
console_cmd cmdlist[] = {
  {"status", cmd_device_status, "Get device status", "status", "status"},
  {"voltage", cmd_voltage, "Get device voltage", "voltage get | get thr", "voltage get | get thr"},
  {"date", cmd_time, "Set/Get module time", "date get | set y m d h m s", "date get | set 2020 7 29 15 55 50"},
  {"log", cmd_log, "Command about log", "log packets | get <packets> <number>", "log packets | get 123 2"},
  {"switch", cmd_switch, "Command about switch", "switch write <chan> | read", "switch write 31 | read"},
  {"il", cmd_IL, "Get Optical Switch Insertion Loss", "il", "il"},
  {"maintain", cmd_maintain, "Get maintain information", "maintain", "maintain"},
  {"debug", cmd_for_debug, "Command for debug", "debug", "debug"},
  {"help", cmd_help, "Help information", "help | <cmd> help", "help | upgrade help"},
};
const uint32_t cmd_count = sizeof(cmdlist) / sizeof(cmdlist[0]);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
  }
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
