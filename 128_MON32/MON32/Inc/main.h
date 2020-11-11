/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "common.h"
#include "command.h"
#include "flash_if.h"
#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t reset_flag;
extern uint8_t flash_in_use;
extern RunTimeStatus run_status;

extern AlarmHistoryState history_alarm_status;

extern TosaCalData tosa_table[];
extern uint8_t tosa_table_count;
extern double tosa_power_high_max_thr;
extern double tosa_power_low_min_thr;

extern uint8_t device_busy;
extern uint8_t allow_tosa;

extern const uint32_t error_file_flash_addr[];
extern const uint32_t error_file_flash_end;
extern const uint8_t error_file_flash_count;
extern const uint32_t normal_file_flash_addr[];
extern const uint32_t normal_file_flash_end;
extern const uint8_t normal_file_flash_count;
extern LogFileState log_file_state;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define UART_PROCESS_PRIORITY     osPriorityRealtime6
#define WATCHDOG_PRIORITY         osPriorityISR
#define INTERRUPT_TASK_PRIORITY   osPriorityISR
#define CMD_PROCESS_PRIORITY      osPriorityHigh
#define LOG_MESSAGE_PRIORITY      osPriorityLow
#define MONITOR_PRIORITY          osPriorityAboveNormal
#define LAZER_MANAGER_PRIORITY    osPriorityHigh

#define WATCH_DOG_DELAY_TIME      500 // ms

// Reset Flags
#define BOR_RESET_BIT             (1UL << 0)
#define PIN_RESET_BIT             (1UL << 1)
#define POR_RESET_BIT             (1UL << 2)
#define SFT_RESET_BIT             (1UL << 3)
#define IWDG_RESET_BIT            (1UL << 4)
#define WWDG_RESET_BIT            (1UL << 5)
#define LPWR_RESET_BIT            (1UL << 6)
#define IS_RESETFLAG_SET(bit)     (reset_flag & bit)
#define SET_RESETFLAG(bit)        (reset_flag |= bit)

#define EEPROM_ADDR               (0x50 << 1)
#define ADC7828_ADDR              (0x48 << 1)

#define LOG_QUEUE_LENGTH          6
#define ERROR_LOG_FIRST_SECTOR          12
#define NORMAL_LOG_FIRST_SECTOR          19

#define ISR_QUEUE_LENGTH          6

#define LM_QUEUE_LENGTH           6

#define RUN_MAGIC                 0xA5A55A5A
#define UPGRADE_MAGIC             0xA5A55A5A
#define LOG_MAGIC                 0xA5A55A5A
#define ALARM_MAGIC               0xA5A55A5A
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MON32_Init(void);
void MON32_Init_Dev(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW2_MODE_SEL_Pin GPIO_PIN_3
#define SW2_MODE_SEL_GPIO_Port GPIOB
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI6_CS2_Pin GPIO_PIN_11
#define SPI6_CS2_GPIO_Port GPIOG
#define SPI6_CS1_Pin GPIO_PIN_10
#define SPI6_CS1_GPIO_Port GPIOG
#define SW2_D4_Pin GPIO_PIN_6
#define SW2_D4_GPIO_Port GPIOD
#define SW2_READY_Pin GPIO_PIN_0
#define SW2_READY_GPIO_Port GPIOD
#define SW2_D3_Pin GPIO_PIN_5
#define SW2_D3_GPIO_Port GPIOD
#define SW2_STROBE_Pin GPIO_PIN_1
#define SW2_STROBE_GPIO_Port GPIOD
#define SW2_STROBE_EXTI_IRQn EXTI1_IRQn
#define SW1_BLOCK_Pin GPIO_PIN_3
#define SW1_BLOCK_GPIO_Port GPIOI
#define L_READY_N_Pin GPIO_PIN_2
#define L_READY_N_GPIO_Port GPIOI
#define SW2_D2_Pin GPIO_PIN_4
#define SW2_D2_GPIO_Port GPIOD
#define SW2_D1_Pin GPIO_PIN_3
#define SW2_D1_GPIO_Port GPIOD
#define SW2_D0_Pin GPIO_PIN_2
#define SW2_D0_GPIO_Port GPIOD
#define ALARM_Pin GPIO_PIN_1
#define ALARM_GPIO_Port GPIOI
#define SW1_STROBE_Pin GPIO_PIN_13
#define SW1_STROBE_GPIO_Port GPIOH
#define SW1_STROBE_EXTI_IRQn EXTI15_10_IRQn
#define SW1_READY_Pin GPIO_PIN_14
#define SW1_READY_GPIO_Port GPIOH
#define HARD_RST_N_Pin GPIO_PIN_0
#define HARD_RST_N_GPIO_Port GPIOI
#define HARD_RST_N_EXTI_IRQn EXTI0_IRQn
#define EEPROM_WP_Pin GPIO_PIN_2
#define EEPROM_WP_GPIO_Port GPIOF
#define SPI6_CS0_Pin GPIO_PIN_8
#define SPI6_CS0_GPIO_Port GPIOG
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define SW1_MODE_SEL_Pin GPIO_PIN_12
#define SW1_MODE_SEL_GPIO_Port GPIOH
#define TMPGD_Pin GPIO_PIN_10
#define TMPGD_GPIO_Port GPIOF
#define SW1_D5_Pin GPIO_PIN_11
#define SW1_D5_GPIO_Port GPIOH
#define SW1_D4_Pin GPIO_PIN_10
#define SW1_D4_GPIO_Port GPIOH
#define SW1_D0_Pin GPIO_PIN_6
#define SW1_D0_GPIO_Port GPIOH
#define SW1_D2_Pin GPIO_PIN_8
#define SW1_D2_GPIO_Port GPIOH
#define SW1_D3_Pin GPIO_PIN_9
#define SW1_D3_GPIO_Port GPIOH
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SW1_D1_Pin GPIO_PIN_7
#define SW1_D1_GPIO_Port GPIOH
#define LATCH_Pin GPIO_PIN_9
#define LATCH_GPIO_Port GPIOE
#define SPI4_CS0_Pin GPIO_PIN_11
#define SPI4_CS0_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define PRO_DIS_N_Pin GPIO_PIN_8
#define PRO_DIS_N_GPIO_Port GPIOD
#define SPI4_CS1_Pin GPIO_PIN_10
#define SPI4_CS1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
