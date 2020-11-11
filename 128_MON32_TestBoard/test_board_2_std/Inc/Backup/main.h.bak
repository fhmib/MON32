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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern uint8_t terminal_buf[];
extern uint8_t communication_buf[];
extern uint8_t uart1_irq_sel;
extern uint8_t print_trans_data;
extern uint32_t tim_counter_max;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t send_cmd(uint8_t ch, char *arg);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW2_MODE_SEL_Pin GPIO_PIN_3
#define SW2_MODE_SEL_GPIO_Port GPIOB
#define SW2_D4_Pin GPIO_PIN_6
#define SW2_D4_GPIO_Port GPIOD
#define SW2_READY_Pin GPIO_PIN_0
#define SW2_READY_GPIO_Port GPIOD
#define SW2_D3_Pin GPIO_PIN_5
#define SW2_D3_GPIO_Port GPIOD
#define SW2_STROBE_Pin GPIO_PIN_1
#define SW2_STROBE_GPIO_Port GPIOD
#define SW1_BLOCK_Pin GPIO_PIN_3
#define SW1_BLOCK_GPIO_Port GPIOI
#define L_READY_Pin GPIO_PIN_2
#define L_READY_GPIO_Port GPIOI
#define MASTER_RESET_Pin GPIO_PIN_4
#define MASTER_RESET_GPIO_Port GPIOI
#define SW2_D2_Pin GPIO_PIN_4
#define SW2_D2_GPIO_Port GPIOD
#define SW2_D1_Pin GPIO_PIN_3
#define SW2_D1_GPIO_Port GPIOD
#define SW2_D0_Pin GPIO_PIN_2
#define SW2_D0_GPIO_Port GPIOD
#define IN_ALARM_Pin GPIO_PIN_1
#define IN_ALARM_GPIO_Port GPIOI
#define SW1_STROBE_Pin GPIO_PIN_13
#define SW1_STROBE_GPIO_Port GPIOH
#define SW1_READY_Pin GPIO_PIN_14
#define SW1_READY_GPIO_Port GPIOH
#define HARD_RESET_Pin GPIO_PIN_0
#define HARD_RESET_GPIO_Port GPIOI
#define PRO_CTL_Pin GPIO_PIN_6
#define PRO_CTL_GPIO_Port GPIOC
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define SW1_MODE_SEL_Pin GPIO_PIN_12
#define SW1_MODE_SEL_GPIO_Port GPIOH
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
#define PRO_DIS_N_Pin GPIO_PIN_8
#define PRO_DIS_N_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
