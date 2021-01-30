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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint8_t reset_flag = 0;
uint8_t flash_in_use;
UpgradeFlashState upgrade_status;

AlarmHistoryState history_alarm_status;

TosaCalData tosa_table_C98[TOSA_TABLE_COUNT];
TosaCalData tosa_table_C122[TOSA_TABLE_COUNT];
uint8_t tosa_table_count_C98 = 0;
uint8_t tosa_table_count_C122 = 0;
double tosa_power_high_max_thr_C98 = 0;
double tosa_power_low_min_thr_C98 = 0;
double tosa_power_high_max_thr_C122 = 0;
double tosa_power_low_min_thr_C122 = 0;

uint8_t device_busy = 1;
uint8_t allow_tosa = 1; // Indicates allow tosa enable or not. Set by reset.
uint8_t enable_tosa_failed = 0; // Indicates endable tosa failed. Clear when pro_dis_n is 0.

const uint32_t error_file_flash_addr[] =  {  0x08100000, 0x08104000, 0x08108000, 0x0810C000, 0x08110000, 0x08120000,\
                                       0x08140000  };
const uint32_t error_file_flash_end = 0x0815FFFF;
const uint8_t error_file_flash_count = sizeof(error_file_flash_addr) / sizeof (error_file_flash_addr[0]);
                                       
const uint32_t normal_file_flash_addr[] =  {  0x08160000, 0x08180000, 0x081A0000};
const uint32_t normal_file_flash_end = 0x081BFFFF;
const uint8_t normal_file_flash_count = sizeof(normal_file_flash_addr) / sizeof (normal_file_flash_addr[0]);

LogFileState log_file_state;

//double power_arr_for_test[10];

RunTimeStatus run_status __attribute__((at(0x2002FC00)));

uint8_t lock_debug = 0;

SwTimControl sw_tim_control;
MsgStruct tim_isr_msg;
MsgStruct pro_isr_msg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UpgradeStruct up_state;

/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_SPI6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  uint8_t buf[3] = {0};
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi3, buf, 1, 10) != HAL_OK) {
    EPT("spi3 transmit filed when init\n");
  }
  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_SET);

  run_status.internal_exp = 0;
  Set_Flag(&run_status.internal_exp, INT_EXP_CONST);
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();
 
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
 
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MON32_Init(void)
{
  osStatus_t status;
  uint32_t i;

  EPT("Firmware version: %s\n", fw_version);

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_PN, (uint8_t*)pn, 16);
  pn[16] = 0;
  status |= RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_HW_VERSION, (uint8_t*)hw_version, 4);
  hw_version[4] = 0;
  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET){
    SET_RESETFLAG(BOR_RESET_BIT);
    //EPT("BOR Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET){
    SET_RESETFLAG(PIN_RESET_BIT);
    //EPT("PIN Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET){
    SET_RESETFLAG(POR_RESET_BIT);
    //EPT("POR/PDR Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET){
    SET_RESETFLAG(SFT_RESET_BIT);
    //EPT("Software Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET){
    SET_RESETFLAG(IWDG_RESET_BIT);
    //EPT("Independent Watchdog Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET){
    SET_RESETFLAG(WWDG_RESET_BIT);
    //EPT("Window Watchdog Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET){
    SET_RESETFLAG(LPWR_RESET_BIT);
    //EPT("Low-Power Reset is set\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  EPT("Run status maigc = %#X, uart_reset = %u\n", run_status.maigc, run_status.uart_reset);

  status = RTOS_DAC128S085_C98_Write(0, 0, DAC128S085_MODE_WTM);
  if (status != osOK) {
    EPT("Changing DAC128S085 to WTM mode failed\n");
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
  }

  status = RTOS_DAC128S085_C122_Write(0, 0, DAC128S085_MODE_WTM);
  if (status != osOK) {
    EPT("Changing DAC128S085 C122 to WTM mode failed\n");
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
  }

  update_tosa_table();

  HAL_GPIO_WritePin(SW1_READY_GPIO_Port, SW1_READY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW2_READY_GPIO_Port, SW2_READY_Pin, GPIO_PIN_SET);

  if (IS_RESETFLAG_SET(SFT_RESET_BIT) || IS_RESETFLAG_SET(IWDG_RESET_BIT)) {
    if (run_status.maigc == RUN_MAGIC) {
      if (run_status.uart_reset) {
        run_status.uart_reset = 0;
        EPT("Startup with UART Reset\n");
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Startup with SOFT Reset\n");
      } else {
        if (IS_RESETFLAG_SET(IWDG_RESET_BIT)) {
          EPT("Startup with WATCHDOG Reset\n");
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Startup with WATCHDOG Reset\n");
        } else {
          EPT("Startup with HARD Reset\n");
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Startup with HARD Reset\n");
        }
      }

      // exp signal
      if (run_status.exp) {
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
      }
#if 0
      // tx switch signal
      if (run_status.tx_block == 1) {
        HAL_GPIO_WritePin(SW1_READY_GPIO_Port, SW1_READY_Pin, GPIO_PIN_SET);
      } else {
        if (run_status.tx_switch_channel < 0x40) {
          if (run_status.tx_switch_channel == Get_Current_Switch_Channel(TX_SWITCH_CHANNEL)) {
            HAL_GPIO_WritePin(SW1_READY_GPIO_Port, SW1_READY_Pin, GPIO_PIN_RESET);
          } else {
            EPT("Detect 2x32 optical switch channel incorrect while initializing\n");
            THROW_LOG(MSG_TYPE_ERROR_LOG, "Detect 2x32 optical switch channel %u incorrect while initializing\n", run_status.tx_switch_channel);
            Reset_Switch_Only(TX_SWITCH_CHANNEL);
            HAL_GPIO_WritePin(SW1_READY_GPIO_Port, SW1_READY_Pin, GPIO_PIN_SET);
          }
        } else {
          HAL_GPIO_WritePin(SW1_READY_GPIO_Port, SW1_READY_Pin, GPIO_PIN_SET);
        }
      }

      // 1x32 switch signal
      if (run_status.rx_switch_channel < 0x20) {
        if (run_status.rx_switch_channel == Get_Current_Switch_Channel(RX_SWITCH_CHANNEL)) {
          HAL_GPIO_WritePin(SW2_READY_GPIO_Port, SW2_READY_Pin, GPIO_PIN_RESET);
        } else {
          EPT("Detect 1x32 optical switch channel incorrect while initializing\n");
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Detect 1x32 optical switch channel %u incorrect while initializing\n", run_status.rx_switch_channel);
          Reset_Switch_Only(RX_SWITCH_CHANNEL);
          HAL_GPIO_WritePin(SW2_READY_GPIO_Port, SW2_READY_Pin, GPIO_PIN_SET);
        }
      } else {
        HAL_GPIO_WritePin(SW2_READY_GPIO_Port, SW2_READY_Pin, GPIO_PIN_SET);
      }
#endif
      // Modulation
      if (run_status.modulation) {
        if (run_status.tosa_C122) {
          HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
          HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
        } else {
          HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
          HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
        }
      }
      
      // Lazer
      if (run_status.lazer_ready) {
        if (run_status.tosa_C122) {
          HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(L_READY_N_2_GPIO_Port, L_READY_N_2_Pin, GPIO_PIN_RESET);
        } else {
          HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(L_READY_N_2_GPIO_Port, L_READY_N_2_Pin, GPIO_PIN_SET);
        }
      } else {
        HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(L_READY_N_2_GPIO_Port, L_READY_N_2_Pin, GPIO_PIN_SET);
      }
      
      HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
      osDelay(pdMS_TO_TICKS(1));
      HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
    } else {
      // Init device
      Reset_Switch_Only(TX_SWITCH_CHANNEL);
      MON32_Init_Dev();
      EPT("Variable run_status exception\n");
      THROW_LOG(MSG_TYPE_ERROR_LOG, "Variable run_status exception\n");
    }
  } else if (IS_RESETFLAG_SET(BOR_RESET_BIT) || IS_RESETFLAG_SET(POR_RESET_BIT)) {
    // Init device
    MON32_Init_Dev();
    EPT("Startup with POWER Reset\n");
    //THROW_LOG("Startup with POWER Reset\n");
  } else {
    // Init device
    run_status.maigc = 0;
    Reset_Switch_Only(TX_SWITCH_CHANNEL);
    MON32_Init_Dev();
    EPT("Startup with MASTER Reset\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Startup with MASTER Reset\n");
  }

  up_state.run = RUN_MODE_APPLICATION;
  up_state.is_erasing = 0;
  flash_in_use = 0;
  
  FLASH_If_Init();
  status = Get_Up_Status(&upgrade_status);
  if (status) {
    EPT("Get upgrade status failed, status = %d\n", status);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Get upgrade status failed, status = %d\n", status);
  }
  EPT("upgrade_status: %#X, %u, %u, %u, %#X, %u, %#X\n", upgrade_status.magic, upgrade_status.run, upgrade_status.flash_empty, upgrade_status.length, upgrade_status.crc32,\
                upgrade_status.factory_length, upgrade_status.factory_crc32);

  if (upgrade_status.magic != UPGRADE_MAGIC) {
    EPT("Verify upgrade magic failed\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Verify upgrade magic failed\n");
  }
  switch (upgrade_status.run) {
    case 0:
      EPT("Boot from factory\n");
      up_state.upgrade_addr = APPLICATION_1_ADDRESS;
      up_state.upgrade_sector = APPLICATION_1_SECTOR;
      break;
    case 1:
      EPT("Boot from Application 1\n");
      up_state.upgrade_addr = APPLICATION_2_ADDRESS;
      up_state.upgrade_sector = APPLICATION_2_SECTOR;
      break;
    case 2:
      EPT("Boot from Application 2\n");
      up_state.upgrade_addr = APPLICATION_1_ADDRESS;
      up_state.upgrade_sector = APPLICATION_1_SECTOR;
      break;
    default:
      EPT("f_state.run invalid\n");
      THROW_LOG(MSG_TYPE_ERROR_LOG, "f_state.run invalid\n");
      up_state.upgrade_addr = RESERVE_ADDRESS;
      up_state.upgrade_sector = RESERVE_SECTOR;
      break;
  }
  EPT("FLASH->OPTCR = %#X\n", FLASH->OPTCR);
  if (!upgrade_status.flash_empty) {
    flash_in_use = 1;
    up_state.is_erasing = 1;
    // erase flash
    EPT("flash is not empty\n");
    if (up_state.upgrade_addr != RESERVE_ADDRESS)
      FLASH_If_Erase_IT(up_state.upgrade_sector);
    EPT("erase sector...\n");
  } else {
    EPT("flash is empty\n");
  }
  
  // log
  Get_Log_Status(&log_file_state);
  EPT("log_file_state: %#X, %#X\n", log_file_state.offset, log_file_state.header);
  for (i = 0; i < error_file_flash_count - 1; ++i) {
    if (log_file_state.error_offset < error_file_flash_addr[i + 1]) {
      break;
    }
  }
  log_file_state.error_cur_sector = ERROR_LOG_FIRST_SECTOR + i;

  for (i = 0; i < normal_file_flash_count - 1; ++i) {
    if (log_file_state.normal_offset < normal_file_flash_addr[i + 1]) {
      break;
    }
  }
  log_file_state.normal_cur_sector = NORMAL_LOG_FIRST_SECTOR + i;

  
  // alarm
  if ((status = Get_EEPROM_Alarm_Status(&history_alarm_status)) != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
  }
  
  // check calibration data
  Check_Cali();
}

void MON32_Init_Dev(void)
{
  osStatus_t status;

  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L_READY_N_2_GPIO_Port, L_READY_N_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
  Init_Run_Status();
  Get_Threshold_Table(&run_status.thr_table);
  Reset_Tec_Dest_Temp(&run_status.thr_table);

  run_status.tosa_high = Get_Tosa_Data(run_status.tosa_dst_power_high);
  run_status.tosa_low = Get_Tosa_Data(run_status.tosa_dst_power_low);

  status = RTOS_DAC128S085_C122_Write(DAC128S085_TOSA_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C122_Write(DAC128S085_TEC_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C122_Write(DAC128S085_TEC_VALUE_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C122_Write(DAC128S085_TOSA_POWER_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C98_Write(DAC128S085_TOSA_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C98_Write(DAC128S085_TEC_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C98_Write(DAC128S085_TEC_VALUE_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_C98_Write(DAC128S085_TOSA_POWER_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
  }
#if 0
  if (run_status.tosa_high.tosa_dac) {
    status = RTOS_DAC128S085_C98_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
    status |= RTOS_DAC128S085_C122_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    }
    RTOS_DAC128S085_C98_Write(DAC128S085_TOSA_POWER_CHANNEL, run_status.tosa_high.tosa_dac, DAC128S085_MODE_NORMAL);
    RTOS_DAC128S085_C122_Write(DAC128S085_TOSA_POWER_CHANNEL, run_status.tosa_high.tosa_dac, DAC128S085_MODE_NORMAL);
  }
#endif
}

extern FLASH_ProcessTypeDef pFlash;
extern osMessageQueueId_t mid_ISR;
extern osMessageQueueId_t mid_SwISR;
extern osMessageQueueId_t mid_LazerManager;
extern osSemaphoreId_t logEraseSemaphore;
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  MsgStruct isr_msg;
  if (0xFFFFFFFFU == ReturnValue) {
    EPT("Erase flash operation completely\n");
    HAL_FLASH_Lock();
    flash_in_use = 0;
    if (up_state.is_erasing == 0) {
      // send signal to logTask
      osSemaphoreRelease(logEraseSemaphore);
    } else {
      upgrade_status.flash_empty = 1;
      up_state.is_erasing = 0;
      isr_msg.type = MSG_TYPE_FLASH_ISR;
      osMessageQueuePut(mid_ISR, &isr_msg, 0U, 0U);
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      // Rising Edge
      pro_isr_msg.type = MSG_TYPE_PRO_CTL_RAISING_ISR;
      osMessageQueuePut(mid_LazerManager, &pro_isr_msg, 0U, 0U);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      // Failing Edge
      pro_isr_msg.type = MSG_TYPE_PRO_CTL_FALLING_ISR;
      osMessageQueuePut(mid_LazerManager, &pro_isr_msg, 0U, 0U);
    }
  } else if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      // Rising Edge
      pro_isr_msg.type = MSG_TYPE_PRO_CTL_RAISING_ISR;
      osMessageQueuePut(mid_LazerManager, &pro_isr_msg, 0U, 0U);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      // Failing Edge
      pro_isr_msg.type = MSG_TYPE_PRO_CTL_FALLING_ISR;
      osMessageQueuePut(mid_LazerManager, &pro_isr_msg, 0U, 0U);
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  MsgStruct isr_msg;

  if (GPIO_Pin == SW1_STROBE_Pin) {
    if (HAL_GPIO_ReadPin(SW1_MODE_SEL_GPIO_Port, SW1_MODE_SEL_Pin) == GPIO_PIN_SET) {
    } else {
      isr_msg.type = MSG_TYPE_SWITCH1_ISR;
      osMessageQueuePut(mid_ISR, &isr_msg, 0U, 0U);
    }
  } else if (GPIO_Pin == SW2_STROBE_Pin) {
    if (HAL_GPIO_ReadPin(SW2_MODE_SEL_GPIO_Port, SW2_MODE_SEL_Pin) == GPIO_PIN_SET) {
    } else {
      isr_msg.type = MSG_TYPE_SWITCH2_ISR;
      osMessageQueuePut(mid_ISR, &isr_msg, 0U, 0U);
    }
  } else if (GPIO_Pin == HARD_RST_N_Pin) {
    __NVIC_SystemReset();
    while (1) {
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
    if (++sw_tim_control.counter < sw_tim_control.time) {
    } else {
      HAL_TIM_Base_Stop_IT(&htim6);
      tim_isr_msg.type = MSG_TYPE_SWITCH_DAC_ISR;
      osMessageQueuePut(mid_SwISR, &tim_isr_msg, 0U, 0U);
      sw_tim_control.counter = 0;
    }
  }
  /* USER CODE END Callback 1 */
}

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
