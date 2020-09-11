/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "command.h"
#include "iwdg.h"
#include "flash_if.h"
#include "functions.h"

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
/* USER CODE BEGIN Variables */
osSemaphoreId_t uartProcessSemaphore;
osSemaphoreId_t cmdProcessSemaphore;

osMessageQueueId_t mid_ISR;                // message queue id

osMutexId_t i2c1Mutex;
osMutexId_t i2c2Mutex;
osMutexId_t spi2Mutex;
osMutexId_t spi3Mutex;
osMutexId_t spi4Mutex;
osMutexId_t spi5Mutex;
osMutexId_t spi6Mutex;
const osMutexAttr_t Thread_Mutex_attr = {
  "Mutex",                              // human readable mutex name
  osMutexPrioInherit,                       // attr_bits
  NULL,                                     // memory for control block   
  0U                                        // size for control block
};

osThreadId_t cmdProcessTaskHandle;
const osThreadAttr_t cmdProcessTask_attributes = {
  .name = "cmdProcessTask",
  .priority = (osPriority_t) CMD_PROCESS_PRIORITY,
  .stack_size = 2048
};

osThreadId_t uartProcessTaskHandle;
const osThreadAttr_t uartProcessTask_attributes = {
  .name = "uartProcessTask",
  .priority = (osPriority_t) UART_PROCESS_PRIORITY,
  .stack_size = 1024
};

osThreadId_t watchdogTaskHandle;
const osThreadAttr_t watchdogTask_attributes = {
  .name = "watchdogTask",
  .priority = (osPriority_t) WATCHDOG_PRIORITY,
  .stack_size = 512
};

osThreadId_t isrTaskHandle;
const osThreadAttr_t isrTask_attributes = {
  .name = "isrTask",
  .priority = (osPriority_t) INTERRUPT_TASK_PRIORITY,
  .stack_size = 1024
};

osThreadId_t monTaskHandle;
const osThreadAttr_t monTask_attributes = {
  .name = "monTask",
  .priority = (osPriority_t) MONITOR_PRIORITY,
  .stack_size = 1024
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void uartProcessTask(void *argument);
void cmdProcessTask(void *argument);
void watchdogTask(void *argument);
void isrTask(void *argument);
void monitorTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  uartProcessSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (uartProcessSemaphore == NULL) {
  }

  cmdProcessSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (cmdProcessSemaphore == NULL) {
  }

  mid_ISR = osMessageQueueNew(ISR_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_ISR == NULL) {
  }

  i2c1Mutex = osMutexNew(&Thread_Mutex_attr);
  if (i2c1Mutex == NULL) {
  }

  i2c2Mutex = osMutexNew(&Thread_Mutex_attr);
  if (i2c2Mutex == NULL) {
  }

  spi2Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi2Mutex == NULL) {
  }

  spi3Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi3Mutex == NULL) {
  }

  spi4Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi4Mutex == NULL) {
  }

  spi5Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi5Mutex == NULL) {
  }

  spi6Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi6Mutex == NULL) {
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  uartProcessTaskHandle = osThreadNew(uartProcessTask, NULL, &uartProcessTask_attributes);
  cmdProcessTaskHandle = osThreadNew(cmdProcessTask, NULL, &cmdProcessTask_attributes);
  watchdogTaskHandle = osThreadNew(watchdogTask, NULL, &watchdogTask_attributes);
  isrTaskHandle = osThreadNew(isrTask, NULL, &isrTask_attributes);
  monTaskHandle = osThreadNew(monitorTask, NULL, &monTask_attributes);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
extern TransStu trans_buf;
extern RespondStu resp_buf;
struct UartTaskSync{
  uint8_t ProcessOnGoing;
  uint8_t ProcessTimeout;
} uart_sync;

void uartProcessTask(void *argument)
{
  uint8_t msg_length;
  uint16_t msg_id;
  uint8_t chk;
  uint32_t wait_time;
  uint16_t *p16;
  osStatus_t status;

  CLEAR_BIT(huart1.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&huart1);
  HAL_UART_Receive_IT(&huart1, trans_buf.buf, 1);

  uart_sync.ProcessTimeout = 0;
  uart_sync.ProcessOnGoing = 0;
  trans_buf.stage = TRANS_WAIT_START;

  for(;;)
  {
    if (trans_buf.stage == TRANS_WAIT_START) {
      wait_time = osWaitForever;
    } else if (trans_buf.stage == TRANS_WAIT_HEADER) {
      wait_time = pdMS_TO_TICKS(TRANS_WAIT_HEAD_TIME);
    } else if (trans_buf.stage == TRANS_RECV_DATA) {
      wait_time = pdMS_TO_TICKS(TRANS_WAIT_TIME);
    }
    status = osSemaphoreAcquire(uartProcessSemaphore, wait_time);
    if (status != osOK) {
      HAL_UART_DMAStop(&huart1);
      EPT("DMA timeout, stage = %u, ErrorCode = %#X\n", trans_buf.stage, huart1.ErrorCode);
      THROW_LOG("DMA timeout, stage = %u, ErrorCode = %#X\n", trans_buf.stage, huart1.ErrorCode);
      Uart_Respond(msg_id, RESPOND_FAILURE, NULL, 0);
      trans_buf.stage = TRANS_WAIT_START;
      HAL_UART_Receive_IT(&huart1, trans_buf.buf, 1);
      continue;
    }

    if (trans_buf.stage == TRANS_WAIT_START) {
      // Received start byte
      if (trans_buf.buf[0] != TRANS_START_BYTE) {
        // Invalid data
        EPT("Invalid character: %#X\n", trans_buf.buf[0]);
        HAL_UART_Receive_IT(&huart1, trans_buf.buf, 1);
      } else {
        // Prepare to receive message header
        msg_id = 0;
        trans_buf.stage = TRANS_WAIT_HEADER;
        HAL_UART_Receive_DMA(&huart1, &trans_buf.buf[1], 3);
      }
    } else if (trans_buf.stage == TRANS_WAIT_HEADER) {
      // Received message header
      p16 = (uint16_t*)&trans_buf.buf[CMD_SEQ_MSG_ID];
      msg_id = switch_endian_16(*p16);
      msg_length = trans_buf.buf[CMD_SEQ_MSG_LENGTH];
      if (msg_length < 4) {
        // Length invalid
        PRINT_HEX("buf", trans_buf.buf, 4);
        EPT("Invalid length %#X\n", msg_length);
        THROW_LOG("Invalid length %#X\n", msg_length);
        Uart_Respond(msg_id, RESPOND_FAILURE, NULL, 0);
        trans_buf.stage = TRANS_WAIT_START;
        HAL_UART_Receive_IT(&huart1, trans_buf.buf, 1);
      } else {
        trans_buf.stage = TRANS_RECV_DATA;
        HAL_UART_Receive_DMA(&huart1, &trans_buf.buf[CMD_SEQ_MSG_DATA], msg_length - 3);
        //EPT("Message Length: %u, ID: %u\n", msg_length, msg_id);
        trans_buf.length = msg_length + 1;
      }
    } else if (trans_buf.stage == TRANS_RECV_DATA){
      // Received message data
      //PRINT_HEX("buf", trans_buf.buf, trans_buf.length);

      // TODO: Check CRC
      chk = Cal_Check((uint8_t*)&trans_buf.buf[1], trans_buf.length - 1 - 1);
      if (chk ^ trans_buf.buf[trans_buf.length - 1]) {
        // Incorrect
        THROW_LOG("UART CRC32 verification failed, msg_length = %#X, trans_buf.length = %#X, chk = %#X, rcv_chk = %#X\n",\
          msg_length, trans_buf.length, chk, trans_buf.buf[trans_buf.length - 1]);
        EPT("UART CRC32 verification failed\n");
        EPT("msg_length = %#X, trans_buf.length = %#X, chk = %#X, rcv_chk = %#X\n", msg_length, trans_buf.length, chk, trans_buf.buf[trans_buf.length - 1]);
        PRINT_HEX("buf", trans_buf.buf, trans_buf.length);
        FILL_RESP_MSG(msg_id, RESPOND_CHECK_ERR, 0);
      } else {
        if (!uart_sync.ProcessOnGoing) {
          // Process Command and Respond
          xTaskNotifyGive(cmdProcessTaskHandle);
          status = osSemaphoreAcquire(cmdProcessSemaphore, pdMS_TO_TICKS(COMMAND_PROCESS_TIME));
          if (status == osErrorTimeout) {
            // TODO: Process command timeout
            THROW_LOG("Process command %#X timeout\n", msg_id);
            EPT("Process command timeout\n");
            uart_sync.ProcessTimeout = 1;
            FILL_RESP_MSG(msg_id, RESPOND_FAILURE, 0);
          } else if (status != osOK) {
            EPT("Error, status=%#X\n", status);
            THROW_LOG("Error, status=%#X\n", status);
            FILL_RESP_MSG(msg_id, RESPOND_FAILURE, 0);
          } else {
            // success
          }
        } else {
            EPT("Detected process is ongoing\n");
            THROW_LOG("Detected process is ongoing\n");
            FILL_RESP_MSG(msg_id, RESPOND_FAILURE, 0);
        }
      }

      trans_buf.stage = TRANS_WAIT_START;
      HAL_UART_Receive_IT(&huart1, trans_buf.buf, 1);
      Uart_Respond(resp_buf.cmd, resp_buf.status, resp_buf.buf, resp_buf.length);
    }
  }
}

void cmdProcessTask(void *argument)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (trans_buf.stage != TRANS_RECV_DATA) {
      EPT("Stage incorrect\n");
      THROW_LOG("Stage incorrect\n");
      continue;
    }

    uart_sync.ProcessOnGoing = 1;
    Cmd_Process();

    if (uart_sync.ProcessTimeout == 0) {
      osSemaphoreRelease(cmdProcessSemaphore);
      //Uart_Respond(resp_buf.cmd, resp_buf.status, resp_buf.buf, resp_buf.length);
    } else {
      EPT("Detected process timeout\n");
      THROW_LOG("Detected process timeout\n");
      uart_sync.ProcessTimeout = 0;
    }

    uart_sync.ProcessOnGoing = 0;
  }
}

void watchdogTask(void *argument)
{
  uint32_t delay = pdMS_TO_TICKS(WATCH_DOG_DELAY_TIME);

  HAL_IWDG_Refresh(&hiwdg);
  for(;;)
  {
    //EPT("Feed dog\n");
    osDelay(delay);
    HAL_IWDG_Refresh(&hiwdg);
  }
}

extern UpgradeFlashState upgrade_status;
void isrTask(void *argument)
{
  osStatus_t status;
  MsgStruct msg;
  uint8_t switch_channel, switch_pos;
  //uint32_t switch_channel;

  MON32_Init();

  for (;;)
  {
    status = osMessageQueueGet(mid_ISR, &msg, 0U, osWaitForever);
    if (status != osOK)
      continue;

    if (msg.type == MSG_TYPE_FLASH_ISR) {
      EPT("Received MSG_TYPE_FLASH_ISR, flash_empty = %u\n", upgrade_status.flash_empty);
      status = Update_Up_Status(&upgrade_status);
      if (status != osOK) {
        EPT("Update upgrade status to eeprom failed\n");
      }
    } else if (msg.type == MSG_TYPE_SWITCH1_ISR) {
      switch_channel = TX_SWITCH_CHANNEL;
      switch_pos = Get_Switch_Position_By_IO(switch_channel);
      EPT("Switch channel is %u, Switch position is %u\n", switch_channel, switch_pos);
      if (switch_pos >= 0x40) {
        EPT("Invalid switch position = %u [io]\n", switch_pos);
        THROW_LOG("Invalid switch position = %u [io]\n", switch_pos);
        continue;
      }

      Clear_Switch_Ready(switch_channel);
      if (Set_Switch(switch_channel, switch_pos)) {
        run_status.tx_switch_channel = 0xFF;
        continue;
      }

      run_status.tx_switch_channel = switch_pos;
      osDelay(pdMS_TO_TICKS(4));

      // Check
      if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
        Reset_Switch(switch_channel);
        continue;
      }

      Set_Switch_Ready(switch_channel);
    } else if (msg.type == MSG_TYPE_SWITCH2_ISR) {
      switch_channel = RX_SWITCH_CHANNEL;
      switch_pos = Get_Switch_Position_By_IO(switch_channel);
      EPT("Switch channel is %u, Switch position is %u\n", switch_channel, switch_pos);
      if (switch_pos >= 0x20) {
        EPT("Invalid switch position = %u [io]\n", switch_pos);
        THROW_LOG("Invalid switch position = %u [io]\n", switch_pos);
        continue;
      }

      Clear_Switch_Ready(switch_channel);
      if (Set_Switch(switch_channel, switch_pos)) {
        run_status.rx_switch_channel = 0xFF;
        continue;
      }

      run_status.rx_switch_channel = switch_pos;
      osDelay(pdMS_TO_TICKS(4));

      // Check
      if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
        Reset_Switch(switch_channel);
        continue;
      }

      Set_Switch_Ready(switch_channel);
    }
  }
}

void monitorTask(void *argument)
{
  osStatus_t status;
  uint16_t value;
  double voltage, temp;
  uint8_t pre_alarm;

  osDelay(pdMS_TO_TICKS(1000));
  pre_alarm = run_status.exp == 0 ? 0: 1;

  for (;;) {
    // 2x32 Switch Block signal
    if (HAL_GPIO_ReadPin(SW1_BLOCK_GPIO_Port, SW1_BLOCK_Pin) == GPIO_PIN_RESET) {
      if (!run_status.tx_block) {
        Clear_Switch_Ready(TX_SWITCH_CHANNEL);
        Reset_Switch_Only(TX_SWITCH_CHANNEL);
        run_status.tx_block = 1;
      }
    } else {
      if (run_status.tx_block) {
        if (run_status.tx_switch_channel != 0xFF) {
          if (Set_Switch(TX_SWITCH_CHANNEL, run_status.tx_switch_channel)) {
            run_status.tx_switch_channel = 0xFF;
            continue;
          }

          osDelay(pdMS_TO_TICKS(4));

          // Check
          if (Get_Current_Switch_Channel(TX_SWITCH_CHANNEL) != run_status.tx_switch_channel) {
            Reset_Switch(TX_SWITCH_CHANNEL);
            continue;
          } else {
            Set_Switch_Ready(TX_SWITCH_CHANNEL);
          }
        }
        run_status.tx_block = 0;
      }
    }

    // PRO_Dis signal
    if (HAL_GPIO_ReadPin(PRO_DIS_N_GPIO_Port, PRO_DIS_N_Pin) == GPIO_PIN_RESET) {
      if (!run_status.tosa_enable) {
        if (Enable_Tosa()) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        }
        run_status.tosa_enable = 1;
      }
    } else {
      if (run_status.tosa_enable) {
        if (Disable_Tosa()) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        }
        run_status.tosa_enable = 0;
      }
    }

    if (run_status.exp && !pre_alarm) {
      Set_Alarm();
      pre_alarm = 1;
    } else if (!run_status.exp && pre_alarm) {
      Clear_Alarm();
      pre_alarm = 0;
    }

    osDelay(pdMS_TO_TICKS(200));
  } 
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
