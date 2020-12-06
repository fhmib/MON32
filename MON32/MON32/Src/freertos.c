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
#include "tim.h"

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
osSemaphoreId_t switchSemaphore;
osSemaphoreId_t logEraseSemaphore;                  // semaphore id

osMessageQueueId_t mid_LogMsg;
osMessageQueueId_t mid_ISR;                // message queue id
osMessageQueueId_t mid_SwISR;                // message queue id
osMessageQueueId_t mid_LazerManager;                // message queue id
osMessageQueueId_t mid_CmdProcess;                // message queue id

osMutexId_t i2c1Mutex;
osMutexId_t i2c2Mutex;
osMutexId_t spi2Mutex;
osMutexId_t spi3Mutex;
osMutexId_t spi4Mutex;
osMutexId_t spi5Mutex;
osMutexId_t spi6Mutex;
osMutexId_t swMutex;
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

osThreadId_t swIsrTaskHandle;
const osThreadAttr_t swIsrTask_attributes = {
  .name = "swIsrTask",
  .priority = (osPriority_t) INTERRUPT_TASK_PRIORITY,
  .stack_size = 1024
};

osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .priority = (osPriority_t) LOG_MESSAGE_PRIORITY,
  .stack_size = 1024
};

osThreadId_t monTaskHandle;
const osThreadAttr_t monTask_attributes = {
  .name = "monTask",
  .priority = (osPriority_t) MONITOR_PRIORITY,
  .stack_size = 1024
};

osThreadId_t lazerManagerTaskHandle;
const osThreadAttr_t lazerManagerTask_attributes = {
  .name = "lazerManagerTask",
  .priority = (osPriority_t) LAZER_MANAGER_PRIORITY,
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
void swIsrTask(void *argument);
void logTask(void *argument);
void monitorTask(void *argument);
void lazerManagerTask(void *argument);

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
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  cmdProcessSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (cmdProcessSemaphore == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  switchSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (switchSemaphore == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  logEraseSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (logEraseSemaphore == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  mid_LogMsg = osMessageQueueNew(LOG_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_LogMsg == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  mid_ISR = osMessageQueueNew(ISR_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_ISR == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  mid_SwISR = osMessageQueueNew(SW_ISR_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_SwISR == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  mid_LazerManager = osMessageQueueNew(LM_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_LazerManager == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  mid_CmdProcess = osMessageQueueNew(LM_QUEUE_LENGTH, sizeof(MsgStruct), NULL);
  if (mid_CmdProcess == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  i2c1Mutex = osMutexNew(&Thread_Mutex_attr);
  if (i2c1Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  i2c2Mutex = osMutexNew(&Thread_Mutex_attr);
  if (i2c2Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  spi2Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi2Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  spi3Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi3Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  spi4Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi4Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  spi5Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi5Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  spi6Mutex = osMutexNew(&Thread_Mutex_attr);
  if (spi6Mutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
  }

  swMutex = osMutexNew(&Thread_Mutex_attr);
  if (swMutex == NULL) {
    Set_Flag(&run_status.internal_exp, INT_EXP_INIT);
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
  swIsrTaskHandle = osThreadNew(swIsrTask, NULL, &swIsrTask_attributes);
  logTaskHandle = osThreadNew(logTask, NULL, &logTask_attributes);
  monTaskHandle = osThreadNew(monitorTask, NULL, &monTask_attributes);
  lazerManagerTaskHandle = osThreadNew(lazerManagerTask, NULL, &lazerManagerTask_attributes);
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
    status = osSemaphoreAcquire(uartProcessSemaphore, wait_time / 2);
    if (status != osOK) {
      HAL_UART_DMAStop(&huart1);
      EPT("DMA timeout, stage = %u, ErrorCode = %#X\n", trans_buf.stage, huart1.ErrorCode);
      THROW_LOG(MSG_TYPE_ERROR_LOG, "DMA timeout, stage = %u, ErrorCode = %#X\n", trans_buf.stage, huart1.ErrorCode);
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
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Invalid length %#X\n", msg_length);
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
        THROW_LOG(MSG_TYPE_ERROR_LOG, "UART Checksum verification failed, msg_length = %#X, cid = %#X, chk = %#X, rcv_chk = %#X\n",\
          msg_length, msg_id, chk, trans_buf.buf[trans_buf.length - 1]);
        EPT("msg_length = %#X, trans_buf.length = %#X, chk = %#X, rcv_chk = %#X\n", msg_length, trans_buf.length, chk, trans_buf.buf[trans_buf.length - 1]);
        PRINT_HEX("buf", trans_buf.buf, trans_buf.length);
        FILL_RESP_MSG(msg_id, RESPOND_CHECK_ERR, 0);
      } else {
        if (!uart_sync.ProcessOnGoing) {
          // Process Command and Respond
          xTaskNotifyGive(cmdProcessTaskHandle);
          status = osSemaphoreAcquire(cmdProcessSemaphore, pdMS_TO_TICKS(COMMAND_PROCESS_TIME) / 2);
          if (status == osErrorTimeout) {
            // TODO: Process command timeout
            THROW_LOG(MSG_TYPE_ERROR_LOG, "Process command %#X timeout\n", msg_id);
            EPT("Process command timeout\n");
            uart_sync.ProcessTimeout = 1;
            FILL_RESP_MSG(msg_id, RESPOND_FAILURE, 0);
          } else if (status != osOK) {
            EPT("Error, status=%#X\n", status);
            THROW_LOG(MSG_TYPE_ERROR_LOG, "Process command failed, status=%#X\n", status);
            FILL_RESP_MSG(msg_id, RESPOND_FAILURE, 0);
          } else {
            // success
          }
        } else {
            EPT("Detected process is ongoing\n");
            THROW_LOG(MSG_TYPE_ERROR_LOG, "Detected process is ongoing\n");
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
      THROW_LOG(MSG_TYPE_ERROR_LOG, "Stage incorrect\n");
      continue;
    }

    uart_sync.ProcessOnGoing = 1;
    Cmd_Process();

    if (uart_sync.ProcessTimeout == 0) {
      osSemaphoreRelease(cmdProcessSemaphore);
      //Uart_Respond(resp_buf.cmd, resp_buf.status, resp_buf.buf, resp_buf.length);
    } else {
      EPT("Detected process timeout\n");
      THROW_LOG(MSG_TYPE_ERROR_LOG, "Detected process timeout\n");
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
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Invalid switch position = %u [io]\n", switch_pos);
        continue;
      }

      if ((status = osMutexAcquire(swMutex, 50)) != osOK) {
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of sw failed\n");
        continue;
      }

      Clear_Switch_Ready(switch_channel);
      if (Set_Switch(switch_channel, switch_pos)) {
        run_status.tx_switch_channel = 0xFF;
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Set Switch failed in io mode\n");
        osMutexRelease(swMutex);
        continue;
      }

      run_status.tx_switch_channel = switch_pos;
      //osDelay(pdMS_TO_TICKS(4));

      // Check
      if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
        Reset_Switch(switch_channel);
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Check Switch abnormal\n");
        osMutexRelease(swMutex);
        continue;
      }

      Set_Switch_Ready(switch_channel);
      osMutexRelease(swMutex);
    } else if (msg.type == MSG_TYPE_SWITCH2_ISR) {
      switch_channel = RX_SWITCH_CHANNEL;
      switch_pos = Get_Switch_Position_By_IO(switch_channel);
      EPT("Switch channel is %u, Switch position is %u\n", switch_channel, switch_pos);
      if (switch_pos >= 0x20) {
        EPT("Invalid switch position = %u [io]\n", switch_pos);
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Invalid switch position = %u [io]\n", switch_pos);
        continue;
      }

      if ((status = osMutexAcquire(swMutex, 50)) != osOK) {
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of sw failed\n");
        continue;
      }

      Clear_Switch_Ready(switch_channel);
      if (Set_Switch(switch_channel, switch_pos)) {
        run_status.rx_switch_channel = 0xFF;
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Set Switch failed in io mode\n");
        osMutexRelease(swMutex);
        continue;
      }

      run_status.rx_switch_channel = switch_pos;
      // osDelay(pdMS_TO_TICKS(4));

      // Check
      if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
        Reset_Switch(switch_channel);
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Check Switch abnormal\n");
        osMutexRelease(swMutex);
        continue;
      }

      Set_Switch_Ready(switch_channel);
      osMutexRelease(swMutex);
    } else if (msg.type == MSG_TYPE_SELF_CHECK_SWITCH) {
      if (run_status.osc_status == OSC_ONGOING) {
        msg.type = MSG_TYPE_SELF_TEST_STEP_2;
        msg.pbuf = NULL;
        // Switch Loopback
        Clear_Switch_Ready(TX_SWITCH_CHANNEL);
        if (Set_Switch(TX_SWITCH_CHANNEL, 64)) {
          run_status.tx_switch_channel = 0xFF;
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Tx Switch failed\n");
          run_status.osc_status = OSC_FAILURE;
          osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
          continue;
        }
        run_status.tx_switch_channel = 64;
        // Check
        if (Get_Current_Switch_Channel(TX_SWITCH_CHANNEL) != 64) {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Tx Switch failed 2\n");
          run_status.osc_status = OSC_FAILURE;
          osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
          continue;
        }

        Clear_Switch_Ready(RX_SWITCH_CHANNEL);
        if (Set_Switch(RX_SWITCH_CHANNEL, 32)) {
          run_status.rx_switch_channel = 0xFF;
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed\n");
          run_status.osc_status = OSC_FAILURE;
          osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
          continue;
        }
        run_status.rx_switch_channel = 32;
        // Check
        if (Get_Current_Switch_Channel(RX_SWITCH_CHANNEL) != 32) {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed 2\n");
          run_status.osc_status = OSC_FAILURE;
          osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
          continue;
        }

        osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
      } else if (run_status.osc_status == OSC_SUCCESS) {
        Clear_Switch_Ready(TX_SWITCH_CHANNEL);
        if (Set_Switch(TX_SWITCH_CHANNEL, 0)) {
          run_status.tx_switch_channel = 0xFF;
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Tx Switch failed\n");
          osMutexRelease(swMutex);
          continue;
        }
        run_status.tx_switch_channel = 0;
        // Check
        if (Get_Current_Switch_Channel(TX_SWITCH_CHANNEL) != 0) {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Tx Switch failed 2\n");
          osMutexRelease(swMutex);
          continue;
        }
        Set_Switch_Ready(TX_SWITCH_CHANNEL);

        Clear_Switch_Ready(RX_SWITCH_CHANNEL);
        if (Set_Switch(RX_SWITCH_CHANNEL, 0)) {
          run_status.rx_switch_channel = 0xFF;
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed\n");
          osMutexRelease(swMutex);
          continue;
        }
        run_status.rx_switch_channel = 0;
        // Check
        if (Get_Current_Switch_Channel(RX_SWITCH_CHANNEL) != 0) {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed 2\n");
          osMutexRelease(swMutex);
          continue;
        }
        Set_Switch_Ready(RX_SWITCH_CHANNEL);
        osMutexRelease(swMutex);
      } else if (run_status.osc_status == OSC_FAILURE) {
        Clear_Switch_Ready(RX_SWITCH_CHANNEL);
        if (Set_Switch(RX_SWITCH_CHANNEL, 0)) {
          run_status.rx_switch_channel = 0xFF;
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed\n");
          osMutexRelease(swMutex);
          continue;
        }
        run_status.rx_switch_channel = 0;
        // Check
        if (Get_Current_Switch_Channel(RX_SWITCH_CHANNEL) != 0) {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "Set Rx Switch failed 2\n");
          osMutexRelease(swMutex);
          continue;
        }
        Set_Switch_Ready(RX_SWITCH_CHANNEL);
        osMutexRelease(swMutex);
      }
    }
  }
}

void swIsrTask(void *argument)
{
  osStatus_t status;
  MsgStruct msg;

  for (;;)
  {
    status = osMessageQueueGet(mid_SwISR, &msg, 0U, osWaitForever);
    if (status != osOK)
      continue;

    if (msg.type == MSG_TYPE_SWITCH_DAC_ISR) {
      if (sw_tim_control.cur_x < sw_tim_control.dst_x) {
        sw_tim_control.cur_x = (sw_tim_control.dst_x - sw_tim_control.cur_x > sw_tim_control.step) ?\
                                (sw_tim_control.cur_x + sw_tim_control.step) : (sw_tim_control.dst_x);
      } else if (sw_tim_control.cur_x > sw_tim_control.dst_x) {
        sw_tim_control.cur_x = (sw_tim_control.cur_x - sw_tim_control.dst_x > sw_tim_control.step) ?\
                                (sw_tim_control.cur_x - sw_tim_control.step) : (sw_tim_control.dst_x);
      }
      
      if (sw_tim_control.cur_y < sw_tim_control.dst_y) {
        sw_tim_control.cur_y = (sw_tim_control.dst_y - sw_tim_control.cur_y > sw_tim_control.step) ?\
                                (sw_tim_control.cur_y + sw_tim_control.step) : (sw_tim_control.dst_y);
      } else if (sw_tim_control.cur_y > sw_tim_control.dst_y) {
        sw_tim_control.cur_y = (sw_tim_control.cur_y - sw_tim_control.dst_y > sw_tim_control.step) ?\
                                (sw_tim_control.cur_y - sw_tim_control.step) : (sw_tim_control.dst_y);
      }

      set_sw_dac_2(sw_tim_control.sw_num, sw_tim_control.cur_x, sw_tim_control.cur_y);

      if (sw_tim_control.cur_x == sw_tim_control.dst_x && sw_tim_control.cur_y == sw_tim_control.dst_y) {
        osSemaphoreRelease(switchSemaphore);
      } else {
        HAL_TIM_Base_Start_IT(&htim6);
      }
    }
  }
}

void logTask(void *argument)
{
  MsgStruct log_msg;
  osStatus_t status;
  uint32_t write_length;
  uint8_t remainder;

  uint32_t error_overflow_addr;
  uint32_t error_next_sector_addr;
  uint8_t error_next_sector;

  uint32_t normal_overflow_addr;
  uint32_t normal_next_sector_addr;
  uint8_t normal_next_sector;

  osDelay(pdMS_TO_TICKS(500));

  if (log_file_state.error_cur_sector == ERROR_LOG_FIRST_SECTOR + error_file_flash_count - 1) {
    error_overflow_addr = error_file_flash_end + 1;
    error_next_sector_addr = error_file_flash_addr[0];
    error_next_sector = ERROR_LOG_FIRST_SECTOR;
  } else {
    error_next_sector_addr = error_overflow_addr = error_file_flash_addr[log_file_state.error_cur_sector - ERROR_LOG_FIRST_SECTOR + 1];
    error_next_sector = log_file_state.error_cur_sector + 1;
  }

  if (log_file_state.normal_cur_sector == NORMAL_LOG_FIRST_SECTOR + normal_file_flash_count - 1) {
    normal_overflow_addr = normal_file_flash_end + 1;
    normal_next_sector_addr = normal_file_flash_addr[0];
    normal_next_sector = NORMAL_LOG_FIRST_SECTOR;
  } else {
    normal_next_sector_addr = normal_overflow_addr = normal_file_flash_addr[log_file_state.normal_cur_sector - NORMAL_LOG_FIRST_SECTOR + 1];
    normal_next_sector = log_file_state.normal_cur_sector + 1;
  }

  for (;;)
  {
    status = osMessageQueueGet(mid_LogMsg, &log_msg, 0U, osWaitForever);
    if (status != osOK)
      continue;

    if (log_msg.type == MSG_TYPE_ERROR_LOG) {
      if (log_file_state.error_offset >= error_file_flash_end + 1 || log_file_state.error_offset < error_file_flash_addr[0]) {
        Set_Flag(&run_status.internal_exp, INT_EXP_LOG_TASK);
        continue;
      }

      // The final amount of data written to flash must be an integer multiple of 4
      remainder = log_msg.length % 4;
      if (remainder) remainder = 4 - remainder;
      if (log_file_state.error_offset + log_msg.length + remainder >= error_overflow_addr) {
        // 1. fill the rest space with ' '
        Log_Write_byte(log_file_state.error_offset, ' ', error_overflow_addr - log_file_state.error_offset - 4);
        Log_Write_byte(log_file_state.error_offset + error_overflow_addr - log_file_state.error_offset - 4, '\n', 4);

        // 2. erase flash
        while (flash_in_use) {
          osDelay(pdMS_TO_TICKS(200));
        }
        //THROW_LOG("Log space overflow, erasing flash sector %u\n", next_sector);
        osSemaphoreAcquire(logEraseSemaphore, 0);
        if (FLASH_If_Erase_IT(error_next_sector) == FLASHIF_OK) {
          flash_in_use = 1;
        } else {
          Set_Flag(&run_status.internal_exp, INT_EXP_LOG_ERASE);
        }

        // 3. waiting for completion
        osSemaphoreAcquire(logEraseSemaphore, osWaitForever);

        // 4. update log_file_state, overflow_addr and next_***s
        log_file_state.error_offset = error_next_sector_addr;
        log_file_state.error_cur_sector = error_next_sector;
        if (log_file_state.error_cur_sector == ERROR_LOG_FIRST_SECTOR + error_file_flash_count - 1) {
          error_overflow_addr = error_file_flash_end + 1;
          error_next_sector_addr = error_file_flash_addr[0];
          error_next_sector = ERROR_LOG_FIRST_SECTOR;
        } else {
          error_next_sector_addr = error_overflow_addr = error_file_flash_addr[log_file_state.error_cur_sector - ERROR_LOG_FIRST_SECTOR + 1];
          error_next_sector = log_file_state.error_cur_sector + 1;
        }
        if (log_file_state.error_offset == log_file_state.error_header) {
          log_file_state.error_header = error_next_sector_addr;
        }
      }

      EPT("Writing log to flash address %#X, header = %#X\n", log_file_state.offset, log_file_state.header);
      while (flash_in_use) {
        EPT("Flash in using...\n");
        osDelay(pdMS_TO_TICKS(200));
      }
      write_length = Log_Write(log_file_state.error_offset, log_msg.pbuf, log_msg.length);
      log_file_state.error_offset += write_length;
    } else if (log_msg.type == MSG_TYPE_NORMAL_LOG) {
      if (log_file_state.normal_offset >= normal_file_flash_end + 1 || log_file_state.normal_offset < normal_file_flash_addr[0]) {
        Set_Flag(&run_status.internal_exp, INT_EXP_LOG_TASK);
        continue;
      }

      // The final amount of data written to flash must be an integer multiple of 4
      remainder = log_msg.length % 4;
      if (remainder) remainder = 4 - remainder;
      if (log_file_state.normal_offset + log_msg.length + remainder >= normal_overflow_addr) {
        // 1. fill the rest space with ' '
        Log_Write_byte(log_file_state.normal_offset, ' ', normal_overflow_addr - log_file_state.normal_offset - 4);
        Log_Write_byte(log_file_state.normal_offset + normal_overflow_addr - log_file_state.normal_offset - 4, '\n', 4);

        // 2. erase flash
        while (flash_in_use) {
          osDelay(pdMS_TO_TICKS(200));
        }
        //THROW_LOG("Log space overflow, erasing flash sector %u\n", next_sector);
        osSemaphoreAcquire(logEraseSemaphore, 0);
        if (FLASH_If_Erase_IT(normal_next_sector) == FLASHIF_OK) {
          flash_in_use = 1;
        } else {
          Set_Flag(&run_status.internal_exp, INT_EXP_LOG_ERASE);
        }

        // 3. waiting for completion
        osSemaphoreAcquire(logEraseSemaphore, osWaitForever);

        // 4. update log_file_state, overflow_addr and next_***s
        log_file_state.normal_offset = normal_next_sector_addr;
        log_file_state.normal_cur_sector = normal_next_sector;
        if (log_file_state.normal_cur_sector == NORMAL_LOG_FIRST_SECTOR + normal_file_flash_count - 1) {
          normal_overflow_addr = normal_file_flash_end + 1;
          normal_next_sector_addr = normal_file_flash_addr[0];
          normal_next_sector = NORMAL_LOG_FIRST_SECTOR;
        } else {
          normal_next_sector_addr = normal_overflow_addr = normal_file_flash_addr[log_file_state.normal_cur_sector - NORMAL_LOG_FIRST_SECTOR + 1];
          normal_next_sector = log_file_state.normal_cur_sector + 1;
        }
        if (log_file_state.normal_offset == log_file_state.normal_header) {
          log_file_state.normal_header = normal_next_sector_addr;
        }
      }

      EPT("Writing log to flash address %#X, header = %#X\n", log_file_state.offset, log_file_state.header);
      while (flash_in_use) {
        EPT("Flash in using...\n");
        osDelay(pdMS_TO_TICKS(200));
      }
      write_length = Log_Write(log_file_state.normal_offset, log_msg.pbuf, log_msg.length);
      log_file_state.normal_offset += write_length;
    } else {
      Set_Flag(&run_status.internal_exp, INT_EXP_LOG_TASK);
      continue;
    }
    
    vPortFree(log_msg.pbuf);
    
    if (osMessageQueueGetCount(mid_LogMsg) == 0) {
      // update log_file_state to eeprom/flash
      Update_Log_Status(&log_file_state);
    }
  }
}

void monitorTask(void *argument)
{
  osStatus_t status;
  uint16_t value;
  uint8_t ret;
  double voltage, temp;
  MsgStruct msg;

  osDelay(pdMS_TO_TICKS(500));

  if (Set_Switch(RX_SWITCH_CHANNEL, 0)) {
    run_status.rx_switch_channel = 0xFF;
  }
  run_status.rx_switch_channel = 0;
  // Check
  if (Get_Current_Switch_Channel(RX_SWITCH_CHANNEL) != 0) {
    Reset_Switch(RX_SWITCH_CHANNEL);
  } else {
    Set_Switch_Ready(RX_SWITCH_CHANNEL);
  }

  device_busy = 0;

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
      if (!run_status.tosa_enable && allow_tosa && !enable_tosa_failed) {
        run_status.tosa_enable = 1;
        msg.type = MSG_TYPE_LAZER_ENABLE;
        osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
      }
    } else {
      enable_tosa_failed = 0;
      if (run_status.tosa_enable) {
        run_status.tosa_enable = 0;
        msg.type = MSG_TYPE_LAZER_DISABLE;
        osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
      }
    }

    // Alarm
    status = RTOS_ADC7828_Read(VOLTAGE_3_3_CHANNEL, &value);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    } else {
      voltage = (double)value / 4096 * 2.5 * 2;
      if (!Is_Flag_Set(&run_status.exp, EXP_VOLTAGE_3_3)) {
        if (voltage > run_status.thr_table.vol_3_3_high_alarm || voltage < run_status.thr_table.vol_3_3_low_alarm) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 3.3V abnormal, current voltage = %.3lfV\n", voltage);
          Set_Flag(&run_status.exp, EXP_VOLTAGE_3_3);
        }
      } else {
        if (voltage <= run_status.thr_table.vol_3_3_high_clear && voltage >= run_status.thr_table.vol_3_3_low_clear) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 3.3V back to normal\n");
          Clear_Flag(&run_status.exp, EXP_VOLTAGE_3_3);
        }
      }
    }

    status = RTOS_ADC7828_Read(VOLTAGE_4_4_CHANNEL, &value);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    } else {
      voltage = (double)value / 4096 * 2.5 * 3;
      if (!Is_Flag_Set(&run_status.exp, EXP_VOLTAGE_4_4)) {
        if (voltage > run_status.thr_table.vol_4_4_high_alarm || voltage < run_status.thr_table.vol_4_4_low_alarm) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 4.4V abnormal, current voltage = %.3lfV\n", voltage);
          Set_Flag(&run_status.exp, EXP_VOLTAGE_4_4);
        }
      } else {
        if (voltage <= run_status.thr_table.vol_4_4_high_clear && voltage >= run_status.thr_table.vol_4_4_low_clear) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 4.4V back to normal\n");
          Clear_Flag(&run_status.exp, EXP_VOLTAGE_4_4);
        }
      }
    }

    status = RTOS_ADC7828_Read(VOLTAGE_5_0_CHANNEL, &value);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
      value = 0;
    } else {
      voltage = (double)value / 4096 * 2.5 * 3;
      if (!Is_Flag_Set(&run_status.exp, EXP_VOLTAGE_5_0)) {
        if (voltage > run_status.thr_table.vol_5_0_high_alarm || voltage < run_status.thr_table.vol_5_0_low_alarm) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 5V abnormal, current voltage = %.3lfV\n", voltage);
          Set_Flag(&run_status.exp, EXP_VOLTAGE_5_0);
        }
      } else {
        if (voltage <= run_status.thr_table.vol_5_0_high_clear && voltage >= run_status.thr_table.vol_5_0_low_clear) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 5V back to normal\n");
          Clear_Flag(&run_status.exp, EXP_VOLTAGE_5_0);
        }
      }
    }

    status = RTOS_ADC7828_Read(VOLTAGE_61_0_CHANNEL, &value);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
      value = 0;
    } else {
      voltage = (double)value / 4096 * 2.5 * 51;
      if (!Is_Flag_Set(&run_status.exp, EXP_VOLTAGE_61_0)) {
        if (voltage > run_status.thr_table.vol_61_0_high_alarm || voltage < run_status.thr_table.vol_61_0_low_alarm) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 64V abnormal, current voltage = %.3lfV\n", voltage);
          Set_Flag(&run_status.exp, EXP_VOLTAGE_61_0);
        }
      } else {
        if (voltage <= run_status.thr_table.vol_61_0_high_clear && voltage >= run_status.thr_table.vol_61_0_low_clear) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Voltage 64V back to normal\n");
          Clear_Flag(&run_status.exp, EXP_VOLTAGE_61_0);
        }
      }
    }

    status = RTOS_ADC7828_Read(TEMPERATURE_CHANNEL, &value);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
      value = 0;
    } else {
      temp = Cal_Temp(value);
      if (!Is_Flag_Set(&run_status.exp, EXP_TEMPERATURE)) {
        if (temp > run_status.thr_table.temp_high_alarm || temp < run_status.thr_table.temp_low_alarm) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Temperature abnormal, current temperature = %.3lfC\n", temp);
          Set_Flag(&run_status.exp, EXP_TEMPERATURE);
        }
      } else {
        if (temp <= run_status.thr_table.temp_high_clear && temp >= run_status.thr_table.temp_low_clear) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Temperature back to normal\n");
          Clear_Flag(&run_status.exp, EXP_TEMPERATURE);
        }
      }
    }

#if 1
    if (run_status.allow_monitor) {
      if (run_status.tosa_enable) {
        status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_CURRENT_CHANNEL, &value);
        if (status != osOK) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        } else {
          voltage = (double)value / 4096 * 2.5;
          voltage = (voltage - 1.25) / 0.285 * 1000;
          if (!Is_Flag_Set(&run_status.exp, EXP_TEC_CURRENT)) {
            if (voltage > run_status.thr_table.tec_cur_high_alarm || voltage < run_status.thr_table.tec_cur_low_alarm) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec current abnormal, current current = %.lfmA\n", voltage);
              Set_Flag(&run_status.exp, EXP_TEC_CURRENT);
            }
          } else {
            if (voltage <= run_status.thr_table.tec_cur_high_clear && voltage >= run_status.thr_table.tec_cur_low_clear) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec current back to normal\n");
              Clear_Flag(&run_status.exp, EXP_TEC_CURRENT);
            }
          }
        }

        status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_VOLTAGE_CHANNEL, &value);
        if (status != osOK) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        } else {
          voltage = (double)value / 4096 * 2.5;
          voltage = (voltage - 1.25) / 0.25 * 1000;
          if (!Is_Flag_Set(&run_status.exp, EXP_TEC_VOLTAGE)) {
            if (voltage > run_status.thr_table.tec_vol_high_alarm || voltage < run_status.thr_table.tec_vol_low_alarm) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec voltage abnormal, current voltage = %.lfmV\n", voltage);
              Set_Flag(&run_status.exp, EXP_TEC_VOLTAGE);
            }
          } else {
            if (voltage <= run_status.thr_table.tec_vol_high_clear && voltage >= run_status.thr_table.tec_vol_low_clear) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec voltage back to normal\n");
              Clear_Flag(&run_status.exp, EXP_TEC_VOLTAGE);
            }
          }
        }
      }
  #endif
      
      if (run_status.tosa_enable) {
        // Power
        if (!run_status.modulation) {
          if ((ret = get_tap_pd_power(&value, &voltage)) != 0) {
            if (!Is_Flag_Set(&run_status.internal_exp, INT_EXP_TAP_PD)) {
              THROW_LOG(MSG_TYPE_ERROR_LOG, "Get_Tap_Power error code = %u\n", ret);
              Set_Flag(&run_status.internal_exp, INT_EXP_TAP_PD);
            }
          } else {
            temp = run_status.tosa_dst_power_high - voltage;
            if (temp > 2 || temp < -2) {
              if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
                THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tap power = %lf, destination is %lf\n", voltage, run_status.tosa_dst_power_high);
                Set_Flag(&run_status.exp, EXP_LAZER_POWER);
              }
            } else {
              if (Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
                THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tap power = %lf back to normal\n", voltage);
                Clear_Flag(&run_status.exp, EXP_LAZER_POWER);
              }
            }
          }
        }

        // TEC LOSS LOCK
        if (HAL_GPIO_ReadPin(TMPGD_GPIO_Port, TMPGD_Pin) == GPIO_PIN_RESET) {
          if (!Is_Flag_Set(&run_status.exp, EXP_TEC_TEMP_LOSS)) {
            THROW_LOG(MSG_TYPE_NORMAL_LOG, "TEC lock loss\n");
            Set_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
          }
        } else {
          if (Is_Flag_Set(&run_status.exp, EXP_TEC_TEMP_LOSS)) {
            THROW_LOG(MSG_TYPE_NORMAL_LOG, "TEC locked\n");
            Clear_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
          }
        }
        
        // Tec Temperature
        status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_TEMP_CHANNEL, &value);
        if (status != osOK) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
          value = 0;
        } else {
          temp = Cal_Tosa_Temp(value);
          if (!Is_Flag_Set(&run_status.exp, EXP_TEC_TEMP)) {
            if (temp > run_status.thr_table.tec_temp_high_alarm || temp < run_status.thr_table.tec_temp_low_alarm) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec Temperature abnormal, current temperature = %.3lfC\n", temp);
              Set_Flag(&run_status.exp, EXP_TEC_TEMP);
            }
          } else {
            if (temp <= run_status.thr_table.tec_temp_high_clear && temp >= run_status.thr_table.tec_temp_low_clear) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tec Temperature back to normal\n");
              Clear_Flag(&run_status.exp, EXP_TEC_TEMP);
            }
          }
        }

        // LD Current
        status = RTOS_ADC7953_SPI5_Read(TEC_ADC_LD_CURRENT_CHANNEL, &value);
        if (status != osOK) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
          value = 0;
        } else {
          temp = (double)value / 4096 * 2.5;
          temp = temp / 22 * 1000;
          if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_CURRENT)) {
            if (temp > run_status.thr_table.LD_cur_high_alarm) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "LD Current abnormal, current temperature = %.3lfC\n", temp);
              Set_Flag(&run_status.exp, EXP_LAZER_CURRENT);
            }
          } else {
            if (temp <= run_status.thr_table.LD_cur_high_clear) {
              THROW_LOG(MSG_TYPE_NORMAL_LOG, "LD Current back to normal\n");
              Clear_Flag(&run_status.exp, EXP_LAZER_CURRENT);
            }
          }
        }
      }
    }


    if (run_status.exp && !run_status.pre_alarm) {
      Set_Alarm();
      if (Update_History_Alarm(run_status.exp) != osOK) {
        Set_Flag(&run_status.internal_exp, INT_EXP_UP_ALARM);
      }
      run_status.pre_alarm = 1;
      run_status.pre_exp_value = run_status.exp;
    } else if (!run_status.exp && run_status.pre_alarm) {
      Clear_Alarm();
      run_status.pre_alarm = 0;
      run_status.pre_exp_value = 0;
    } else if (run_status.exp && run_status.pre_alarm) {
      if (run_status.exp != run_status.pre_exp_value) {
        if (Update_History_Alarm(run_status.exp) != osOK) {
          Set_Flag(&run_status.internal_exp, INT_EXP_UP_ALARM);
        }
        run_status.pre_exp_value = run_status.exp;
      }
    }

    if (run_status.modulation) {
      osDelay(pdMS_TO_TICKS(1000));
    } else {
      osDelay(pdMS_TO_TICKS(300));
    }
  }
}

void lazerManagerTask(void *argument)
{
  osStatus_t status;
  uint8_t buf[2];
  uint8_t ret;
  MsgStruct msg;
  double power_cur, power_sub, power_dst;
  uint32_t i, times;

  osDelay(pdMS_TO_TICKS(500));

  for (;;)
  {
    status = osMessageQueueGet(mid_LazerManager, &msg, 0U, osWaitForever);
    if (status != osOK)
      continue;

    if (msg.type == MSG_TYPE_LAZER_ENABLE) {
      if (run_status.modulation) {
        status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, (run_status.tosa_high.tec_dac + run_status.tosa_low.tec_dac) / 2, DAC128S085_MODE_NORMAL);
        if (run_status.power_mode) {
          DAC5541_Write(run_status.tosa_high.tosa_dac);
        } else {
          DAC5541_Write(run_status.tosa_low.tosa_dac);
        }
      } else {
        status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
        DAC5541_Write(run_status.tosa_high.tosa_dac);
      }

      if ((ret = Enable_Tosa()) != 0) {
        Set_Flag(&run_status.internal_exp, INT_EXP_TOSA);
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Enable Tosa failed, ret = %u\n", ret);
        enable_tosa_failed = 1;
        continue;
      }
      osDelay(pdMS_TO_TICKS(100));

      if (!run_status.modulation) {
        if (Cali_Power(run_status.tosa_dst_power_high)) {
          allow_tosa = 0;
          run_status.tosa_enable = 0;
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Module has disabled Tosa until reset\n");
          Disable_Tosa();
          continue;
        }
      }
      
      Set_Lazer_Ready();

      i = 0;
      times = 0;
      while (i++ < 100) {
        if (Is_Tec_Lock()) {
          if (++times > 5) {
            break;
          }
        }
        osDelay(5);
      }
      if (times > 5) {
        Update_Tec_Dest_Temp(&run_status.thr_table);
      } else {
        THROW_LOG(MSG_TYPE_ERROR_LOG, "TMPGD error\n");
        Set_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
        break;
      }
      run_status.allow_monitor = 1;
    } else if (msg.type == MSG_TYPE_LAZER_DISABLE) {
      run_status.allow_monitor = 0;
      Clear_Lazer_Ready();

      Disable_Tosa();
    } else if (msg.type == MSG_TYPE_LAZER_POWER) {
      //msg.type = MSG_TYPE_CMD_PROCESS;
      //msg.pbuf = pvPortMalloc(1);
      //msg.length = 1;

      run_status.tosa_high = Get_Tosa_Data(run_status.tosa_dst_power_high);
      run_status.tosa_low = Get_Tosa_Data(run_status.tosa_dst_power_low);

      if (run_status.tosa_enable) {
        run_status.allow_monitor = 0;
        
        if (run_status.modulation) {
          status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, (run_status.tosa_high.tec_dac + run_status.tosa_low.tec_dac) / 2, DAC128S085_MODE_NORMAL);
          if (run_status.power_mode) {
            DAC5541_Write(run_status.tosa_high.tosa_dac);
          } else {
            DAC5541_Write(run_status.tosa_low.tosa_dac);
          }
        } else {
          status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
          DAC5541_Write(run_status.tosa_high.tosa_dac);
          osDelay(1);
          if (Cali_Power(run_status.tosa_dst_power_high)) {
            THROW_LOG(MSG_TYPE_NORMAL_LOG, "Module has disabled Tosa until reset\n");
            allow_tosa = 0;
            run_status.tosa_enable = 0;
            Disable_Tosa();
            // return failure
            //*(uint8_t*)msg.pbuf = 1;
            //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);
            continue;
          }
        }

        // return success
        //*(uint8_t*)msg.pbuf = 0;
        //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);

        i = 0;
        times = 0;
        while (i++ < 100) {
          if (Is_Tec_Lock()) {
            if (++times > 5) {
              break;
            }
          }
          osDelay(5);
        }
        if (times > 5) {
          Update_Tec_Dest_Temp(&run_status.thr_table);
        } else {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "TMPGD error\n");
          Set_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
          break;
        }
        run_status.allow_monitor = 1;
      } else {
        // return success
        //*(uint8_t*)msg.pbuf = 0;
        //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);
      }
    } else if (msg.type == MSG_TYPE_MODULATION_ON) {
      //msg.type = MSG_TYPE_CMD_PROCESS;
      //msg.pbuf = pvPortMalloc(1);
      //msg.length = 1;

      if (run_status.tosa_enable) {
        run_status.allow_monitor = 0;
        
        status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, (run_status.tosa_high.tec_dac + run_status.tosa_low.tec_dac) / 2, DAC128S085_MODE_NORMAL);

        run_status.modulation = 1;
        HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);

        i = 0;
        times = 0;
        while (i++ < 100) {
          if (Is_Tec_Lock()) {
            if (++times > 5) {
              break;
            }
          }
          osDelay(5);
        }
        if (times > 5) {
          Update_Tec_Dest_Temp(&run_status.thr_table);
        } else {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "TMPGD error\n");
          Set_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
          break;
        }
        run_status.allow_monitor = 1;
      } else {
        run_status.modulation = 1;
        HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
      }
      
      //*(uint8_t*)msg.pbuf = 0;
      //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);

    } else if (msg.type == MSG_TYPE_MODULATION_OFF) {
      //msg.type = MSG_TYPE_CMD_PROCESS;
      //msg.pbuf = pvPortMalloc(1);
      //msg.length = 1;

      HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_1);
      HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_3);
      
      run_status.power_mode = 1;
      
      if (run_status.tosa_enable) {
        run_status.allow_monitor = 0;
        
        status = RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
        DAC5541_Write(run_status.tosa_high.tosa_dac);
        osDelay(pdMS_TO_TICKS(1));

        if (Cali_Power(run_status.tosa_dst_power_high)) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Module has disabled Tosa until reset\n");
          allow_tosa = 0;
          run_status.tosa_enable = 0;
          Disable_Tosa();
          // return failure
          //*(uint8_t*)msg.pbuf = 1;
          //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);
          continue;
        }


        i = 0;
        times = 0;
        while (i++ < 100) {
          if (Is_Tec_Lock()) {
            if (++times > 5) {
              break;
            }
          }
          osDelay(5);
        }
        if (times > 5) {
          Update_Tec_Dest_Temp(&run_status.thr_table);
        } else {
          THROW_LOG(MSG_TYPE_ERROR_LOG, "TMPGD error\n");
          Set_Flag(&run_status.exp, EXP_TEC_TEMP_LOSS);
          break;
        }
        run_status.allow_monitor = 1;
      }
      
      run_status.modulation = 0;
      //*(uint8_t*)msg.pbuf = 0;
      //osMessageQueuePut(mid_CmdProcess, &msg, 0U, 0U);
    } else if (msg.type == MSG_TYPE_SELF_TEST) {
      if (Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
        Clear_Flag(&run_status.exp, EXP_SELFCHECK);
      }
      if (Is_Flag_Set(&run_status.exp, EXP_RX_PD)) {
        Clear_Flag(&run_status.exp, EXP_RX_PD);
      }
      if (Is_Flag_Set(&run_status.exp, EXP_SWITCH)) {
        Clear_Flag(&run_status.exp, EXP_SWITCH);
      }

      if (osMutexAcquire(swMutex, 50) != osOK) {
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of sw failed when selfcheck\n");
        continue;
      }

      run_status.osc_status = OSC_ONGOING;
      msg.type = MSG_TYPE_SELF_CHECK_SWITCH;
      msg.pbuf = NULL;
      osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
    } else if (msg.type == MSG_TYPE_SELF_TEST_STEP_2) {
      msg.type = MSG_TYPE_SELF_CHECK_SWITCH;
      msg.pbuf = NULL;
      if (run_status.osc_status == OSC_FAILURE) {
        if (!Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
          Set_Flag(&run_status.exp, EXP_SELFCHECK);
        }
        osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
        continue;
      }

      if ((ret = Get_Tap_Power(&power_cur)) != 0) {
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check failed because get tap-pd failed.\n");
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Get_Tap_Power error code = %u when self_check\n", ret);
        Set_Flag(&run_status.internal_exp, INT_EXP_TAP_PD);
        if (!Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
          Set_Flag(&run_status.exp, EXP_SELFCHECK);
        }
        run_status.osc_status = OSC_FAILURE;
        osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
        continue;
      }
      power_sub = run_status.tosa_dst_power_high - power_cur;
      // THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check Tap power = %lf, dest power is %lf\r\n", power_h, run_status.tosa_dst_power_high);
      if (power_sub > 2 || power_sub < -2) {
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check failed.\n");
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tap power = %lf, destination is %lf\n", power_cur, run_status.tosa_dst_power_high);
        if (!Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
          Set_Flag(&run_status.exp, EXP_SELFCHECK);
        }
        if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
          Set_Flag(&run_status.exp, EXP_LAZER_POWER);
        }
        run_status.osc_status = OSC_FAILURE;
        osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
        continue;
      }

      if ((ret = Get_Rx_Power(&power_cur)) != 0) {
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check failed because get rx-pd failed.\n");
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Get_Rx_Power error code = %u when self_check\n", ret);
        Set_Flag(&run_status.internal_exp, INT_EXP_RX_PD);
        if (!Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
          Set_Flag(&run_status.exp, EXP_SELFCHECK);
        }
        run_status.osc_status = OSC_FAILURE;
        osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
        continue;
      }
      status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_LB_IL + 2, buf, 2);
      // power_dst is loopback insertion loss vale
      power_dst = (double)(int16_t)Buffer_To_BE16(buf) / 100;
      power_sub = run_status.tosa_dst_power_high - (power_cur + power_dst);
      // THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check Rx power = %lf, IL = %lf, dest power is %lf, \r\n", power_h, power_dst, run_status.tosa_dst_power_high);
      if (power_sub > 3 || power_sub < -3) {
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Self-check failed.\n");
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Rx power = %lf, IL = %lf, destination is %lf\n", power_cur, power_dst, run_status.tosa_dst_power_high);
        if (power_sub > 60 || power_sub < -60) {
          if (!Is_Flag_Set(&run_status.exp, EXP_SWITCH)) {
            Set_Flag(&run_status.exp, EXP_SWITCH);
          }
        } else {
          if (!Is_Flag_Set(&run_status.exp, EXP_RX_PD)) {
            Set_Flag(&run_status.exp, EXP_RX_PD);
          }
        }
        if (!Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
          Set_Flag(&run_status.exp, EXP_SELFCHECK);
        }
        run_status.osc_status = OSC_FAILURE;
        osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
        continue;
      }

      if (Is_Flag_Set(&run_status.exp, EXP_SELFCHECK)) {
        Clear_Flag(&run_status.exp, EXP_SELFCHECK);
      }
      run_status.osc_status = OSC_SUCCESS;
      osMessageQueuePut(mid_ISR, &msg, 0U, 0U);
    }
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
