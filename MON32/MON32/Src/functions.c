#include "main.h"
#include "command.h"
#include "flash_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "i2c.h"
#include "spi.h"
#include "rtc.h"
#include "tim.h"

extern osSemaphoreId_t switchSemaphore;

ChannelSwitchMapStruct channel_map[] = {
  {TX_SWITCH_CHANNEL,  1, 11, SWITCH_NUM_1, EE_CAL_SWITCH1, SWITCH_NUM_2, EE_CAL_SWITCH2},
  {TX_SWITCH_CHANNEL, 12, 22, SWITCH_NUM_1, EE_CAL_SWITCH1, SWITCH_NUM_3, EE_CAL_SWITCH3},
  {TX_SWITCH_CHANNEL, 23, 33, SWITCH_NUM_1, EE_CAL_SWITCH1, SWITCH_NUM_4, EE_CAL_SWITCH4},
  {RX_SWITCH_CHANNEL,  1, 11, SWITCH_NUM_5, EE_CAL_SWITCH5, SWITCH_NUM_6, EE_CAL_SWITCH6},
  {RX_SWITCH_CHANNEL, 12, 22, SWITCH_NUM_5, EE_CAL_SWITCH5, SWITCH_NUM_7, EE_CAL_SWITCH7},
  {RX_SWITCH_CHANNEL, 23, 33, SWITCH_NUM_5, EE_CAL_SWITCH5, SWITCH_NUM_8, EE_CAL_SWITCH8},
};
uint32_t channel_map_count = sizeof(channel_map) / sizeof(channel_map[0]);

SwitchDacMapStruct switch_map[] = {
  {SWITCH_NUM_1, /*DAC channel*/  0,  8,  3,  2, /*ADC channel*/  0,  1,  2,  3},
  {SWITCH_NUM_2, /*DAC channel*/  5,  7,  6,  9, /*ADC channel*/  4,  5,  6,  7},
  {SWITCH_NUM_3, /*DAC channel*/ 12,  4,  1, 10, /*ADC channel*/  8,  9, 10, 11},
  {SWITCH_NUM_4, /*DAC channel*/ 11, 16, 13, 14, /*ADC channel*/ 12, 13, 14, 15},
  {SWITCH_NUM_5, /*DAC channel*/ 15, 17, 20, 18, /*ADC channel*/ 16, 17, 18, 19},
  {SWITCH_NUM_6, /*DAC channel*/ 21, 25, 22, 23, /*ADC channel*/ 20, 21, 22, 23},
  {SWITCH_NUM_7, /*DAC channel*/ 24, 19, 26, 27, /*ADC channel*/ 24, 25, 26, 27},
  {SWITCH_NUM_8, /*DAC channel*/ 28, 29, 30, 31, /*ADC channel*/ 28, 29, 30, 31}
};

extern osMessageQueueId_t mid_LogMsg;
extern RespondStu resp_buf;

#define LOG_HEADER "%04u-%02u-%02u %02u:%02u:%02u : "

void Throw_Log(uint8_t type, uint8_t *buf, uint32_t length)
{
  MsgStruct log_msg;
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
#ifdef PRINT_DEBUG_MESSAGE
  osStatus_t status;
#endif

  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

  log_msg.pbuf = pvPortMalloc(length + 32);
  sprintf(log_msg.pbuf, LOG_HEADER, 2000 + date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
  log_msg.length = strlen((char*)log_msg.pbuf);
  memcpy((char*)log_msg.pbuf + log_msg.length, buf, length);
  log_msg.length += length;
  log_msg.type = type;
#ifdef PRINT_DEBUG_MESSAGE
  if ((status = osMessageQueuePut(mid_LogMsg, &log_msg, 0U, 0U)) != osOK) {
    EPT("Put Message to Queue failed, status = %d\n", status);
  }
#else
  osMessageQueuePut(mid_LogMsg, &log_msg, 0U, 0U);
#endif
}

uint32_t Log_Write(uint32_t addr, uint8_t *pbuf, uint32_t length)
{
  uint32_t w_len = 0, data, i;
  uint8_t remainder = length % 4;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  while (w_len < length) {
    if (w_len + 4 > length) {
      for (i = 0, data = 0; i < 4; ++i) {
        data |= (i < remainder ? pbuf[w_len + i] : i == 3 ? '\n' : ' ') << i * 8;
      }
    } else {
      memcpy(&data, pbuf + w_len, 4);
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)data) != HAL_OK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_LOG_PROGRAM);
      break;
    }
    addr += 4;
    w_len += 4;
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return w_len;
}

uint32_t Log_Write_byte(uint32_t addr, uint8_t ch, uint32_t length)
{
  uint32_t w_len = 0, data;

  if (ch == '\n' && length == 4) {
    data = ch << 24 | ' ' << 16 | ' ' << 8 | ' ';
  } else {
    data = ch << 24 | ch << 16 | ch << 8 | ch;
  }
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  while (w_len < length) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)data);
    addr += 4;
    w_len += 4;
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return w_len;
}

uint32_t Log_Read(uint32_t addr, uint8_t *pbuf, uint32_t length)
{
  memcpy(pbuf, (void*)addr, length);

  return length;
}

osStatus_t Get_Log_Status(LogFileState *log_status)
{
  osStatus_t status;
  uint8_t buf[32];

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_LOG_MAGIC, buf, sizeof(buf));
  if (status != osOK) {
    return status;
  }

  log_status->magic = Buffer_To_BE32(&buf[EE_LOG_MAGIC - EE_LOG_MAGIC]);
  log_status->magic_2 = Buffer_To_BE32(&buf[EE_LOG_MAGIC_2 - EE_LOG_MAGIC]);
  if (log_status->magic_2 != LOG_MAGIC) {
    log_status->magic = LOG_MAGIC;
    log_status->magic_2 = LOG_MAGIC;
    log_status->error_offset = error_file_flash_addr[0];
    log_status->error_header = error_file_flash_addr[0];
    log_status->normal_offset = normal_file_flash_addr[0];
    log_status->normal_header = normal_file_flash_addr[0];
    return Update_Log_Status(log_status);
  } else {
    log_status->error_offset = Buffer_To_BE32(&buf[EE_ERROR_LOG_OFFSET - EE_LOG_MAGIC]);
    log_status->error_header = Buffer_To_BE32(&buf[EE_ERROR_LOG_HEADER - EE_LOG_MAGIC]);
    log_status->normal_offset = Buffer_To_BE32(&buf[EE_NORMAL_LOG_OFFSET - EE_LOG_MAGIC]);
    log_status->normal_header = Buffer_To_BE32(&buf[EE_NORMAL_LOG_HEADER - EE_LOG_MAGIC]);
    return status;
  }
}

osStatus_t Update_Log_Status(LogFileState *log_status)
{
  uint8_t buf[32] = {0xFF};
  
  BE32_To_Buffer(log_status->magic, &buf[EE_LOG_MAGIC - EE_LOG_MAGIC]);
  BE32_To_Buffer(log_status->error_offset, &buf[EE_ERROR_LOG_OFFSET - EE_LOG_MAGIC]);
  BE32_To_Buffer(log_status->error_header, &buf[EE_ERROR_LOG_HEADER - EE_LOG_MAGIC]);
  BE32_To_Buffer(log_status->normal_offset, &buf[EE_NORMAL_LOG_OFFSET - EE_LOG_MAGIC]);
  BE32_To_Buffer(log_status->normal_header, &buf[EE_NORMAL_LOG_HEADER - EE_LOG_MAGIC]);
  BE32_To_Buffer(log_status->magic_2, &buf[EE_LOG_MAGIC_2 - EE_LOG_MAGIC]);

  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_LOG_MAGIC, buf, sizeof(buf));
}

osStatus_t Reset_Log_Status()
{
  uint8_t buf[32] = {0xFF};

  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_LOG_MAGIC, buf, sizeof(buf));
}

osStatus_t Get_Up_Status(UpgradeFlashState *up_status)
{
  osStatus_t status;
  uint8_t buf[EE_UP_FACTORY_CRC32 + 4 - EE_UP_MAGIC];

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_UP_MAGIC, buf, sizeof(buf));
  if (status != osOK) {
    return status;
  }
  
  up_status->magic = Buffer_To_BE32(&buf[EE_UP_MAGIC - EE_UP_MAGIC]);
  up_status->run = buf[EE_UP_RUN - EE_UP_MAGIC];
  up_status->flash_empty = buf[EE_UP_FLASH_EMPTY - EE_UP_MAGIC];
  up_status->length = Buffer_To_BE32(&buf[EE_UP_LENGTH - EE_UP_MAGIC]);
  up_status->crc32 = Buffer_To_BE32(&buf[EE_UP_CRC32 - EE_UP_MAGIC]);
  up_status->factory_length = Buffer_To_BE32(&buf[EE_UP_FACTORY_LENGTH - EE_UP_MAGIC]);
  up_status->factory_crc32 = Buffer_To_BE32(&buf[EE_UP_FACTORY_CRC32 - EE_UP_MAGIC]);
  return status;
}

osStatus_t Update_Up_Status(UpgradeFlashState *up_status)
{
  uint8_t buf[EE_UP_FACTORY_CRC32 + 4 - EE_UP_MAGIC];
  
  BE32_To_Buffer(up_status->magic, &buf[EE_UP_MAGIC - EE_UP_MAGIC]);
  buf[EE_UP_RUN - EE_UP_MAGIC] = up_status->run;
  buf[EE_UP_FLASH_EMPTY - EE_UP_MAGIC] = up_status->flash_empty;
  BE32_To_Buffer(up_status->length, &buf[EE_UP_LENGTH - EE_UP_MAGIC]);
  BE32_To_Buffer(up_status->crc32, &buf[EE_UP_CRC32 - EE_UP_MAGIC]);
  BE32_To_Buffer(up_status->factory_length, &buf[EE_UP_FACTORY_LENGTH - EE_UP_MAGIC]);
  BE32_To_Buffer(up_status->factory_crc32, &buf[EE_UP_FACTORY_CRC32 - EE_UP_MAGIC]);

  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_UP_MAGIC, buf, sizeof(buf));
}

osStatus_t Reset_Up_Status()
{
  uint8_t buf[4] = {0};

  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_UP_MAGIC, buf, sizeof(buf));
}

osStatus_t Get_Threshold_Table(ThresholdStruct *table)
{
  table->vol_3_3_high_alarm = 3.3 * 1.06;
  table->vol_3_3_high_clear = 3.3 * 1.05;
  table->vol_3_3_low_alarm = 3.3 * 0.94;
  table->vol_3_3_low_clear = 3.3 * 0.95;

  table->vol_4_4_high_alarm = 4.4 * 1.06;
  table->vol_4_4_high_clear = 4.4 * 1.05;
  table->vol_4_4_low_alarm = 4.4 * 0.94;
  table->vol_4_4_low_clear = 4.4 * 0.95;

  table->vol_5_0_high_alarm = 5 * 1.06;
  table->vol_5_0_high_clear = 5 * 1.05;
  table->vol_5_0_low_alarm = 5 * 0.94;
  table->vol_5_0_low_clear = 5 * 0.95;

  table->vol_61_0_high_alarm = 64 * 1.06;
  table->vol_61_0_high_clear = 64 * 1.05;
  table->vol_61_0_low_alarm = 64 * 0.94;
  table->vol_61_0_low_clear = 64 * 0.95;

  table->temp_high_alarm = 85 * 1.01;
  table->temp_high_clear = 85 * 1.00;
  table->temp_low_alarm = -10 * 1.01;
  table->temp_low_clear = -10 * 1.00;

  table->tec_cur_high_alarm = 600;
  table->tec_cur_high_clear = 550;
  table->tec_cur_low_alarm = -600;
  table->tec_cur_low_clear = -550;

  table->tec_vol_high_alarm = 1800;
  table->tec_vol_high_clear = 1750;
  table->tec_vol_low_alarm = -1800;
  table->tec_vol_low_clear = -1750;

  table->tec_temp_high_alarm = 0;
  table->tec_temp_high_clear = 0;
  table->tec_temp_low_alarm = 0;
  table->tec_temp_low_clear = 0;

  table->LD_cur_high_alarm = 88;
  table->LD_cur_high_clear = 80;

  return osOK;
}

osStatus_t Update_Tec_Dest_Temp(ThresholdStruct *table)
{
  osStatus_t status;
  uint16_t value, i = 0, times = 0;
  double temp, temp_first;

  while (times++ < 30 && i < 5) {
    status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_TEMP_CHANNEL, &value);
    if (status != osOK) {
      return status;
    }
    temp = Cal_Tosa_Temp(value);

    if (i == 0) {
      temp_first = temp;
      i++;
    } else {
      if (temp >= temp_first - 0.1 && temp <= temp_first + 0.1) {
        i++;
      } else {
        i = 0;
      }
    }

    osDelay(10);
  }
  
  // THROW_LOG(MSG_TYPE_NORMAL_LOG, "Get Temp times = %u, i = %u, temp = %.1lf\n", times, i, temp);

  table->tec_temp_high_alarm = temp + 5;
  table->tec_temp_high_clear = temp + 4.95;
  table->tec_temp_low_alarm = temp - 5;
  table->tec_temp_low_clear = temp - 4.95;

  return status;
}

osStatus_t Reset_Tec_Dest_Temp(ThresholdStruct *table)
{
  osStatus_t status;
  int32_t value;
  double temp;

  status = get_32_from_eeprom(EEPROM_ADDR, EE_TEC_DEF_TEMP, (uint32_t*)&value);
  if (status != osOK) {
    return status;
  }
  temp = (double)value / 10;
  if (temp > 100 || temp < -50) {
    temp = 35;
  }

  table->tec_temp_high_alarm = temp + 5;
  table->tec_temp_high_clear = temp + 4.95;
  table->tec_temp_low_alarm = temp - 5;
  table->tec_temp_low_clear = temp - 4.95;

  return status;
}

void Check_Cali(void)
{
  uint16_t start = 0x1000;
  uint16_t end = EE_PARA_TABLE_END;
  uint16_t offset;
  uint8_t buf[4];
  uint32_t crc32 = 0xFFFFFFFF;
  uint32_t chk;
  
  if (get_32_from_eeprom(EEPROM_ADDR, EE_CALI_CHECK, &chk) != osOK) {
    return;
  }

  for (offset = start; offset < end; offset += 4) {
    if (RTOS_EEPROM_Read(EEPROM_ADDR, offset, buf, 4) != osOK) {
      return ;
    }
    
    crc32 = Cal_CRC32_2(buf, 4, crc32);
  }
  
  crc32 = ~crc32;
  
  if (crc32 != chk) {
    if (!Is_Flag_Set(&run_status.exp, EXP_CALI_CHK)) {
      Set_Flag(&run_status.exp, EXP_CALI_CHK);
    }
  } else {
    if (Is_Flag_Set(&run_status.exp, EXP_CALI_CHK)) {
      Clear_Flag(&run_status.exp, EXP_CALI_CHK);
    }
  }
  
  return;
}

osStatus_t Get_EEPROM_Alarm_Status(AlarmHistoryState *alarm)
{
  osStatus_t status;
  uint8_t buf[6];

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_ALARM_MAGIC, buf, sizeof(buf));
  if (status != osOK) {
    return status;
  }

  alarm->magic = Buffer_To_BE32(&buf[0]);
  if (alarm->magic != ALARM_MAGIC) {
    alarm->magic = ALARM_MAGIC;
    alarm->start = 0;
    alarm->end = 0;
    return Update_EEPROM_Alarm_Status(alarm);
  } else {
    alarm->start = buf[4];
    alarm->end = buf[5];
  }

  return status;
}

osStatus_t Update_EEPROM_Alarm_Status(AlarmHistoryState *alarm)
{
  uint8_t buf[6];

  BE32_To_Buffer(alarm->magic, &buf[0]);
  buf[4] = alarm->start;
  buf[5] = alarm->end;

  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_ALARM_MAGIC, buf, sizeof(buf));
}

osStatus_t Reset_EEPROM_Alarm_Status()
{
  uint8_t buf[4] = {0};

  history_alarm_status.magic = 0;
  return RTOS_EEPROM_Write(EEPROM_ADDR, EE_ALARM_MAGIC, buf, sizeof(buf));
}

osStatus_t Update_History_Alarm(uint32_t exp)
{
  uint8_t buf[4];
  osStatus_t status;

  if (history_alarm_status.magic != ALARM_MAGIC) {
    return osError;
  }

  if (history_alarm_status.start == 0) {
    history_alarm_status.start = 1;
    history_alarm_status.end = 1;
  } else if (history_alarm_status.end < history_alarm_status.start || history_alarm_status.end == 10) {
    history_alarm_status.end = (history_alarm_status.end % 10) + 1;
    history_alarm_status.start = (history_alarm_status.end % 10) + 1;
  } else {
    history_alarm_status.end = (history_alarm_status.end % 10) + 1;
  }
  
  BE32_To_Buffer(run_status.exp, buf);
  status = RTOS_EEPROM_Write(EEPROM_ADDR, EE_ALARM_HISTORY + (history_alarm_status.end - 1) * 4, buf, sizeof(buf));
  status |= Update_EEPROM_Alarm_Status(&history_alarm_status);

  return status;
}

uint8_t Get_Switch_Position_By_IO(uint8_t switch_channel)
{
  uint8_t val = 0;

  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (HAL_GPIO_ReadPin(SW1_D5_GPIO_Port, SW1_D5_Pin) == GPIO_PIN_SET)
      val |= 1 << 4;
    if (HAL_GPIO_ReadPin(SW1_D4_GPIO_Port, SW1_D4_Pin) == GPIO_PIN_SET)
      val |= 1 << 3;
    if (HAL_GPIO_ReadPin(SW1_D3_GPIO_Port, SW1_D3_Pin) == GPIO_PIN_SET)
      val |= 1 << 2;
    if (HAL_GPIO_ReadPin(SW1_D2_GPIO_Port, SW1_D2_Pin) == GPIO_PIN_SET)
      val |= 1 << 1;
    if (HAL_GPIO_ReadPin(SW1_D1_GPIO_Port, SW1_D1_Pin) == GPIO_PIN_SET)
      val |= 1 << 0;

    if (HAL_GPIO_ReadPin(SW1_D0_GPIO_Port, SW1_D0_Pin) == GPIO_PIN_SET)
      val += 0x20;
  } else {
    if (HAL_GPIO_ReadPin(SW2_D4_GPIO_Port, SW2_D4_Pin) == GPIO_PIN_SET)
      val |= 1 << 4;
    if (HAL_GPIO_ReadPin(SW2_D3_GPIO_Port, SW2_D3_Pin) == GPIO_PIN_SET)
      val |= 1 << 3;
    if (HAL_GPIO_ReadPin(SW2_D2_GPIO_Port, SW2_D2_Pin) == GPIO_PIN_SET)
      val |= 1 << 2;
    if (HAL_GPIO_ReadPin(SW2_D1_GPIO_Port, SW2_D1_Pin) == GPIO_PIN_SET)
      val |= 1 << 1;
    if (HAL_GPIO_ReadPin(SW2_D0_GPIO_Port, SW2_D0_Pin) == GPIO_PIN_SET)
      val |= 1 << 0;
  }
  
  return val;
}

/* 
 *  switch_pos
 *      channel = 0
 *        0-31: 2*32Switch 1 to 32
 *        32-63: 2*32Switch 2 to 32
 *        64: 2*32Switch 1 to 33
 *        65: 2*32Switch 2 to 33
 *      channel = 1
 *        0-31: 1*32Switch 1 to 32
 *        32: 1*32Switch 1 to 33
 */
int8_t Set_Switch(uint8_t switch_channel, uint8_t switch_pos)
{
  int32_t pre_index, index;
  uint8_t first_switch, second_switch;
  uint16_t addr;
  int32_t val_x, val_y;
  osStatus_t status;
  uint8_t pre_pos;

  int32_t old_x, old_y;
  int32_t tmp_x, tmp_y;
  int32_t safe_dist = 1000;

  if (   (switch_channel == TX_SWITCH_CHANNEL && switch_pos >= 66) \
      || (switch_channel == RX_SWITCH_CHANNEL && switch_pos >= 33) \
      || (switch_channel > 1)) {
    return -1;
  }

  index = Get_Index_Of_Channel_Map(switch_channel, switch_pos);
  if (index < 0) {
    EPT("Invalid switch channel number : %u %u\n", switch_channel, switch_pos);
    return -1;
  }

  first_switch = channel_map[index].first_switch;
  second_switch = channel_map[index].second_switch;

  // TODO: Disable previous switch channel
  if (switch_channel == TX_SWITCH_CHANNEL) {
    pre_pos = run_status.tx_switch_channel;
  } else {
    pre_pos = run_status.rx_switch_channel;
  }
#if 0
  if (pre_pos != 0xFF) {
    pre_index = Get_Index_Of_Channel_Map(switch_channel, pre_pos);

    if (pre_index != index) {
      Clear_Switch_Dac(channel_map[pre_index].second_switch);
    } else {
      Clear_Switch_Dac(first_switch);
    }
  }
#else
  if (pre_pos != 0xFF) {
    pre_index = Get_Index_Of_Channel_Map(switch_channel, pre_pos);
    Clear_Switch_Dac(channel_map[pre_index].first_switch);
    Clear_Switch_Dac(channel_map[pre_index].second_switch);
  }
  old_x = 0;
  old_y = 0;
#endif

  // Configure new switch channel

  // second siwtch
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (switch_pos < 64) {
      addr = channel_map[index].second_eeprom_addr + ((switch_pos % 32 + 1) - channel_map[index].min_channel_num) * 8;
    } else {
      addr = channel_map[index].second_eeprom_addr + 10 * 8;
    }
  } else {
    if (switch_pos < 32) {
      addr = channel_map[index].second_eeprom_addr + ((switch_pos % 32 + 1) - channel_map[index].min_channel_num) * 8;
    } else {
      addr = channel_map[index].second_eeprom_addr + 10 * 8;
    }
  }

  status = get_32_from_eeprom(EEPROM_ADDR, addr, (uint32_t*)&val_x);
  if (status != osOK) {
    EPT("Get val_x failed 1. status = %d\n", status);
    return -2;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr + 4, (uint32_t*)&val_y);
  if (status != osOK) {
    EPT("Get val_y failed 1. status = %d\n", status);
    return -3;
  }

  if (set_sw_dac(second_switch, val_x, val_y)) {
    EPT("Write DAC faliled\n");
    return -4;
  }
  osDelay(pdMS_TO_TICKS(2));

  // first switch
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (switch_pos < 32 || switch_pos == 64) {
      addr = channel_map[index].first_eeprom_addr + index * 8;
    } else {
      addr = channel_map[index].first_eeprom_addr + 3 * 8 + index * 8;
    }
  } else {
    addr = channel_map[index].first_eeprom_addr + (index - 3) * 8;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr, (uint32_t*)&val_x);
  if (status != osOK) {
    EPT("Get val_x failed 2. status = %d\n", status);
    return -5;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr + 4, (uint32_t*)&val_y);
  if (status != osOK) {
    EPT("Get val_y failed 2. status = %d\n", status);
    return -6;
  }

  if (val_x - old_x >= 0 && val_y - old_y >= 0) {
    // third quadrant
    tmp_x = (val_x - old_x >= safe_dist) ? (val_x - safe_dist) : (old_x);
    tmp_y = (val_y - old_y >= safe_dist) ? (val_y - safe_dist) : (old_y);
  } else if (val_x - old_x <= 0 && val_y - old_y >= 0) {
    // fourth quadrant
    tmp_x = (old_x - val_x >= safe_dist) ? (val_x + safe_dist) : (old_x);
    tmp_y = (val_y - old_y >= safe_dist) ? (val_y - safe_dist) : (old_y);
  } else if (val_x - old_x <= 0 && val_y - old_y <= 0) {
    // first quadrant
    tmp_x = (old_x - val_x >= safe_dist) ? (val_x + safe_dist) : (old_x);
    tmp_y = (old_y - val_y >= safe_dist) ? (val_y + safe_dist) : (old_y);
  } else if (val_x - old_x >= 0 && val_y - old_y <= 0) {
    // second quadrant
    tmp_x = (val_x - old_x >= safe_dist) ? (val_x - safe_dist) : (old_x);
    tmp_y = (old_y - val_y >= safe_dist) ? (val_y + safe_dist) : (old_y);
  }


  if (set_sw_dac(first_switch, tmp_x, tmp_y)) {
    EPT("Write DAC faliled\n");
    return -7;
  }
    
  sw_tim_control.time = 3; //300us
  sw_tim_control.step = 20;
  sw_tim_control.counter = 0;
  sw_tim_control.sw_num = first_switch;
  sw_tim_control.cur_x = tmp_x;
  sw_tim_control.cur_y = tmp_y;
  sw_tim_control.dst_x = val_x;
  sw_tim_control.dst_y = val_y;

  HAL_IWDG_Refresh(&hiwdg);
  HAL_TIM_Base_Start_IT(&htim6);

  status = osSemaphoreAcquire(switchSemaphore, 30);
  if (status != osOK) {
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Set Switch Timeout!\n");
  }
  osDelay(1);


  return 0;
}

int8_t Get_Switch_Adc(uint32_t switch_id, uint16_t *px, uint16_t *nx, uint16_t *py, uint16_t *ny)
{
  uint32_t i;
  osStatus_t status;

  if (switch_id >= SWITCH_NUM_TOTAL || switch_id == 0) {
    return -1;
  }

  for (i = 0; i < sizeof(switch_map)/sizeof(switch_map[0]); ++i) {
    if (switch_map[i].sw_num == switch_id)
      break;
  }
  status = RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_px_chan, px);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_nx_chan, nx);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_py_chan, py);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_ny_chan, ny);
  if (status != osOK) {
    EPT("Read adc failed\n");
    return -2;
  }

  return 0;
}

int8_t Get_Current_Switch_Channel(uint8_t switch_channel)
{
  int32_t index, std_x, std_y;
  uint8_t first_switch, second_switch;
  uint16_t addr;
  osStatus_t status;
  uint32_t val_x, val_y;
  uint16_t px, nx, py, ny;
  uint8_t pos;
  
  if (switch_channel == TX_SWITCH_CHANNEL) {
    pos = run_status.tx_switch_channel;
  } else if (switch_channel == RX_SWITCH_CHANNEL) {
    pos = run_status.rx_switch_channel;
  } else {
    return -1;
  }

  if (pos == 0xFF) {
    EPT("No switch channel is valid\n");
    //THROW_LOG("No switch channel is valid\n");
    return -2;
  }

  index = Get_Index_Of_Channel_Map(switch_channel, pos);
  if (index < 0) {
    EPT("Invalid switch channel number : %u %u\n", switch_channel, pos);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Invalid switch channel number : %u %u\n", switch_channel, pos);
    return -2;
  }

  first_switch = channel_map[index].first_switch;
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (pos < 32 || pos == 64) {
      addr = channel_map[index].first_eeprom_addr + index * 8;
    } else {
      addr = channel_map[index].first_eeprom_addr + 3 * 8 + index * 8;
    }
  } else {
    addr = channel_map[index].first_eeprom_addr + (index - 3) * 8;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr, (uint32_t*)&std_x);
  if (status != osOK) {
    EPT("Get val_x failed 1. status = %d\n", status);
    return -2;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr + 4, (uint32_t*)&std_y);
  if (status != osOK) {
    EPT("Get val_y failed 1. status = %d\n", status);
    return -3;
  }
  if (Get_Switch_Adc(first_switch, &px, &nx, &py, &ny)) {
    EPT("Get_Switch_Adc failed 1\n");
    return -4;
  }
  if (std_x >= 0)
    val_x = (uint32_t)px;
  else
    val_x = (uint32_t)nx;
  if (std_y >= 0)
    val_y = (uint32_t)py;
  else
    val_y = (uint32_t)ny;
  EPT("First switch DA/AD value: val_x = %d, std_x = %d, val_y = %d, std_y = %d\n", val_x, std_x, val_y, std_y);
  if (!Is_Value_Approximate(val_x, my_abs(std_x)) || !Is_Value_Approximate(val_y, my_abs(std_y))) {
    EPT("First level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "First level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    Set_Switch_Alarm(first_switch);
    return -5;
  }
  Clear_Switch_Alarm(first_switch);

  second_switch = channel_map[index].second_switch;
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (pos < 64) {
      addr = channel_map[index].second_eeprom_addr + ((pos % 32 + 1) - channel_map[index].min_channel_num) * 8;
    } else {
      addr = channel_map[index].second_eeprom_addr + 10 * 8;
    }
  } else {
    if (pos < 32) {
      addr = channel_map[index].second_eeprom_addr + ((pos % 32 + 1) - channel_map[index].min_channel_num) * 8;
    } else {
      addr = channel_map[index].second_eeprom_addr + 10 * 8;
    }
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr, (uint32_t*)&std_x);
  if (status != osOK) {
    EPT("Get val_x failed 2. status = %d\n", status);
    return -6;
  }
  status = get_32_from_eeprom(EEPROM_ADDR, addr + 4, (uint32_t*)&std_y);
  if (status != osOK) {
    EPT("Get val_y failed 2. status = %d\n", status);
    return -7;
  }
  if (Get_Switch_Adc(second_switch, &px, &nx, &py, &ny)) {
    EPT("Get_Switch_Adc failed 1\n");
    return -8;
  }
  if (std_x >= 0)
    val_x = (uint32_t)px;
  else
    val_x = (uint32_t)nx;
  if (std_y >= 0)
    val_y = (uint32_t)py;
  else
    val_y = (uint32_t)ny;
  EPT("Second switch DA/AD value: val_x = %d, std_x = %d, val_y = %d, std_y = %d\n", val_x, std_x, val_y, std_y);
  if (!Is_Value_Approximate(val_x, my_abs(std_x)) || !Is_Value_Approximate(val_y, my_abs(std_y))) {
    EPT("Second level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Second level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    Set_Switch_Alarm(second_switch);
    return -9;
  }
  Clear_Switch_Alarm(second_switch);

  if (switch_channel == TX_SWITCH_CHANNEL) {
    return run_status.tx_switch_channel;
  } else {
    return run_status.rx_switch_channel;
  }
}

void Set_Switch_Alarm(uint8_t sw_num)
{
  uint32_t alarm_bit;
  
  switch (sw_num) {
    case SWITCH_NUM_1:
    case SWITCH_NUM_2:
      alarm_bit = EXP_SWICH_VOLTAGE1;
      break;
    case SWITCH_NUM_3:
    case SWITCH_NUM_4:
      alarm_bit = EXP_SWICH_VOLTAGE2;
      break;
    case SWITCH_NUM_5:
    case SWITCH_NUM_6:
      alarm_bit = EXP_SWICH_VOLTAGE3;
      break;
    case SWITCH_NUM_7:
    case SWITCH_NUM_8:
      alarm_bit = EXP_SWICH_VOLTAGE4;
      break;
  }
  
  Set_Flag(&run_status.exp, alarm_bit);
}

void Clear_Switch_Alarm(uint8_t sw_num)
{
  uint32_t alarm_bit;
  
  switch (sw_num) {
    case SWITCH_NUM_1:
    case SWITCH_NUM_2:
      alarm_bit = EXP_SWICH_VOLTAGE1;
      break;
    case SWITCH_NUM_3:
    case SWITCH_NUM_4:
      alarm_bit = EXP_SWICH_VOLTAGE2;
      break;
    case SWITCH_NUM_5:
    case SWITCH_NUM_6:
      alarm_bit = EXP_SWICH_VOLTAGE3;
      break;
    case SWITCH_NUM_7:
    case SWITCH_NUM_8:
      alarm_bit = EXP_SWICH_VOLTAGE4;
      break;
  }
  
  Clear_Flag(&run_status.exp, alarm_bit);
}

int8_t Get_Current_Switch_ADC(uint8_t switch_channel, int16_t *x1, int16_t *y1, int16_t *x2, int16_t *y2)
{
  int32_t index;
  uint8_t first_switch, second_switch;
  uint16_t px, nx, py, ny;
  uint8_t pos;
  
  if (switch_channel == TX_SWITCH_CHANNEL) {
    pos = run_status.tx_switch_channel;
  } else if (switch_channel == RX_SWITCH_CHANNEL) {
    pos = run_status.rx_switch_channel;
  } else {
    return -1;
  }

  first_switch = switch_channel == TX_SWITCH_CHANNEL ? SWITCH_NUM_1 : SWITCH_NUM_5;
  if (Get_Switch_Adc(first_switch, &px, &nx, &py, &ny)) {
    return -2;
  }
  *x1 = (int16_t)px + (int16_t)nx;
  *y1 = (int16_t)py + (int16_t)ny;

  if (pos == 0xFF) {
    *x2 = 0;
    *y2 = 0;
    return 0;
  }

  index = Get_Index_Of_Channel_Map(switch_channel, pos);
  if (index < 0) {
    return -3;
  }

  second_switch = channel_map[index].second_switch;
  if (Get_Switch_Adc(second_switch, &px, &nx, &py, &ny)) {
    return -4;
  }
  *x2 = (int16_t)px + (int16_t)nx;
  *y2 = (int16_t)py + (int16_t)ny;

  return 0;
}

int8_t Clear_Switch_Dac(uint32_t switch_id)
{
  if (set_sw_dac(switch_id, 0, 0)) {
    EPT("Write DAC faliled\n");
    return -1;
  }

  return 0;
}

/* 
 *  switch_pos
 *      channel = 0
 *        0-31: 2*32Switch 1 to 32
 *        32-63: 2*32Switch 2 to 32
 *        64: 2*32Switch 1 to 33
 *        65: 2*32Switch 2 to 33
 *      channel = 1
 *        0-31: 1*32Switch 1 to 32
 *        32: 1*32Switch 1 to 33
 */
int32_t Get_Index_Of_Channel_Map(uint8_t switch_channel, uint8_t switch_pos)
{
  int i;
  uint8_t act_pos;
  
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (switch_pos < 64) {
      act_pos = (switch_pos % 32) + 1;
    } else if (switch_pos < 66) {
      act_pos = 33;
    } else {
      act_pos = 0xFF;
    }
  } else {
    if (switch_pos < 32) {
      act_pos = (switch_pos % 32) + 1;
    } else if (switch_pos < 33) {
      act_pos = 33;
    } else {
      act_pos = 0xFF;
    }
  }

  for (i = 0; i < channel_map_count; ++i) {
    if (channel_map[i].switch_number != switch_channel) {
      continue;
    }
    if (act_pos >= channel_map[i].min_channel_num && act_pos <= channel_map[i].max_channel_num) {
      break;
    }
  }
  
  if (i >= channel_map_count) {
    return -1;
  }
  
  return i;
}

void Reset_Switch(uint8_t switch_channel)
{
  uint32_t index;
  uint8_t pos;
  
  if (run_status.maigc == RUN_MAGIC) {
    if (switch_channel == TX_SWITCH_CHANNEL) {
      pos = run_status.tx_switch_channel;
    } else if (switch_channel == RX_SWITCH_CHANNEL) {
      pos = run_status.rx_switch_channel;
    } else {
      return;
    }

    if (pos != 0xFF) {
      if (switch_channel == TX_SWITCH_CHANNEL) {
        Clear_Switch_Ready(TX_SWITCH_CHANNEL);
      } else {
        Clear_Switch_Ready(RX_SWITCH_CHANNEL);
      }
      index = Get_Index_Of_Channel_Map(switch_channel, pos);
      Clear_Switch_Dac(channel_map[index].first_switch);
      Clear_Switch_Dac(channel_map[index].second_switch);
      if (switch_channel == TX_SWITCH_CHANNEL) {
        Set_Switch_Ready(TX_SWITCH_CHANNEL);
      } else {
        Set_Switch_Ready(RX_SWITCH_CHANNEL);
      }
    }

    if (switch_channel == TX_SWITCH_CHANNEL) {
      run_status.tx_switch_channel = 0xFF;
    } else {
      run_status.rx_switch_channel = 0xFF;
    }
  } else {
    Clear_Switch_Ready(TX_SWITCH_CHANNEL);
    Clear_Switch_Ready(RX_SWITCH_CHANNEL);
    Clear_Switch_Dac(SWITCH_NUM_1);
    Clear_Switch_Dac(SWITCH_NUM_2);
    Clear_Switch_Dac(SWITCH_NUM_3);
    Clear_Switch_Dac(SWITCH_NUM_4);
    Clear_Switch_Dac(SWITCH_NUM_5);
    Clear_Switch_Dac(SWITCH_NUM_6);
    Clear_Switch_Dac(SWITCH_NUM_7);
    Clear_Switch_Dac(SWITCH_NUM_8);
    run_status.tx_switch_channel = 0xFF;
    run_status.rx_switch_channel = 0xFF;
    Set_Switch_Ready(TX_SWITCH_CHANNEL);
    Set_Switch_Ready(RX_SWITCH_CHANNEL);
  }
}

void Reset_Switch_Only(uint8_t switch_channel)
{
  uint32_t index;
  uint8_t pos;
  
  if (run_status.maigc == RUN_MAGIC) {
    if (switch_channel == TX_SWITCH_CHANNEL) {
      pos = run_status.tx_switch_channel;
    } else if (switch_channel == RX_SWITCH_CHANNEL) {
      pos = run_status.rx_switch_channel;
    } else {
      return;
    }

    if (pos != 0xFF) {
      index = Get_Index_Of_Channel_Map(switch_channel, pos);
      Clear_Switch_Dac(channel_map[index].first_switch);
      Clear_Switch_Dac(channel_map[index].second_switch);
    }
  } else {
    Clear_Switch_Dac(SWITCH_NUM_1);
    Clear_Switch_Dac(SWITCH_NUM_2);
    Clear_Switch_Dac(SWITCH_NUM_3);
    Clear_Switch_Dac(SWITCH_NUM_4);
    Clear_Switch_Dac(SWITCH_NUM_5);
    Clear_Switch_Dac(SWITCH_NUM_6);
    Clear_Switch_Dac(SWITCH_NUM_7);
    Clear_Switch_Dac(SWITCH_NUM_8);
  }
}

inline void Set_Alarm(void)
{
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}

inline void Clear_Alarm(void)
{
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}

void Set_Lazer_Ready(void)
{
  HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
  run_status.lazer_ready = 1;
}

void Clear_Lazer_Ready(void)
{
  HAL_GPIO_WritePin(L_READY_N_GPIO_Port, L_READY_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
  run_status.lazer_ready = 0;
}

void Init_Run_Status(void)
{
  run_status.maigc = RUN_MAGIC;
  run_status.uart_reset = 0;
  run_status.tx_switch_channel = 0xFF;
  run_status.rx_switch_channel = 0xFF;
  run_status.pre_alarm = 0;
  run_status.pre_exp_value = 0;
  run_status.exp = 0;
  if (HAL_GPIO_ReadPin(SW1_BLOCK_GPIO_Port, SW1_BLOCK_Pin) == GPIO_PIN_RESET) {
    run_status.tx_block = 1;
  } else {
    run_status.tx_block = 0;
  }

  run_status.modulation = 0;
  run_status.tosa_enable = 0;
  run_status.lazer_ready = 0;
  run_status.tosa_dst_power_high = 2;
  run_status.tosa_dst_power_low = -2;
  run_status.allow_monitor = 0;
  run_status.power_mode = 1;

  run_status.osc_status = 0xFF;
  run_status.sw_adc_int = 200;
  run_status.sw_adc_double = 0.05;
}

void Set_Switch_Ready(uint8_t switch_channel)
{
  GPIO_TypeDef *port;
  uint16_t pin;
  
  if (switch_channel == TX_SWITCH_CHANNEL) {
    port = SW1_READY_GPIO_Port;
    pin = SW1_READY_Pin;
  } else {
    port = SW2_READY_GPIO_Port;
    pin = SW2_READY_Pin;
  }
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}

void Clear_Switch_Ready(uint8_t switch_channel)
{
  GPIO_TypeDef *port;
  uint16_t pin;
  
  if (switch_channel == TX_SWITCH_CHANNEL) {
    port = SW1_READY_GPIO_Port;
    pin = SW1_READY_Pin;
  } else {
    port = SW2_READY_GPIO_Port;
    pin = SW2_READY_Pin;
  }
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
  osDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}

void Set_Flag(uint32_t *status, uint32_t bit_addr)
{
  *status |= 1 << bit_addr;
}

void Clear_Flag(uint32_t *status, uint32_t bit_addr)
{
  *status &= ~(1 << bit_addr);
}  

uint8_t Is_Flag_Set(uint32_t *status, uint32_t bit_addr)
{
  if (*status & (1 << bit_addr)) return 1;
  else return 0;
}

double Cal_Temp(uint16_t adc)
{
  double d_value, temp;
  d_value = (double)adc / 4096 * 2.5;
  d_value = d_value / (2.5 - d_value);
  d_value = log(d_value);
  temp = 503620 / (149 * d_value + 1690) - 273;
  return temp;
}

double Cal_Tosa_Temp(uint16_t adc)
{
  double d_value, temp;
  // voltage
  d_value = (double)adc / 4096 * 2.5;
  
  // R
  d_value = 17.8 / (d_value / 1.25 - 1 + 17.8 / 17.68) - 7.68;
  
  // Temperature
  d_value = log(d_value / 10);
  temp = (49125 - 40667 * d_value) / (149 * d_value + 1965);
  return temp;
}

uint8_t Enable_Tosa()
{
  osStatus_t status;

  if (tosa_table_count == 0) {
    Set_Flag(&run_status.internal_exp, INT_EXP_TOSA_DATA);
    return 4;
  }

  status = RTOS_DAC128S085_Write(DAC128S085_TEC_SWITCH_CHANNEL, 4095, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    return 1;
  }
  status = RTOS_DAC128S085_Write(DAC128S085_TOSA_SWITCH_CHANNEL, 4095, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    return 3;
  }

  return 0;
}

uint8_t Disable_Tosa()
{
  osStatus_t status;

  status = RTOS_DAC128S085_Write(DAC128S085_TOSA_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_Write(DAC128S085_TEC_SWITCH_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, 0, DAC128S085_MODE_NORMAL);
  DAC5541_Write(0);

  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    return 1;
  }
  return 0;
}

void update_tosa_table(void)
{
  uint8_t i, buf[16];
  int32_t val;
  osStatus_t status;
  
  tosa_table_count = 0;
  for (i = 0; i < TOSA_TABLE_COUNT; ++i) {
    status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_TOSA + 16 * i, buf, 16);
    if (status != osOK) {
      tosa_table_count = 0;
      return ;
    }

    val = Buffer_To_BE32(buf);
    if (val == 0 || val >> 16) {
      break;
    }
    val = Buffer_To_BE32(buf + 4);
    if (val == 0 || val >> 12) {
      break;
    }
    val = Buffer_To_BE32(buf + 8);
    if (val == 0 || val >> 12) {
      break;
    }
    tosa_table[i].tosa_dac = (uint16_t)Buffer_To_BE32(buf);
    tosa_table[i].tec_dac = (uint16_t)Buffer_To_BE32(buf + 4);
    tosa_table[i].tap_adc = (uint16_t)Buffer_To_BE32(buf + 8);
    val = (int32_t)Buffer_To_BE32(buf + 12);
    tosa_table[i].tap_power = (double)val / 100;
    if (tosa_table[i].tap_power > 10 || tosa_table[i].tap_power < -20) {
      break;
    }
  }
  tosa_table_count = i;
  if (tosa_table_count > 1) {
    if (tosa_table[tosa_table_count - 1].tap_power < 4) {
      tosa_power_high_max_thr = tosa_table[tosa_table_count - 1].tap_power;
    } else {
      tosa_power_high_max_thr = 4;
    }
    if (tosa_table[0].tap_power > -8) {
      tosa_power_low_min_thr = tosa_table[0].tap_power;
    } else {
      tosa_power_low_min_thr = -8;
    }
  }
}

TosaCalData Get_Tosa_Data(double power)
{
  TosaCalData tosa_data, tosa_x1, tosa_x2;
  uint8_t i;
  
  if (!tosa_table_count) {
    tosa_data.tosa_dac = 0;
  } else if (tosa_table_count == 1) {
    tosa_data.tosa_dac = 0;
  } else {
    if (power < tosa_table[0].tap_power || power > tosa_table[tosa_table_count - 1].tap_power) {
      tosa_x1 = tosa_table[0];
      tosa_x2 = tosa_table[tosa_table_count - 1];
    } else {
      for (i = 1; i < tosa_table_count; ++i) {
        if (power <= tosa_table[i].tap_power) 
          break;
      }
      tosa_x1 = tosa_table[i - 1];
      tosa_x2 = tosa_table[i];
    }
    tosa_data = Cal_Tosa_Data(tosa_x1, tosa_x2, power);
  }
  
  return tosa_data;
}

TosaCalData Cal_Tosa_Data(TosaCalData tosa_node_1, TosaCalData tosa_node_2, double power)
{
  TosaCalData tosa_data;
  double x1, x2, y1, y2, x, y;
  
  tosa_data.tap_power = power;
  
  x1 = tosa_node_1.tap_power; x2 = tosa_node_2.tap_power;
  y1 = tosa_node_1.tap_adc; y2 = tosa_node_2.tap_adc;
  x = tosa_data.tap_power;
  y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
  tosa_data.tap_adc = (uint32_t)(y + 0.5);

  x1 = (double)tosa_node_1.tap_adc; x2 = (double)tosa_node_2.tap_adc;
  y1 = (double)tosa_node_1.tosa_dac; y2 = (double)tosa_node_2.tosa_dac;
  x = tosa_data.tap_adc;
  y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
  tosa_data.tosa_dac = (uint32_t)(y + 0.5);

  x1 = (double)tosa_node_1.tosa_dac; x2 = (double)tosa_node_2.tosa_dac;
  y1 = (double)tosa_node_1.tec_dac; y2 = (double)tosa_node_2.tec_dac;
  x = tosa_data.tosa_dac;
  y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
  tosa_data.tec_dac = (uint32_t)(y + 0.5);
  
  return tosa_data;
}

uint8_t Get_Tap_Power(double *cur_power)
{
  uint8_t i = 0, times = 0;
  uint16_t tap_adc;
  double tap_power, pre_power, first_power;

#if 0
  while (i < 10) {
    if (get_tap_pd_power(&tap_adc, &tap_power)) {
      return 1;
    }
    power_arr_for_test[i] = tap_power;
    ++i;
    osDelay(pdMS_TO_TICKS(15));
  }

  i = 0;
#endif
  // osDelay(100);
  while (1) {
    if (get_tap_pd_power(&tap_adc, &tap_power)) {
      return 1;
    }
    
    if (i == 0) {
      pre_power = tap_power;
      first_power = pre_power;
      i++;
    } else if (i < 5) {
      if ((tap_power <= pre_power + 0.3 && tap_power >= pre_power - 0.3) && \
        (tap_power <= first_power + 0.5 && tap_power >= first_power - 0.5)) {
        i++;
        pre_power = (pre_power + tap_power) / 2;
      } else {
        i = 0;
        times++;
      }
    } else {
      break;
    }
    
    if (times > 5) {
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Get Tap PD Power exception\n");
      break;
    }
    osDelay(pdMS_TO_TICKS(3));
  }
  
  *cur_power = pre_power;
#if 0
  THROW_LOG(MSG_TYPE_NORMAL_LOG, "Power Data: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", \
            power_arr_for_test[0], power_arr_for_test[1], power_arr_for_test[2], power_arr_for_test[3], power_arr_for_test[4], \
            power_arr_for_test[5], power_arr_for_test[6], power_arr_for_test[7], power_arr_for_test[8], power_arr_for_test[9]);
#endif
  return 0;
}

uint8_t Get_Rx_Power(double *cur_power)
{
  uint8_t i = 0;
  uint16_t rx_adc;
  double rx_power, sum;

  for (i = 0, sum = 0; i < 10; ++i) {
    if (get_rx_pd_power(&rx_adc, &rx_power)) {
      return 1;
    }
    sum += rx_power;
    osDelay(pdMS_TO_TICKS(15));
  }
  *cur_power = sum / i;
  return 0;
}

uint8_t Is_Tec_Lock(void)
{
  if (HAL_GPIO_ReadPin(TMPGD_GPIO_Port, TMPGD_Pin) == GPIO_PIN_RESET)
    return 0;
  
  return 1;
}

uint8_t Cali_Power(double power_dst)
{
  uint8_t ret;
  uint32_t times = 0;
  double power_tap, power_tmp;

  do {
    if ((ret = Get_Tap_Power(&power_tap)) != 0) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "Get_Tap_Power error code = %u\n", ret);
      Set_Flag(&run_status.internal_exp, INT_EXP_TAP_PD);
      return 1;
    }
    if (power_tap < -30) {
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Tap Power = %.2lf is not normal!\n", power_tap);
      return 2;
    }
    if (power_tap > power_dst + 0.5 || power_tap < power_dst - 0.5) {
      ++times;
      if (times > 10) {
        if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
          Set_Flag(&run_status.exp, EXP_LAZER_POWER);
        }
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "Module cannot reach destination power after self-control process\n");
        return 0;
      }


      power_tmp = power_dst - (power_tap - power_dst);
      run_status.tosa_high = Get_Tosa_Data(power_tmp);
      THROW_LOG(MSG_TYPE_ERROR_LOG, "Tap power = %.2lf, destination is %.2lf, times = %u, research power is %.2lf(dac:%u)\n",\
                    power_tap, power_dst, times, power_tmp, run_status.tosa_high.tosa_dac);

      if (run_status.tosa_high.tosa_dac > 0x10000 * 0.95) {
        THROW_LOG(MSG_TYPE_ERROR_LOG, "Dac value exceeds limitation %u\n", run_status.tosa_high.tosa_dac);
        if (power_tap >= 0) {
          if (power_tap > power_dst + 2 || power_tap < power_dst - 2) {
            if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
              Set_Flag(&run_status.exp, EXP_LAZER_POWER);
            }
          } else {
            if (Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
              Clear_Flag(&run_status.exp, EXP_LAZER_POWER);
            }
            if (Is_Flag_Set(&run_status.exp, EXP_LAZER_AGING)) {
              Clear_Flag(&run_status.exp, EXP_LAZER_AGING);
            }
          }
          return 0;
        } else {
          // Recover power actual value
          run_status.tosa_high = Get_Tosa_Data(run_status.tosa_dst_power_high);
          run_status.tosa_low = Get_Tosa_Data(run_status.tosa_dst_power_low);
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Lazer is aging\n");

          if (!Is_Flag_Set(&run_status.exp, EXP_LAZER_AGING)) {
            Set_Flag(&run_status.exp, EXP_LAZER_AGING);
          }
          return 3;
        }
      } else {
        RTOS_DAC128S085_Write(DAC128S085_TEC_VALUE_CHANNEL, run_status.tosa_high.tec_dac, DAC128S085_MODE_NORMAL);
        DAC5541_Write(run_status.tosa_high.tosa_dac);
        osDelay(pdMS_TO_TICKS(1));
      }
    } else {
      if (Is_Flag_Set(&run_status.exp, EXP_LAZER_POWER)) {
        Clear_Flag(&run_status.exp, EXP_LAZER_POWER);
      }
      if (Is_Flag_Set(&run_status.exp, EXP_LAZER_AGING)) {
        Clear_Flag(&run_status.exp, EXP_LAZER_AGING);
      }
      return 0;
    }
  } while (1);
}


uint8_t cal_tap_pd_by_power(uint16_t *adc, double power)
{
  double d_val;
  uint16_t adc1, adc2;
  double power1, power2;
  uint8_t dst, offset;


  d_val = power;

  if (tosa_table_count < 2) {
    return 2;
  }

  power1 = tosa_table[0].tap_power;
  power2 = tosa_table[1].tap_power;
  if (power1 > power2) {
    dst = 1;
  } else if (power1 < power2) {
    dst = 0;
  } else {
    return 3;
  }

  offset = 1;
  if (dst) {
    while ((power2 = tosa_table[offset].tap_power) > d_val) {
      if (offset > tosa_table_count - 2)
        break;
      ++offset;
      power1 = power2;
    }
  } else {
    while ((power2 = tosa_table[offset].tap_power) < d_val) {
      if (offset > tosa_table_count - 2)
        break;
      ++offset;
      power1 = power2;
    }
  }
  adc1 = tosa_table[offset - 1].tap_adc;
  adc2 = tosa_table[offset].tap_adc;

  if (adc1 == adc2 || power1 == power2) {
    return 4;
  }

  *adc = (uint16_t)(int16_t)((d_val - power1) * ((double)(int16_t)adc2 - (double)(int16_t)adc1) / (power2 - power1) + (double)(int16_t)adc1);

  return 0;
}

uint8_t get_tap_pd_power(uint16_t *adc, double *power)
{
  osStatus_t status;
  uint16_t u_val;
  uint16_t adc1, adc2;
  double power1, power2;
  uint8_t dst, offset;

  status = RTOS_ADC7828_Read(TAP_PD_CHANNEL, &u_val);
  if (status != osOK) {
    EPT("Read adc failed\n");
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    return 1;
  }

  if (tosa_table_count < 2) {
    return 2;
  }

  adc1 = tosa_table[0].tap_adc;
  adc2 = tosa_table[1].tap_adc;
  if (adc1 > adc2) {
    dst = 1;
  } else if (adc1 < adc2) {
    dst = 0;
  } else {
    return 3;
  }

  offset = 1;
  if (dst) {
    while ((adc2 = tosa_table[offset].tap_adc) > u_val) {
      if (offset > tosa_table_count - 2)
        break;
      ++offset;
      adc1 = adc2;
    }
  } else {
    while ((adc2 = tosa_table[offset].tap_adc) < u_val) {
      if (offset > tosa_table_count - 2)
        break;
      ++offset;
      adc1 = adc2;
    }
  }
  power1 = tosa_table[offset - 1].tap_power;
  power2 = tosa_table[offset].tap_power;

  if (adc1 == adc2 || power1 == power2) {
    return 4;
  }

  *adc = u_val;
  *power = (u_val - adc1) * (power2 - power1) / (adc2 - adc1) + power1;

  return 0;
}

uint8_t get_rx_pd_power(uint16_t *adc, double *power)
{
  osStatus_t status;
  uint16_t u_val;
  uint16_t addr = EE_CAL_RX_PD;
  uint32_t adc1, adc2, u32_val;
  double power1, power2;
  uint8_t dst, offset;

  status = RTOS_ADC7828_Read(RX_PD_CHANNEL, &u_val);
  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    return 1;
  }

  status = get_32_from_eeprom(EEPROM_ADDR, addr, &adc1);
  status |= get_32_from_eeprom(EEPROM_ADDR, addr + 8, &adc2);
  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    return 1;
  }

  if (adc1 > adc2) {
    dst = 1;
  } else if (adc1 < adc2) {
    dst = 0;
  } else {
    return 3;
  }

  offset = 1;
  while (1) {
    status = get_32_from_eeprom(EEPROM_ADDR, addr + offset * 8, &adc2);
    if (status != osOK) {
      Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
      return 1;
    }
    if (dst) {
      if ((uint16_t)adc2 > u_val) {
        if (offset > 10 - 2)
          break;
        ++offset;
        adc1 = adc2;
      } else {
        break;
      }
    } else {
      if ((uint16_t)adc2 < u_val) {
        if (offset > 10 - 2)
          break;
        ++offset;
        adc1 = adc2;
      } else {
        break;
      }
    }
  }

  status = get_32_from_eeprom(EEPROM_ADDR, addr + 4 + (offset - 1) * 8, &u32_val);
  power1 = (double)((int32_t)u32_val) / 100;
  status |= get_32_from_eeprom(EEPROM_ADDR, addr + 4 + offset * 8, &u32_val);
  power2 = (double)((int32_t)u32_val) / 100;
  if (status != osOK) {
    Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
    return 1;
  }

  if (adc1 == adc2 || power1 == power2) {
    return 4;
  }

  *adc = u_val;
  *power = (u_val - (uint16_t)adc1) * (power2 - power1) / ((uint16_t)adc2 - (uint16_t)adc1) + power1;

  return 0;
}

uint8_t Get_Performance(uint8_t per_id, uint8_t *pBuf)
{
  osStatus_t status;
  double d_val, d_val_2;
  uint8_t buf[4];
  int32_t val32;
  int16_t val16_1, val16_2, val16_3, val16_4;
  uint8_t u_val8;
  uint16_t u_val16;

  switch (per_id) {
    case 0:
      status = RTOS_ADC7953_SPI5_Read(TEC_ADC_LD_CURRENT_CHANNEL, &u_val16);
      if (status != osOK) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        d_val = (double)u_val16 / 4096 * 2.5;
        d_val = d_val / 22 * 1000;
        val32 = (uint32_t)(int32_t)d_val;
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 1:
      status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_TEMP_CHANNEL, &u_val16);
      if (status != osOK) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        d_val = Cal_Tosa_Temp(u_val16);
        if (d_val >= 0) {
          val32 = (int32_t)(d_val * 10 + 0.5);
        } else {
          val32 = (int32_t)(d_val * 10 - 0.5);
        }
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 2:
      status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_CURRENT_CHANNEL, &u_val16);
      if (status != osOK) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        d_val = (double)u_val16 / 4096 * 2.5;
        d_val = (d_val - 1.25) / 0.285 * 1000;
        if (d_val >= 0) {
          val32 = (int32_t)(d_val + 0.5);
        } else {
          val32 = (int32_t)(d_val - 0.5);
        }
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 3:
      status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_VOLTAGE_CHANNEL, &u_val16);
      if (status != osOK) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        d_val = (double)u_val16 / 4096 * 2.5;
        d_val = (d_val - 1.25) / 0.25 * 1000;
        if (d_val >= 0) {
          val32 = (int32_t)(d_val + 0.5);
        } else {
          val32 = (int32_t)(d_val - 0.5);
        }
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 4:
      BE32_To_Buffer(0x80000000, pBuf);
      break;
    case 5:
      if (run_status.lazer_ready) {
        BE32_To_Buffer(1, pBuf);
      } else {
        BE32_To_Buffer(0, pBuf);
      }
      break;
    case 6:
      if (run_status.tosa_enable && HAL_GPIO_ReadPin(TMPGD_GPIO_Port,TMPGD_Pin) == GPIO_PIN_SET) {
        BE32_To_Buffer(1, pBuf);
      } else {
        BE32_To_Buffer(0, pBuf);
      }
      break;
    case 7:
      if (run_status.tosa_enable)
        BE32_To_Buffer(1, pBuf);
      else
        BE32_To_Buffer(0, pBuf);
      break;
    case 8:
      if (run_status.modulation)
        BE32_To_Buffer(1, pBuf);
      else
        BE32_To_Buffer(0, pBuf);
      break;
    case 9:
      u_val8 = get_tap_pd_power(&u_val16, &d_val);
      if (u_val8 == 1) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else if (u_val8 > 1) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        if (d_val >= 0) {
          val32 = (int32_t)(d_val * 100 + 0.5);
        } else {
          val32 = (int32_t)(d_val * 100 - 0.5);
        }
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 0xA:
      u_val8 = get_rx_pd_power(&u_val16, &d_val);
      if (u_val8 == 1) {
        Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
        BE32_To_Buffer(0x80000000, pBuf);
      } else if (u_val8 > 1) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        if (d_val >= 0) {
          val32 = (int32_t)(d_val * 100 + 0.5);
        } else {
          val32 = (int32_t)(d_val * 100 - 0.5);
        }
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 0xB:
      if (Get_Current_Switch_ADC(TX_SWITCH_CHANNEL, &val16_1, &val16_2, &val16_3, &val16_4)) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        BE16_To_Buffer(val16_1, pBuf);
        BE16_To_Buffer(val16_3, pBuf + 2);
      }
      break;
    case 0xC:
      if (Get_Current_Switch_ADC(TX_SWITCH_CHANNEL, &val16_1, &val16_2, &val16_3, &val16_4)) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        BE16_To_Buffer(val16_2, pBuf);
        BE16_To_Buffer(val16_4, pBuf + 2);
      }
      break;
    case 0xD:
      if (Get_Current_Switch_ADC(RX_SWITCH_CHANNEL, &val16_1, &val16_2, &val16_3, &val16_4)) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        BE16_To_Buffer(val16_1, pBuf);
        BE16_To_Buffer(val16_3, pBuf + 2);
      }
      break;
    case 0xE:
      if (Get_Current_Switch_ADC(RX_SWITCH_CHANNEL, &val16_1, &val16_2, &val16_3, &val16_4)) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        BE16_To_Buffer(val16_2, pBuf);
        BE16_To_Buffer(val16_4, pBuf + 2);
      }
      break;
    case 0xF:
      if (run_status.tx_block || run_status.tx_switch_channel >= 64) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        u_val8 = get_tap_pd_power(&u_val16, &d_val);
        if (u_val8 == 1) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
          BE32_To_Buffer(0x80000000, pBuf);
        } else if (u_val8 > 1) {
          BE32_To_Buffer(0x80000000, pBuf);
        } else {
          status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_TX_IL + run_status.tx_switch_channel * 4 + 2, buf, 2);
          d_val_2 = (double)(int16_t)Buffer_To_BE16(buf) / 100;
          d_val = d_val - d_val_2;
          if (d_val >= 0) {
            val32 = (int32_t)(d_val * 100 + 0.5);
          } else {
            val32 = (int32_t)(d_val * 100 - 0.5);
          }
          BE32_To_Buffer(val32, pBuf);
        }
      }
      break;
    case 0x10:
      if (run_status.rx_switch_channel >= 32) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        u_val8 = get_rx_pd_power(&u_val16, &d_val);
        if (u_val8 == 1) {
          Set_Flag(&run_status.internal_exp, INT_EXP_OS_ERR);
          BE32_To_Buffer(0x80000000, pBuf);
        } else if (u_val8 > 1) {
          BE32_To_Buffer(0x80000000, pBuf);
        } else {
          status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_RX_IL + run_status.rx_switch_channel * 4 + 2, buf, 2);
          d_val_2 = (double)(int16_t)Buffer_To_BE16(buf) / 100;
          d_val = d_val + d_val_2;
          if (d_val >= 0) {
            val32 = (int32_t)(d_val * 100 + 0.5);
          } else {
            val32 = (int32_t)(d_val * 100 - 0.5);
          }
          BE32_To_Buffer(val32, pBuf);
        }
      }
      break;
    case 0x11:
      if (run_status.tx_block || run_status.tx_switch_channel >= 64) {
        val32 = -60 * 100;
        BE32_To_Buffer(val32, pBuf);
      } else {
        status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_TX_IL + run_status.tx_switch_channel * 4 + 2, buf, 2);
        val32 = (int32_t)(int16_t)Buffer_To_BE16(buf);
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    case 0x12:
      if (run_status.rx_switch_channel >= 32) {
        BE32_To_Buffer(0x80000000, pBuf);
      } else {
        status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_RX_IL + run_status.rx_switch_channel * 4 + 2, buf, 2);
        val32 = (int32_t)(int16_t)Buffer_To_BE16(buf);
        BE32_To_Buffer(val32, pBuf);
      }
      break;
    default:
      return 1;
  }

  return 0;
}

uint8_t Set_Threshold(uint8_t alarm_id, int32_t val32_low, int32_t val32_high)
{
  switch (alarm_id) {
    case 0:
      run_status.thr_table.temp_low_alarm = (double)val32_low / 10;
      if (run_status.thr_table.temp_low_alarm < 0)
        run_status.thr_table.temp_low_clear = run_status.thr_table.temp_low_alarm * 0.99;
      else
        run_status.thr_table.temp_low_clear = run_status.thr_table.temp_low_alarm * 1.01;

      run_status.thr_table.temp_high_alarm = (double)val32_high / 10;
      if (run_status.thr_table.temp_high_alarm < 0)
        run_status.thr_table.temp_high_clear = run_status.thr_table.temp_high_alarm * 1.01;
      else
        run_status.thr_table.temp_high_clear = run_status.thr_table.temp_high_alarm * 0.99;
      break;
    case 1:
      run_status.thr_table.tec_temp_low_alarm = (double)val32_low / 10;
      if (run_status.thr_table.tec_temp_low_alarm < 0)
        run_status.thr_table.tec_temp_low_clear = run_status.thr_table.tec_temp_low_alarm * 0.99;
      else
        run_status.thr_table.tec_temp_low_clear = run_status.thr_table.tec_temp_low_alarm * 1.01;

      run_status.thr_table.tec_temp_high_alarm = (double)val32_high / 10;
      if (run_status.thr_table.tec_temp_high_alarm < 0)
        run_status.thr_table.tec_temp_high_clear = run_status.thr_table.tec_temp_high_alarm * 1.01;
      else
        run_status.thr_table.tec_temp_high_clear = run_status.thr_table.tec_temp_high_alarm * 0.99;
      break;
    case 2:
      run_status.thr_table.LD_cur_high_alarm = (double)val32_high;
      if (run_status.thr_table.LD_cur_high_alarm < 0)
        run_status.thr_table.LD_cur_high_clear = run_status.thr_table.LD_cur_high_alarm * 1.01;
      else
        run_status.thr_table.LD_cur_high_clear = run_status.thr_table.LD_cur_high_alarm * 0.99;
      break;
    case 3:
#if 1
      run_status.thr_table.tec_cur_low_alarm = (double)val32_low;
      if (run_status.thr_table.tec_cur_low_alarm < 0)
        run_status.thr_table.tec_cur_low_clear = run_status.thr_table.tec_cur_low_alarm * 0.99;
      else
        run_status.thr_table.tec_cur_low_clear = run_status.thr_table.tec_cur_low_alarm * 1.01;

      run_status.thr_table.tec_cur_high_alarm = (double)val32_high;
      if (run_status.thr_table.tec_cur_high_alarm < 0)
        run_status.thr_table.tec_cur_high_clear = run_status.thr_table.tec_cur_high_alarm * 1.01;
      else
        run_status.thr_table.tec_cur_high_clear = run_status.thr_table.tec_cur_high_alarm * 0.99;
#endif
      break;
    case 4:
#if 1
      run_status.thr_table.tec_vol_low_alarm = (double)val32_low;
      if (run_status.thr_table.tec_vol_low_alarm < 0)
        run_status.thr_table.tec_vol_low_clear = run_status.thr_table.tec_vol_low_alarm * 0.99;
      else
        run_status.thr_table.tec_vol_low_clear = run_status.thr_table.tec_vol_low_alarm * 1.01;

      run_status.thr_table.tec_vol_high_alarm = (double)val32_high;
      if (run_status.thr_table.tec_vol_high_alarm < 0)
        run_status.thr_table.tec_vol_high_clear = run_status.thr_table.tec_vol_high_alarm * 1.01;
      else
        run_status.thr_table.tec_vol_high_clear = run_status.thr_table.tec_vol_high_alarm * 0.99;
#endif
      break;
    case 5:
      run_status.thr_table.vol_5_0_low_alarm = (double)val32_low / 100;
      run_status.thr_table.vol_5_0_low_clear = run_status.thr_table.vol_5_0_low_alarm * 1.01;

      run_status.thr_table.vol_5_0_high_alarm = (double)val32_high / 100;
      run_status.thr_table.vol_5_0_high_clear = run_status.thr_table.vol_5_0_high_alarm * 0.99;
      break;
    case 6:
      run_status.thr_table.vol_3_3_low_alarm = (double)val32_low / 100;
      run_status.thr_table.vol_3_3_low_clear = run_status.thr_table.vol_3_3_low_alarm * 1.01;

      run_status.thr_table.vol_3_3_high_alarm = (double)val32_high / 100;
      run_status.thr_table.vol_3_3_high_clear = run_status.thr_table.vol_3_3_high_alarm * 0.99;
      break;
    case 7:
      run_status.thr_table.vol_4_4_low_alarm = (double)val32_low / 100;
      run_status.thr_table.vol_4_4_low_clear = run_status.thr_table.vol_4_4_low_alarm * 1.01;

      run_status.thr_table.vol_4_4_high_alarm = (double)val32_high / 100;
      run_status.thr_table.vol_4_4_high_clear = run_status.thr_table.vol_4_4_high_alarm * 0.99;
      break;
    case 8:
      run_status.thr_table.vol_61_0_low_alarm = (double)val32_low / 100;
      run_status.thr_table.vol_61_0_low_clear = run_status.thr_table.vol_61_0_low_alarm * 1.01;

      run_status.thr_table.vol_61_0_high_alarm = (double)val32_high / 100;
      run_status.thr_table.vol_61_0_high_clear = run_status.thr_table.vol_61_0_high_alarm * 0.99;
      break;
    case 9:
      break;
    case 10:
      break;
    default:
      return 1;
  }

  return 0;
}

uint8_t Get_Threshold(uint8_t alarm_id, uint8_t *pBuf)
{
  int32_t val32;

  switch (alarm_id) {
    case 0:
      if (run_status.thr_table.temp_low_alarm < 0)
        val32 = (int32_t)(run_status.thr_table.temp_low_alarm * 10 - 0.5);
      else
        val32 = (int32_t)(run_status.thr_table.temp_low_alarm * 10 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      if (run_status.thr_table.temp_high_alarm < 0)
        val32 = (int32_t)(run_status.thr_table.temp_high_alarm * 10 - 0.5);
      else
        val32 = (int32_t)(run_status.thr_table.temp_high_alarm * 10 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 1:
      if (run_status.thr_table.tec_temp_low_alarm < 0)
        val32 = (int32_t)(run_status.thr_table.tec_temp_low_alarm * 10 - 0.5);
      else
        val32 = (int32_t)(run_status.thr_table.tec_temp_low_alarm * 10 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      if (run_status.thr_table.tec_temp_high_alarm < 0)
        val32 = (int32_t)(run_status.thr_table.tec_temp_high_alarm * 10 - 0.5);
      else
        val32 = (int32_t)(run_status.thr_table.tec_temp_high_alarm * 10 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 2:
      BE32_To_Buffer(0x80000000, pBuf);
      val32 = (int32_t)run_status.thr_table.LD_cur_high_alarm;
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 3:
#if 1
      val32 = (int32_t)run_status.thr_table.tec_cur_low_alarm;
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)run_status.thr_table.tec_cur_high_alarm;
      BE32_To_Buffer(val32, pBuf + 4);
#else
      BE32_To_Buffer(0x80000000, pBuf);
      BE32_To_Buffer(0x7FFFFFFF, pBuf + 4);
#endif
      break;
    case 4:
#if 1
      val32 = (int32_t)run_status.thr_table.tec_vol_low_alarm;
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)run_status.thr_table.tec_vol_high_alarm;
      BE32_To_Buffer(val32, pBuf + 4);
#else
      BE32_To_Buffer(0x80000000, pBuf);
      BE32_To_Buffer(0x7FFFFFFF, pBuf + 4);
#endif
      break;
    case 5:
      val32 = (int32_t)(run_status.thr_table.vol_5_0_low_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)(run_status.thr_table.vol_5_0_high_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 6:
      val32 = (int32_t)(run_status.thr_table.vol_3_3_low_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)(run_status.thr_table.vol_3_3_high_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 7:
      val32 = (int32_t)(run_status.thr_table.vol_4_4_low_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)(run_status.thr_table.vol_4_4_high_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 8:
      val32 = (int32_t)(run_status.thr_table.vol_61_0_low_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf);
      val32 = (int32_t)(run_status.thr_table.vol_61_0_high_alarm * 100 + 0.5);
      BE32_To_Buffer(val32, pBuf + 4);
      break;
    case 9:
      BE32_To_Buffer(0x80000000, pBuf);
      BE32_To_Buffer(0x7FFFFFFF, pBuf + 4);
      break;
    case 10:
      BE32_To_Buffer(0x80000000, pBuf);
      BE32_To_Buffer(0x7FFFFFFF, pBuf + 4);
      break;
    default:
      return 1;
  }

  return 0;
}

uint8_t debug_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y)
{

  if (sw_num >= SWITCH_NUM_TOTAL) {
    return RESPOND_FAILURE;
  }
  
  if (set_sw_dac(sw_num, val_x, val_y)) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

int8_t set_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y)
{
  uint32_t i;
  uint16_t dac_px_val, dac_nx_val, dac_py_val, dac_ny_val;
  osStatus_t status;

  if (sw_num != SWITCH_ALL_CHANNEL) {
    for (i = 0; i < sizeof(switch_map)/sizeof(switch_map[0]); ++i) {
      if (switch_map[i].sw_num == sw_num) {
        // x
        if (val_x >= 0) {
          dac_px_val = (uint16_t)val_x;
          dac_nx_val = 0;
        } else {
          dac_px_val = 0;
          dac_nx_val = (uint16_t)my_abs(val_x);
        }
        // y
        if (val_y >= 0) {
          dac_py_val = (uint16_t)val_y;
          dac_ny_val = 0;
        } else {
          dac_py_val = 0;
          dac_ny_val = (uint16_t)my_abs(val_y);
        }
        break;
      }
    }
    // write dac
    status = RTOS_DAC5535_Write(switch_map[i].px_chan, dac_px_val);
    status |= RTOS_DAC5535_Write(switch_map[i].nx_chan, dac_nx_val);
    status |= RTOS_DAC5535_Write(switch_map[i].py_chan, dac_py_val);
    status |= RTOS_DAC5535_Write(switch_map[i].ny_chan, dac_ny_val);
    EPT("Set switch info : sw_num = %u, px = %u, nx = %u, py = %u, ny = %u\n", sw_num, dac_px_val, dac_nx_val, dac_py_val, dac_ny_val);
    if (status != osOK) {
      EPT("Write dac failed\n");
      return -1;
    }
    // DAC need 2ms at least
    osDelay(pdMS_TO_TICKS(2));
  } else {
    dac_px_val = (uint16_t)my_abs(val_x);
    for (i = 0; i < 32; ++i) {
      status = RTOS_DAC5535_Write((uint8_t)i, dac_px_val);
      if (status != osOK) {
        EPT("Write dac failed\n");
        return -1;
      }
    }
  }
  
  return 0;
}

void set_sw_dac_2(uint8_t sw_num, int32_t val_x, int32_t val_y)
{
  uint32_t i;
  uint16_t dac_px_val, dac_nx_val, dac_py_val, dac_ny_val;

  for (i = 0; i < sizeof(switch_map)/sizeof(switch_map[0]); ++i) {
    if (switch_map[i].sw_num == sw_num) {
      // x
      if (val_x >= 0) {
        dac_px_val = (uint16_t)val_x;
        dac_nx_val = 0;
      } else {
        dac_px_val = 0;
        dac_nx_val = (uint16_t)my_abs(val_x);
      }
      // y
      if (val_y >= 0) {
        dac_py_val = (uint16_t)val_y;
        dac_ny_val = 0;
      } else {
        dac_py_val = 0;
        dac_ny_val = (uint16_t)my_abs(val_y);
      }
      break;
    }
  }
  // write dac
  RTOS_DAC5535_Write_Nodelay(switch_map[i].px_chan, dac_px_val);
  RTOS_DAC5535_Write_Nodelay(switch_map[i].nx_chan, dac_nx_val);
  RTOS_DAC5535_Write_Nodelay(switch_map[i].py_chan, dac_py_val);
  RTOS_DAC5535_Write_Nodelay(switch_map[i].ny_chan, dac_ny_val);

  return;
}

uint8_t debug_sw_adc(uint8_t sw_num)
{
  uint32_t i;
  osStatus_t status;
  uint16_t px, nx, py, ny;

  if (sw_num >= SWITCH_NUM_TOTAL || sw_num == 0) {
    return RESPOND_FAILURE;
  }
  // THROW_LOG(MSG_TYPE_NORMAL_LOG, "run_status size is %u\n", sizeof(RunTimeStatus));

  for (i = 0; i < sizeof(switch_map)/sizeof(switch_map[0]); ++i) {
    if (switch_map[i].sw_num == sw_num)
      break;
  }
  status = RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_px_chan, &px);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_nx_chan, &nx);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_py_chan, &py);
  status |= RTOS_SWITCH_ADC7953_Read(switch_map[i].adc_ny_chan, &ny);
  if (status != osOK) {
    EPT("Read adc failed\n");
    return RESPOND_FAILURE;
  }
  i = px << 16;
  i |= nx << 0;
  BE32_To_Buffer(i, resp_buf.buf);
  i = py << 16;
  i |= ny << 0;
  BE32_To_Buffer(i, resp_buf.buf + 4);

  return RESPOND_SUCCESS;
}

uint8_t debug_vol_adc(uint8_t chan)
{
  osStatus_t status;
  uint16_t value;

  if (chan > 7) {
    return RESPOND_FAILURE;
  }

  status = RTOS_ADC7828_Read(chan, &value);
  if (status != osOK) {
    EPT("Read adc failed\n");
    return RESPOND_FAILURE;
  }
  BE32_To_Buffer((uint32_t)value, resp_buf.buf);

  return RESPOND_SUCCESS;
}

uint8_t debug_tag(uint8_t type, uint8_t *p, uint32_t length)
{
  osStatus_t status;
  uint16_t addr;
  uint8_t buf[TAG_MAX_SPACE];

  memset(buf, 0x20, TAG_MAX_SPACE);
  if (type > 4 || length > TAG_MAX_SPACE) {
    return RESPOND_FAILURE;
  }
  
  switch (type) {
    case 0:
      addr = EE_TAG_SN;
      break;
    case 1:
      addr = EE_TAG_DATE;
      break;
    case 2:
      addr = EE_TAG_PN;
      break;
    case 3:
      addr = EE_TAG_VENDOR;
      break;
    case 4:
      addr = EE_TAG_HW_VERSION;
      break;
    default:
      break;
  }

  memcpy(buf, p, length);
  status = RTOS_EEPROM_Write(EEPROM_ADDR, addr, buf, TAG_MAX_SPACE);

  if (status != osOK) {
    EPT("Read adc failed\n");
    return RESPOND_FAILURE;
  }

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_PN, (uint8_t*)pn, 16);
  pn[16] = 0;
  status |= RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_HW_VERSION, (uint8_t*)hw_version, 4);
  hw_version[4] = 0;
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_switch(uint8_t sw_num, uint32_t chan, int32_t val_x, int32_t val_y)
{
  osStatus status;
  uint16_t addr;
  uint32_t chan_count;

  if (sw_num >= SWITCH_NUM_TOTAL || sw_num == SWITCH_ALL_CHANNEL) {
    return RESPOND_FAILURE;
  }
  
  switch (sw_num) {
    case SWITCH_NUM_1:
      addr = EE_CAL_SWITCH1;
      chan_count = 6;
      break;
    case SWITCH_NUM_2:
      addr = EE_CAL_SWITCH2;
      chan_count = 11;
      break;
    case SWITCH_NUM_3:
      addr = EE_CAL_SWITCH3;
      chan_count = 11;
      break;
    case SWITCH_NUM_4:
      addr = EE_CAL_SWITCH4;
      chan_count = 11;
      break;
    case SWITCH_NUM_5:
      addr = EE_CAL_SWITCH5;
      chan_count = 3;
      break;
    case SWITCH_NUM_6:
      addr = EE_CAL_SWITCH6;
      chan_count = 11;
      break;
    case SWITCH_NUM_7:
      addr = EE_CAL_SWITCH7;
      chan_count = 11;
      break;
    case SWITCH_NUM_8:
      addr = EE_CAL_SWITCH8;
      chan_count = 11;
      break;
  }

  if (chan > chan_count || chan == 0) {
    return RESPOND_FAILURE;
  }
  status = write_32_to_eeprom(EEPROM_ADDR, addr + (chan - 1) * 8, (uint32_t)val_x);
  status |= write_32_to_eeprom(EEPROM_ADDR, addr + (chan - 1) * 8 + 4, (uint32_t)val_y);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_tosa(uint8_t num, uint32_t tosa_dac, uint32_t tec_dac, uint32_t pd_adc, int32_t pd)
{
  osStatus status;
  uint16_t addr;

  if (num > 9) {
    return RESPOND_FAILURE;
  }

  addr = EE_CAL_TOSA + num * 16;

  status = write_32_to_eeprom(EEPROM_ADDR, addr, (uint32_t)tosa_dac);
  status |= write_32_to_eeprom(EEPROM_ADDR, addr + 4, (uint32_t)tec_dac);
  status |= write_32_to_eeprom(EEPROM_ADDR, addr + 8, (uint32_t)pd_adc);
  status |= write_32_to_eeprom(EEPROM_ADDR, addr + 12, (uint32_t)pd);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  update_tosa_table();
  return RESPOND_SUCCESS;
}

uint8_t debug_get_tosa_val(int32_t val, uint32_t *resp_len)
{
  /*
  double power = (double)val / 100;
  TosaCalData data;
  
  data = Get_Tosa_Data(power);
  
  if (data.tosa_dac < 10) {
    data.tec_dac = 1;
    data.tap_adc = 1;
    data.tap_power = 1;
  }
  */
  
  BE32_To_Buffer((uint32_t)run_status.tosa_high.tosa_dac, resp_buf.buf);
  BE32_To_Buffer((uint32_t)run_status.tosa_high.tec_dac, resp_buf.buf + 4);
  BE32_To_Buffer((uint32_t)run_status.tosa_high.tap_adc, resp_buf.buf + 8);
  BE32_To_Buffer((uint32_t)(int32_t)(run_status.tosa_high.tap_power * 100), resp_buf.buf + 12);

  BE32_To_Buffer((uint32_t)run_status.tosa_low.tosa_dac, resp_buf.buf + 16);
  BE32_To_Buffer((uint32_t)run_status.tosa_low.tec_dac, resp_buf.buf + 20);
  BE32_To_Buffer((uint32_t)run_status.tosa_low.tap_adc, resp_buf.buf + 24);
  BE32_To_Buffer((uint32_t)(int32_t)(run_status.tosa_low.tap_power * 100), resp_buf.buf + 28);

  *resp_len = 32;

  return RESPOND_SUCCESS;
}

uint8_t debug_pin(uint8_t pin, uint8_t val)
{
  GPIO_TypeDef *gpio_port;
  uint16_t gpio_pin;

  switch (pin) {
    case 0:
      gpio_port = LATCH_GPIO_Port;
      gpio_pin = LATCH_Pin;
      break;
    case 1:
      gpio_port = ALARM_GPIO_Port;
      gpio_pin = ALARM_Pin;
      break;
    case 2:
      gpio_port = SW1_READY_GPIO_Port;
      gpio_pin = SW1_READY_Pin;
      break;
    case 3:
      gpio_port = SW2_READY_GPIO_Port;
      gpio_pin = SW2_READY_Pin;
      break;
    case 4:
      gpio_port = L_READY_N_GPIO_Port;
      gpio_pin = L_READY_N_Pin;
      break;
    case 5:
      if (HAL_GPIO_ReadPin(SW1_BLOCK_GPIO_Port, SW1_BLOCK_Pin) == GPIO_PIN_SET) {
        return RESPOND_SUCCESS;
      } else {
        return RESPOND_FAILURE;
      }
    default:
      return RESPOND_FAILURE;
  }
  
  if (val) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_il(uint8_t num, int32_t val)
{
  osStatus status;
  uint16_t addr = EE_CAL_TX_IL;

  if (num > 97 || num == 0) {
    return RESPOND_FAILURE;
  }

  status = write_32_to_eeprom(EEPROM_ADDR, addr + (num - 1) * 4, (uint32_t)val);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_default_temp(int32_t val)
{
  osStatus status;
  uint16_t addr = EE_TEC_DEF_TEMP;

  status = write_32_to_eeprom(EEPROM_ADDR, addr, (uint32_t)val);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_rx_pd(uint8_t num, uint32_t adc, int32_t val)
{
  osStatus status;
  uint16_t addr = EE_CAL_RX_PD;

  if (num > 10 || num == 0) {
    return RESPOND_FAILURE;
  }

  status = write_32_to_eeprom(EEPROM_ADDR, addr + (num - 1) * 8, adc);
  status |= write_32_to_eeprom(EEPROM_ADDR, addr + (num - 1) * 8 + 4, (uint32_t)val);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

uint8_t debug_cal_dump(uint32_t which, uint32_t *resp_len)
{
  osStatus_t status;
  uint32_t len;
  uint16_t addr;
  
  switch (which) {
    case 0: // IL
      len = 0;
      addr = 0;
      break;
    case 1: // Switch 1
      len = 6 * 8;
      addr = EE_CAL_SWITCH1;
      break;
    case 2: // Switch 2
      len = 11 * 8;
      addr = EE_CAL_SWITCH2;
      break;
    case 3: // Switch 3
      len = 11 * 8;
      addr = EE_CAL_SWITCH3;
      break;
    case 4: // Switch 4
      len = 11 * 8;
      addr = EE_CAL_SWITCH4;
      break;
    case 5: // Switch 5
      len = 3 * 8;
      addr = EE_CAL_SWITCH5;
      break;
    case 6: // Switch 6
      len = 11 * 8;
      addr = EE_CAL_SWITCH6;
      break;
    case 7: // Switch 7
      len = 11 * 8;
      addr = EE_CAL_SWITCH7;
      break;
    case 8: // Switch 8
      len = 11 * 8;
      addr = EE_CAL_SWITCH8;
      break;
    case 9: // Threshold
      len = 0;
      addr = 0;
      break;
    case 10: // Tosa
      len = 10 * 16;
      addr = EE_CAL_TOSA;
      break;
    case 11: // TX 1 to n IL
      len = 32 * 4;
      addr = EE_CAL_TX_IL;
      break;
    case 12: // TX 2 to n IL
      len = 32 * 4;
      addr = EE_CAL_TX_IL + 32 * 4;
      break;
    case 13: // RX IL
      len = 32 * 4;
      addr = EE_CAL_RX_IL;
      break;
    case 14: // Loopback IL
      len = 4;
      addr = EE_CAL_LB_IL;
      break;
    case 15: // RX PD
      len = 8 * 10;
      addr = EE_CAL_RX_PD;
      break;
    case 16:
      len = 4;
      addr = EE_TEC_DEF_TEMP;
      break;
  }
  status = RTOS_EEPROM_Read(EEPROM_ADDR, addr, resp_buf.buf, len);
  if (status != osOK) {
    EPT("Read EEPROM failed, status = %d\n", status);
    *resp_len = 4;
    return RESPOND_FAILURE;
  }
  
  *resp_len = len;
  return RESPOND_SUCCESS;
}

uint8_t debug_eeprom(uint32_t addr, uint32_t *len)
{
  osStatus_t status;

  if (*len > 240) {
    *len = 0;
    return RESPOND_INVALID_PARA;
  }
  status = RTOS_EEPROM_Read(EEPROM_ADDR, addr, resp_buf.buf, *len);
  if (status != osOK) {
    EPT("Read EEPROM failed, status = %d\n", status);
    *len = 4;
    return RESPOND_FAILURE;
  }
  
  return RESPOND_SUCCESS;
}

uint8_t debug_test_tosa()
{
  osStatus_t status;
  uint16_t value;

  status = RTOS_DAC128S085_Write(1, 4095, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_Write(2, 4095, DAC128S085_MODE_NORMAL);

  if (status != osOK) {
    EPT("Read adc failed\n");
    return RESPOND_FAILURE;
  }
  BE32_To_Buffer((uint32_t)value, resp_buf.buf);

  return RESPOND_SUCCESS;
}

uint8_t debug_set_tosa(uint16_t low, uint16_t high)
{
  run_status.tosa_low.tosa_dac = low;
  run_status.tosa_high.tosa_dac = high;

  DAC5541_Write(high);
  return RESPOND_SUCCESS;
}

uint8_t debug_set_tec(uint16_t value)
{
  osStatus_t status;

  status = RTOS_DAC128S085_Write(0, value, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    return RESPOND_FAILURE;
  }

  return RESPOND_SUCCESS;
}

 uint8_t debug_get_tosa_tmp()
{
  osStatus_t status;
  uint16_t value;
  double temp;

  status = RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_TEMP_CHANNEL, &value);
  temp = Cal_Tosa_Temp(value);
  EPT("Temperature = %lf\n", temp);
  BE32_To_Buffer((uint32_t)value, resp_buf.buf);
  value = (uint16_t)(int16_t)(temp * 100);
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 4);

  status |= RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_VOLTAGE_CHANNEL, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (temp - 1.25) / 0.25 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 8);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 12);

  status |= RTOS_ADC7953_SPI5_Read(TEC_ADC_TEC_CURRENT_CHANNEL, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (temp - 1.25) / 0.285 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 16);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 20);

  status |= RTOS_ADC7953_SPI5_Read(TEC_ADC_LD_CURRENT_CHANNEL, &value);
  temp = (double)value / 4096 * 2.5;
  temp = temp / 22 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 24);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 28);

  status |= RTOS_ADC7953_SPI5_Read(TEC_ADC_LD_VOLTAGE_CHANNEL, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (3.3 - temp * 2) * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 32);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 36);

  status |= RTOS_ADC7953_SPI5_Read(TEC_ADC_MPD_CURRENT_CHANNEL, &value);
  temp = (double)value / 4096 * 2.5 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 40);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 44);

  if (status != osOK) {
    return RESPOND_FAILURE;
  }
  return RESPOND_SUCCESS;
}

uint8_t debug_get_pd(uint32_t which)
{
  uint8_t ret;
  uint16_t adc;
  double power;

  if (which == 0) {
    ret = get_tap_pd_power(&adc, &power);
  } else if (which == 1) {
    ret = get_rx_pd_power(&adc, &power);
  } else {
    BE32_To_Buffer(0xFF, resp_buf.buf);
    return RESPOND_FAILURE;
  }

  if (ret) {
    BE32_To_Buffer((uint32_t)ret, resp_buf.buf);
    return RESPOND_FAILURE;
  }

  BE32_To_Buffer((uint32_t)adc, resp_buf.buf);
  BE32_To_Buffer((int32_t)(power * 100), resp_buf.buf + 4);

  return RESPOND_SUCCESS;
}

uint8_t debug_set_lp(uint32_t which)
{
  uint8_t switch_channel, switch_pos;

  if (which == 0) {
    switch_channel = TX_SWITCH_CHANNEL;
    switch_pos = 64;
  } else if (which == 1) {
    switch_channel = TX_SWITCH_CHANNEL;
    switch_pos = 65;
  } else if (which == 2) {
    switch_channel = RX_SWITCH_CHANNEL;
    switch_pos = 32;
  } else {
    return RESPOND_FAILURE;
  }

  if (run_status.tx_block && switch_channel == TX_SWITCH_CHANNEL) {
    return RESPOND_FAILURE;
  }

  Clear_Switch_Ready(switch_channel);

  if (Set_Switch(switch_channel, switch_pos)) {
    if (switch_channel == TX_SWITCH_CHANNEL) {
      run_status.tx_switch_channel = 0xFF;
    } else {
      run_status.rx_switch_channel = 0xFF;
    }
    return RESPOND_FAILURE;
  }

  if (switch_channel == TX_SWITCH_CHANNEL) {
    run_status.tx_switch_channel = switch_pos;
  } else {
    run_status.rx_switch_channel = switch_pos;
  }

  // Check
  if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
    Reset_Switch_Only(switch_channel);
    return RESPOND_FAILURE;
  }

  Set_Switch_Ready(switch_channel);
  return RESPOND_SUCCESS;
}

uint8_t debug_get_switch_channel(uint8_t switch_channel)
{
  int8_t ret;

  resp_buf.buf[0] = switch_channel;
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (run_status.tx_block) {
      resp_buf.buf[1] = 0xFF;
      return RESPOND_SUCCESS;
    }

    ret = Get_Current_Switch_Channel(switch_channel);
    if (ret == -2) {
      resp_buf.buf[1] = 0xFF;
    } else if (ret < 0) {
      resp_buf.buf[1] = 0xFF;
      return RESPOND_FAILURE;
    } else if (ret >= 66) {
      resp_buf.buf[1] = 0xFF;
    } else {
      resp_buf.buf[1] = ret;
    }
    return RESPOND_SUCCESS;
  } else if (switch_channel == RX_SWITCH_CHANNEL) {
    ret = Get_Current_Switch_Channel(switch_channel);
    if (ret == -2) {
      resp_buf.buf[1] = 0xFF;
      return RESPOND_INVALID_PARA;
    } else if (ret < 0) {
      resp_buf.buf[1] = 0xFF;
      return RESPOND_FAILURE;
    } else if (ret >= 33) {
      resp_buf.buf[1] = 0xFF;
      return RESPOND_INVALID_PARA;
    } else {
      resp_buf.buf[1] = ret;
    }
    return RESPOND_SUCCESS;
  } else {
    return RESPOND_INVALID_PARA;
  }
}

uint8_t debug_get_inter_exp(void)
{
  BE32_To_Buffer(run_status.internal_exp, resp_buf.buf);
  return RESPOND_SUCCESS;
}

uint8_t debug_Check_Cali(void)
{
  uint16_t start = 0x1000;
  uint16_t end = EE_PARA_TABLE_END;
  uint16_t offset;
  uint8_t buf[4];
  uint32_t crc32 = 0xFFFFFFFF;
  
  for (offset = start; offset < end; offset += 4) {
    if (RTOS_EEPROM_Read(EEPROM_ADDR, offset, buf, 4) != osOK) {
      return RESPOND_FAILURE;
    }
    
    crc32 = Cal_CRC32_2(buf, 4, crc32);
  }
  
  crc32 = ~crc32;

  BE32_To_Buffer(crc32, resp_buf.buf);
  if (RTOS_EEPROM_Write(EEPROM_ADDR, EE_CALI_CHECK, resp_buf.buf, 4) !=osOK) {
    return RESPOND_FAILURE;
  }
  
  return RESPOND_SUCCESS;
}


