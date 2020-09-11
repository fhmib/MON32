#include "main.h"
#include "command.h"
#include "flash_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "i2c.h"
#include "spi.h"
#include "rtc.h"

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

extern RespondStu resp_buf;

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
  int val_x, val_y;
  osStatus_t status;
  uint8_t pre_pos;

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

  if (pre_pos != 0xFF) {
    pre_index = Get_Index_Of_Channel_Map(switch_channel, pre_pos);

    if (pre_index != index) {
      Clear_Switch_Dac(channel_map[pre_index].second_switch);
    } else {
      Clear_Switch_Dac(first_switch);
    }
  }

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

  // first switch
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (switch_pos < 32) {
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

  if (set_sw_dac(first_switch, val_x, val_y)) {
    EPT("Write DAC faliled\n");
    return -7;
  }

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
    THROW_LOG("Invalid switch channel number : %u %u\n", switch_channel, pos);
    return -2;
  }

  first_switch = channel_map[index].first_switch;
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (pos < 32) {
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
  if (!Is_Value_Approximate(val_x, my_abs(std_x), 0.05) || !Is_Value_Approximate(val_y, my_abs(std_y), 0.05)) {
    EPT("First level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    THROW_LOG("First level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    return -5;
  }

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
  if (!Is_Value_Approximate(val_x, my_abs(std_x), 0.05) || !Is_Value_Approximate(val_y, my_abs(std_y), 0.05)) {
    EPT("Second level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    THROW_LOG("Second level switch DA/AD different: %u, %d, %u, %d\n", val_x, std_x, val_y, std_y);
    return -9;
  }

  if (switch_channel == TX_SWITCH_CHANNEL) {
    return run_status.tx_switch_channel;
  } else {
    return run_status.rx_switch_channel;
  }
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
    } else if (switch_channel < 66) {
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
      index = Get_Index_Of_Channel_Map(switch_channel, pos);
      Clear_Switch_Dac(channel_map[index].first_switch);
      Clear_Switch_Dac(channel_map[index].second_switch);
    }

    if (switch_channel == TX_SWITCH_CHANNEL) {
      run_status.tx_switch_channel = 0xFF;
      Clear_Switch_Ready(TX_SWITCH_CHANNEL);
    } else {
      run_status.rx_switch_channel = 0xFF;
      Clear_Switch_Ready(RX_SWITCH_CHANNEL);
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
    run_status.tx_switch_channel = 0xFF;
    run_status.rx_switch_channel = 0xFF;
    Clear_Switch_Ready(TX_SWITCH_CHANNEL);
    Clear_Switch_Ready(RX_SWITCH_CHANNEL);
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

void Init_Run_Status(void)
{
  run_status.maigc = RUN_MAGIC;
  run_status.uart_reset = 0;
  run_status.tx_switch_channel = 0xFF;
  run_status.rx_switch_channel = 0xFF;
  run_status.exp = 0;
  //run_status.tosa_high = 30000;
  //run_status.tosa_low = 0;
  run_status.modulation = 0;
  if (HAL_GPIO_ReadPin(SW1_BLOCK_GPIO_Port, SW1_BLOCK_Pin) == GPIO_PIN_RESET) {
    run_status.tx_block = 1;
  } else {
    run_status.tx_block = 0;
  }
  if (HAL_GPIO_ReadPin(PRO_DIS_N_GPIO_Port, PRO_DIS_N_Pin) == GPIO_PIN_RESET) {
    run_status.tosa_enable = 1;
  } else {
    run_status.tosa_enable = 0;
  }
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
  uint32_t i = 0;

  status = RTOS_DAC128S085_Write(1, 4095, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    return 1;
  }
  while (HAL_GPIO_ReadPin(TMPGD_GPIO_Port, TMPGD_Pin) == GPIO_PIN_RESET) {
    osDelay(1);
    if (++i > 100) {
      Set_Flag(&run_status.internal_exp, INT_EXP_TMPGD);
      return 2;
    }
  }
  status = RTOS_DAC128S085_Write(2, 4095, DAC128S085_MODE_NORMAL);
  if (status != osOK) {
    return 3;
  }

  return 0;
}

uint8_t Disable_Tosa()
{
  osStatus_t status;

  status = RTOS_DAC128S085_Write(2, 0, DAC128S085_MODE_NORMAL);
  status |= RTOS_DAC128S085_Write(1, 0, DAC128S085_MODE_NORMAL);

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
    if (val >> 12) {
      break;
    }
    val = Buffer_To_BE32(buf + 8);
    if (val >> 12) {
      break;
    }
    tosa_table[i].tosa_dac = (uint16_t)Buffer_To_BE32(buf);
    tosa_table[i].tec_dac = (uint16_t)Buffer_To_BE32(buf + 4);
    tosa_table[i].tap_adc = (uint16_t)Buffer_To_BE32(buf + 8);
    val = (int32_t)Buffer_To_BE32(buf + 12);
    tosa_table[i].tap_power = (double)val / 100;
    if (tosa_table[i].tap_power > 5 || tosa_table[i].tap_power < -15) {
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
  tosa_data.tap_adc = (uint16_t)(y + 0.5);

  x1 = (double)tosa_node_1.tap_adc; x2 = (double)tosa_node_2.tap_adc;
  y1 = (double)tosa_node_1.tosa_dac; y2 = (double)tosa_node_2.tosa_dac;
  x = tosa_data.tap_adc;
  y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
  tosa_data.tosa_dac = (uint16_t)(y + 0.5);

  x1 = (double)tosa_node_1.tosa_dac; x2 = (double)tosa_node_2.tosa_dac;
  y1 = (double)tosa_node_1.tec_dac; y2 = (double)tosa_node_2.tec_dac;
  x = tosa_data.tosa_dac;
  y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
  tosa_data.tec_dac = (uint16_t)(y + 0.5);
  
  return tosa_data;
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
    osDelay(pdMS_TO_TICKS(4));
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

uint8_t debug_sw_adc(uint8_t sw_num)
{
  uint32_t i;
  osStatus_t status;
  uint16_t px, nx, py, ny;

  if (sw_num >= SWITCH_NUM_TOTAL || sw_num == 0) {
    return RESPOND_FAILURE;
  }

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
  uint8_t buf[TAG_MAX_SPACE] = {0x20};

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
  status = RTOS_EEPROM_Write(EEPROM_ADDR, addr, buf, sizeof(buf));

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

  status = RTOS_ADC7953_SPI5_Read(0, &value);
  temp = Cal_Tosa_Temp(value);
  EPT("Temperature = %lf\n", temp);
  BE32_To_Buffer((uint32_t)value, resp_buf.buf);
  value = (uint16_t)(int16_t)(temp * 100);
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 4);

  status |= RTOS_ADC7953_SPI5_Read(1, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (temp - 1.25) / 0.25 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 8);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 12);

  status |= RTOS_ADC7953_SPI5_Read(2, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (temp - 1.25) / 0.285 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 16);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 20);

  status |= RTOS_ADC7953_SPI5_Read(3, &value);
  temp = (double)value / 4096 * 2.5;
  temp = temp / 22 * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 24);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 28);

  status |= RTOS_ADC7953_SPI5_Read(4, &value);
  temp = (double)value / 4096 * 2.5;
  temp = (3.3 - temp * 2) * 1000;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 32);
  value = (uint16_t)(int16_t)temp;
  BE32_To_Buffer((uint32_t)value, resp_buf.buf + 36);

  status |= RTOS_ADC7953_SPI5_Read(5, &value);
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

uint8_t debug_get_inter_exp(void)
{
  BE32_To_Buffer(run_status.internal_exp, resp_buf.buf);
  return RESPOND_SUCCESS;
}



