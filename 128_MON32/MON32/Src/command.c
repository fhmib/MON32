#include "main.h"
#include "command.h"
#include "flash_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "i2c.h"
#include "spi.h"
#include "rtc.h"
#include "functions.h"
#include "tim.h"

RespondStu resp_buf;
TransStu trans_buf;

UpgradeStruct up_state;

char pn[17];
char hw_version[5];
char *fw_version = "S1.0"; // 4 bytes

extern osMessageQueueId_t mid_LazerManager;
extern osMessageQueueId_t mid_CmdProcess;

extern UpgradeFlashState upgrade_status;

CmdStruct command_list[] = {
  {CMD_QUERY_SN, 4, Cmd_Query_SN},
  {CMD_QUERY_MDATE, 4, Cmd_Query_MDate},
  {CMD_QUERY_PN, 4, Cmd_Query_PN},
  {CMD_QUERY_VENDOR, 4, Cmd_Query_Vendor},
  {CMD_QUERY_VERSION, 4, Cmd_Get_Version},
  {CMD_SET_SWITCH, 6, Cmd_Set_Switch},
  {CMD_QUERY_SWITCH, 5, Cmd_Get_Switch},
  {CMD_QUERY_TEMP, 4, Cmd_Get_Temperature},
  {CMD_QUERY_IL, 5, Cmd_Get_IL},
  {CMD_QUERY_TOSA_THR, 4, Cmd_Query_Tosa_Thr},
//  {CMD_SET_TOSA, 8, Cmd_Set_Tosa},
  {CMD_QUERY_TOSA, 4, Cmd_Query_Tosa},
  {CMD_RX_PD_CALI, 4, Cmd_RX_PD_CALI},
  {CMD_TAP_PD_CALI, 4, Cmd_TAP_PD_CALI},
  {CMD_QUERY_VOLTAGE, 4, Cmd_Voltage},
//  {CMD_LOOPBACK_TEST, 5, Cmd_Loopback_Test},
//  {CMD_LOOPBACK_RESULT, 4, Cmd_Loopback_Result},
  {CMD_QUERY_ALARM, 4, Cmd_Query_Alarm},
  {CMD_SET_MODULATION, 5, Cmd_Set_Modulation},
  {CMD_QUERY_MODULATION, 4, Cmd_Query_Modulation},
  {CMD_QUERY_STATUS, 4, Cmd_Query_Status},
//  {CMD_QUERY_ALARM_HISTORY, 4, Cmd_Query_Alarm_History},
  {CMD_UPGRADE_MODE, 5, Cmd_Upgrade_Init},
  {CMD_UPGRADE_DATA, 0x86, Cmd_Upgrade_Data},
  {CMD_UPGRADE_RUN, 4, Cmd_Upgrade_Install},
  {CMD_SOFTRESET, 4, Cmd_Softreset},
//  {CMD_SET_LOG_TIME, 0xA, Cmd_Set_Time},
//  {CMD_QUERY_LOG_TIME, 4, Cmd_Get_Time},
  {CMD_QUERY_PERFORMANCE, 0xFFFF, Cmd_Performance},
  {CMD_SET_THRESHOLD, 0xD, Cmd_Set_Threshold},
  {CMD_QUERY_THRESHOLD, 5, Cmd_Query_Threshold},
//  {CMD_QUERY_LOG_SIZE, 5, Cmd_LOG_Size},
//  {CMD_GET_LOG_CONTENT, 0xA, Cmd_LOG_Content},

  {CMD_FOR_DEBUG, 0xFFFF, Cmd_For_Debug},
};

uint8_t Cmd_Process()
{
  int i;
  uint16_t cmd_id;
  uint8_t cmd_length;

  cmd_id = switch_endian_16(*(uint16_t*)&trans_buf.buf[CMD_SEQ_MSG_ID]);
  cmd_length = trans_buf.buf[CMD_SEQ_MSG_LENGTH];

  for (i = 0; i < sizeof(command_list) / sizeof(command_list[0]); ++i) {
    if (cmd_id == command_list[i].cmd_id) {
      //EPT("Command id = %#X\n", cmd_id);
      if (command_list[i].func == NULL) {
        break;
      }
      if (command_list[i].cmd_std_len != 0xFFFF) {
        if (cmd_length != (uint8_t)command_list[i].cmd_std_len) {
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Command %#X Length %#X invalid\n", cmd_id, cmd_length);
          FILL_RESP_MSG(cmd_id, RESPOND_INVALID_PARA, 0);
          return RESPOND_INVALID_PARA;
        }
      }
      return command_list[i].func();
    }
  }

  THROW_LOG(MSG_TYPE_NORMAL_LOG, "Unknow command id = %#X\n", cmd_id);
  FILL_RESP_MSG(cmd_id, RESPOND_UNKNOWN_CMD, 0);
  return RESPOND_UNKNOWN_CMD;
}

int Uart_Respond(uint16_t cmd, uint8_t status, uint8_t *pdata, uint8_t len)
{
  uint8_t cmd_len = 0;

  trans_buf.buf[cmd_len++] = TRANS_START_BYTE;
  trans_buf.buf[cmd_len++] = 1 + 2 + len + 1 + 1; // Length + command + data + status + chk
  trans_buf.buf[cmd_len++] = (uint8_t)(cmd >> 8);
  trans_buf.buf[cmd_len++] = (uint8_t)cmd;
  if (len) {
    memcpy(&trans_buf.buf[cmd_len], pdata, len);
    cmd_len += len;
  }
  trans_buf.buf[cmd_len++] = (uint8_t)status;
  trans_buf.buf[cmd_len++] = Cal_Check((uint8_t*)&trans_buf.buf[CMD_SEQ_MSG_LENGTH], 1 + 2 + len + 1);
  
  if (HAL_UART_Transmit(&huart1, trans_buf.buf, cmd_len, 0xFF) != HAL_OK) {
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Respond command failed, ErrorCode = %#X\n", huart1.ErrorCode);
    EPT("Respond command failed, ErrorCode = %#X\n", huart1.ErrorCode);
      return -1;
  } else {
    if (status != 0)
      EPT("Respond command = %#X, status = %#X\n", cmd, status);
    return 0;
  }
}

uint8_t Cmd_Query_SN(void)
{
  osStatus_t status = osOK;

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_SN, resp_buf.buf, 9);
  if (status != osOK) {
    EPT("EEPROM ERROR! status = %#X", status);
  }

  FILL_RESP_MSG(CMD_QUERY_SN, RESPOND_SUCCESS, 9);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_MDate(void)
{
  osStatus_t status = osOK;

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_DATE, resp_buf.buf, 8);
  if (status != osOK) {
    EPT("EEPROM ERROR! status = %#X", status);
  }

  FILL_RESP_MSG(CMD_QUERY_MDATE, RESPOND_SUCCESS, 8);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_PN(void)
{
  osStatus_t status = osOK;

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_PN, resp_buf.buf, 16);
  if (status != osOK) {
    EPT("EEPROM ERROR! status = %#X", status);
  }

  FILL_RESP_MSG(CMD_QUERY_PN, RESPOND_SUCCESS, 16);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Vendor(void)
{
  osStatus_t status = osOK;

  status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_TAG_VENDOR, resp_buf.buf, 16);
  if (status != osOK) {
    EPT("EEPROM ERROR! status = %#X", status);
  }

  FILL_RESP_MSG(CMD_QUERY_VENDOR, RESPOND_SUCCESS, 16);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Get_Version(void)
{
  uint8_t *p = resp_buf.buf;
  
  memcpy(p, hw_version, 4);
  memcpy(p + 4, fw_version, 4);
  FILL_RESP_MSG(CMD_QUERY_VERSION, RESPOND_SUCCESS, 8);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Set_Switch(void)
{
  uint8_t switch_channel, switch_pos;

  memset(resp_buf.buf, 0, 4);
  
  switch_channel = trans_buf.buf[CMD_SEQ_MSG_DATA];
  switch_pos = trans_buf.buf[CMD_SEQ_MSG_DATA + 1];

  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (HAL_GPIO_ReadPin(SW1_MODE_SEL_GPIO_Port, SW1_MODE_SEL_Pin) == GPIO_PIN_RESET) {
      EPT("Switch 2x32 command come but switch mode is wrong\n");
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch 2x32 command come but switch mode is wrong\n");
      FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
    
    if(switch_pos > 0x3F) {
      EPT("Switch 2x32 command parameter is invalid\n");
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch 2x32 command parameter is invalid\n");
      FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_INVALID_PARA, 0);
      return RESPOND_INVALID_PARA;
    }
  } else if (switch_channel == RX_SWITCH_CHANNEL) {
    if (HAL_GPIO_ReadPin(SW2_MODE_SEL_GPIO_Port, SW2_MODE_SEL_Pin) == GPIO_PIN_RESET) {
      EPT("Switch 1x32 command come but switch mode is wrong\n");
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch 1x32 command come but switch mode is wrong\n");
      FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
    
    if(switch_pos > 0x1F) {
      EPT("Switch 1x32 command parameter is invalid\n");
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch 1x32 command parameter is invalid\n");
      FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_INVALID_PARA, 0);
      return RESPOND_INVALID_PARA;
    }
  } else {
    EPT("Switch command parameter is invalid\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch command parameter is invalid\n");
    FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  
  if (run_status.tx_block && switch_channel == TX_SWITCH_CHANNEL) {
    EPT("Switch is blocked\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch is blocked\n\n");
    FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  Clear_Switch_Ready(switch_channel);

  if (Set_Switch(switch_channel, switch_pos)) {
    if (switch_channel == TX_SWITCH_CHANNEL) {
      run_status.tx_switch_channel = 0xFF;
    } else {
      run_status.rx_switch_channel = 0xFF;
    }
    EPT("Set switch channel failed\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Set switch channel failed\n");
    FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  if (switch_channel == TX_SWITCH_CHANNEL) {
    run_status.tx_switch_channel = switch_pos;
  } else {
    run_status.rx_switch_channel = switch_pos;
  }

  // Check
  if (Get_Current_Switch_Channel(switch_channel) != switch_pos) {
    Reset_Switch(switch_channel);
    EPT("Set switch channel failed 2\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Set switch channel failed 2\n");
    FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  Set_Switch_Ready(switch_channel);
  FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_SUCCESS, 0);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Get_Switch(void)
{
  uint8_t switch_channel = trans_buf.buf[CMD_SEQ_MSG_DATA];
  int8_t ret;

  resp_buf.buf[0] = switch_channel;
  if (switch_channel == TX_SWITCH_CHANNEL) {
    if (run_status.tx_block) {
      resp_buf.buf[1] = 0xFF;
      FILL_RESP_MSG(CMD_SET_SWITCH, RESPOND_SUCCESS, 2);
      return RESPOND_SUCCESS;
    }

    ret = Get_Current_Switch_Channel(switch_channel);
    if (ret == -2) {
      resp_buf.buf[1] = 0xFF;
    } else if (ret < 0) {
      resp_buf.buf[1] = 0xFF;
      FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_FAILURE, 2);
      return RESPOND_FAILURE;
    } else if (ret >= 64) {
      resp_buf.buf[1] = 0xFF;
    } else {
      resp_buf.buf[1] = ret;
    }
    FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_SUCCESS, 2);
    return RESPOND_SUCCESS;
  } else if (switch_channel == RX_SWITCH_CHANNEL) {
    ret = Get_Current_Switch_Channel(switch_channel);
    if (ret == -2) {
      resp_buf.buf[1] = 0xFF;
      FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_INVALID_PARA, 2);
      return RESPOND_INVALID_PARA;
    } else if (ret < 0) {
      resp_buf.buf[1] = 0xFF;
      FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_FAILURE, 2);
      return RESPOND_FAILURE;
    } else if (ret >= 32) {
      resp_buf.buf[1] = 0xFF;
      FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_INVALID_PARA, 2);
      return RESPOND_INVALID_PARA;
    } else {
      resp_buf.buf[1] = ret;
    }
    FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_SUCCESS, 2);
    return RESPOND_SUCCESS;
  } else {
    EPT("Switch command parameter is invalid\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Switch command parameter is invalid\n");
    FILL_RESP_MSG(CMD_QUERY_SWITCH, RESPOND_INVALID_PARA, 2);
    return RESPOND_INVALID_PARA;
  }
}

uint8_t Cmd_Get_Temperature(void)
{
  osStatus_t status;
  uint16_t value;
  int16_t *p = (int16_t*)resp_buf.buf;
  double temp;
  int16_t res;

  status = RTOS_ADC7828_Read(TEMPERATURE_CHANNEL, &value);
  if (status != osOK) {
    EPT("Read adc7828 failed\n");
    FILL_RESP_MSG(CMD_QUERY_TEMP, RESPOND_FAILURE, 0);
    return RESPOND_SUCCESS;
  }
  temp = Cal_Temp(value);
  if (temp >= 0)
    res = (int16_t)(temp * 10 + 0.5);
  else
    res = (int16_t)(temp * 10 - 0.5);

  *p = switch_endian_16((uint16_t)res);

  FILL_RESP_MSG(CMD_QUERY_TEMP, RESPOND_SUCCESS, 2);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Get_IL(void)
{
  osStatus_t status = osOK;
  uint8_t which = trans_buf.buf[CMD_SEQ_MSG_DATA];
  uint32_t i;

  if (which == 0) {
    resp_buf.buf[0] = 0;
    for (i = 0; i < 64; ++i) {
      status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_TX_IL + i * 4 + 2, resp_buf.buf + 1 + i * 2, 2);
    }
    if (status != osOK) {
      EPT("Read EEPROM failed, status = %d\n", status);
      FILL_RESP_MSG(CMD_QUERY_IL, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
  } else if (which == 1) {
    resp_buf.buf[0] = 1;
    for (i = 0; i < 32; ++i) {
      status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_RX_IL + i * 4 + 2, resp_buf.buf + 1 + i * 2, 2);
    }
    if (status != osOK) {
      EPT("Read EEPROM failed, status = %d\n", status);
      FILL_RESP_MSG(CMD_QUERY_IL, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
    memset(resp_buf.buf + 1 + 64, 0xFF, 64);
  } else {
    FILL_RESP_MSG(CMD_QUERY_IL, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }

  FILL_RESP_MSG(CMD_QUERY_IL, RESPOND_SUCCESS, 1 + 64 * 2);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Tosa_Thr()
{
  uint16_t power;

  power = (int16_t)(tosa_power_high_max_thr * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf);
  power = (int16_t)(0 * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf + 2);
  power = (int16_t)(0 * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf + 4);
  power = (int16_t)(tosa_power_low_min_thr * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf + 6);
  FILL_RESP_MSG(CMD_QUERY_TOSA_THR, RESPOND_SUCCESS, 8);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Tosa()
{
  uint16_t power;
  
  power = (int16_t)(run_status.tosa_dst_power_high * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf);
  power = (int16_t)(run_status.tosa_dst_power_low * 100);
  BE16_To_Buffer((uint16_t)power, resp_buf.buf + 2);
  FILL_RESP_MSG(CMD_QUERY_TOSA, RESPOND_SUCCESS, 4);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_RX_PD_CALI(void)
{
  osStatus_t status = osOK;
  uint32_t i;

  resp_buf.buf[0] = 10;
  for (i = 0; i < 10; ++i) {
    status = RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_RX_PD + i * 8 + 4 + 2, resp_buf.buf + 1 + i * 4, 2);
    status |= RTOS_EEPROM_Read(EEPROM_ADDR, EE_CAL_RX_PD + i * 8 + 2, resp_buf.buf + 1 + i * 4 + 2, 2);
  }
  if (status != osOK) {
    EPT("Read EEPROM failed, status = %d\n", status);
    FILL_RESP_MSG(CMD_RX_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  FILL_RESP_MSG(CMD_RX_PD_CALI, RESPOND_SUCCESS, 41);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_TAP_PD_CALI(void)
{
  uint16_t adc;
  uint8_t ret = 0;

  resp_buf.buf[0] = 5;
  ret = cal_tap_pd_by_power(&adc, -15);
  if (ret) {
    FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  } else {
    BE16_To_Buffer((uint16_t)(int16_t)(-15 * 100), resp_buf.buf + 1);
    BE16_To_Buffer((uint16_t)(int16_t)adc, resp_buf.buf + 1 + 2);
  }

  ret = cal_tap_pd_by_power(&adc, -10);
  if (ret) {
    FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  } else {
    BE16_To_Buffer((uint16_t)(int16_t)(-10 * 100), resp_buf.buf + 1 + 4);
    BE16_To_Buffer((uint16_t)(int16_t)adc, resp_buf.buf + 1 + 6);
  }

  ret = cal_tap_pd_by_power(&adc, -5);
  if (ret) {
    FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  } else {
    BE16_To_Buffer((uint16_t)(int16_t)(-5 * 100), resp_buf.buf + 1 + 8);
    BE16_To_Buffer((uint16_t)(int16_t)adc, resp_buf.buf + 1 + 10);
  }

  ret = cal_tap_pd_by_power(&adc, 0);
  if (ret) {
    FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  } else {
    BE16_To_Buffer((uint16_t)(int16_t)(0 * 100), resp_buf.buf + 1 + 12);
    BE16_To_Buffer((uint16_t)(int16_t)adc, resp_buf.buf + 1 + 14);
  }

  ret = cal_tap_pd_by_power(&adc, 5);
  if (ret) {
    FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  } else {
    BE16_To_Buffer((uint16_t)(int16_t)(5 * 100), resp_buf.buf + 1 + 16);
    BE16_To_Buffer((uint16_t)(int16_t)adc, resp_buf.buf + 1 + 18);
  }

  FILL_RESP_MSG(CMD_TAP_PD_CALI, RESPOND_SUCCESS, 21);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Voltage()
{
  uint16_t value;
  double voltage;
  osStatus_t status;

  resp_buf.buf[CMD_SEQ_MSG_DATA] = 4;

  status = RTOS_ADC7828_Read(VOLTAGE_5_0_CHANNEL, &value);
  if (status != osOK) {
    EPT("Read adc7828 failed\n");
    value = 0;
  }
  voltage = (double)value / 4096 * 2.5 * 3;
  value = (uint16_t)(voltage * 10 + 0.5);
  BE16_To_Buffer(50, resp_buf.buf + 1);
  BE16_To_Buffer(value, resp_buf.buf + 3);

  status = RTOS_ADC7828_Read(VOLTAGE_3_3_CHANNEL, &value);
  if (status != osOK) {
    EPT("Read adc7828 failed\n");
    value = 0;
  }
  voltage = (double)value / 4096 * 2.5 * 2;
  value = (uint16_t)(voltage * 10 + 0.5);
  BE16_To_Buffer(33, resp_buf.buf + 5);
  BE16_To_Buffer(value, resp_buf.buf + 7);

  status = RTOS_ADC7828_Read(VOLTAGE_4_4_CHANNEL, &value);
  if (status != osOK) {
    EPT("Read adc7828 failed\n");
    value = 0;
  }
  voltage = (double)value / 4096 * 2.5 * 3;
  value = (uint16_t)(voltage * 10 + 0.5);
  BE16_To_Buffer(44, resp_buf.buf + 9);
  BE16_To_Buffer(value, resp_buf.buf + 11);

  status = RTOS_ADC7828_Read(VOLTAGE_61_0_CHANNEL, &value);
  if (status != osOK) {
    EPT("Read adc7828 failed\n");
    value = 0;
  }
  voltage = (double)value / 4096 * 2.5 * 51;
  value = (uint16_t)(voltage * 10 + 0.5);
  BE16_To_Buffer(640, resp_buf.buf + 13);
  BE16_To_Buffer(value, resp_buf.buf + 15);

  memset(resp_buf.buf + 17, 0xFF, 8);

  FILL_RESP_MSG(CMD_QUERY_VOLTAGE, RESPOND_SUCCESS, 25);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Alarm(void)
{
  uint32_t alarm_val = switch_endian(run_status.exp);

  BE32_To_Buffer(alarm_val, resp_buf.buf);
  FILL_RESP_MSG(CMD_QUERY_ALARM, RESPOND_SUCCESS, 4);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Set_Modulation(void)
{
  int8_t val = trans_buf.buf[CMD_SEQ_MSG_DATA];
  MsgStruct msg;
  
  if (val == 0) {
    if (run_status.modulation) {
      Clear_Lazer_Ready();
      HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_1);
      HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_3);
      run_status.modulation = 0;

      msg.pbuf = pvPortMalloc(8);
      msg.length = 8;
      msg.type = MSG_TYPE_LAZER_POWER;
      BE32_To_Buffer((uint32_t)(0xFFFFFFFF), (uint8_t*)msg.pbuf);
      BE32_To_Buffer((uint32_t)(0xFFFFFFFF), (uint8_t*)msg.pbuf + 4);
      osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
    }
  } else if (val == 1) {
    if (!run_status.modulation) {
      Clear_Lazer_Ready();
      HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
      run_status.modulation = 1;

      msg.pbuf = pvPortMalloc(8);
      msg.length = 8;
      msg.type = MSG_TYPE_LAZER_POWER;
      BE32_To_Buffer((uint32_t)(0xFFFFFFFF), (uint8_t*)msg.pbuf);
      BE32_To_Buffer((uint32_t)(0xFFFFFFFF), (uint8_t*)msg.pbuf + 4);
      osMessageQueuePut(mid_LazerManager, &msg, 0U, 0U);
    }
  } else {
    FILL_RESP_MSG(CMD_SET_MODULATION, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  FILL_RESP_MSG(CMD_SET_MODULATION, RESPOND_SUCCESS, 0);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Modulation(void)
{
  resp_buf.buf[0] = run_status.modulation;
  FILL_RESP_MSG(CMD_QUERY_MODULATION, RESPOND_SUCCESS, 1);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Status(void)
{
  resp_buf.buf[0] = device_busy;
  FILL_RESP_MSG(CMD_QUERY_STATUS, RESPOND_SUCCESS, 1);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Upgrade_Init()
{
  up_state.run = RUN_MODE_UPGRADE;
  up_state.block_size = 128;
  up_state.pre_state = UPGRADE_UNUSABLE;
  up_state.pre_seq = 0;
  up_state.recvd_length = 0;
  FILL_RESP_MSG(CMD_UPGRADE_MODE, RESPOND_SUCCESS, 0);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Upgrade_Data()
{
  uint8_t *p_fw_data = &trans_buf.buf[CMD_SEQ_MSG_DATA];
  uint32_t seq = switch_endian_16(*(uint16_t*)p_fw_data);
  uint32_t length = up_state.block_size;
  uint32_t to = 0;
  //EPT("seq = %u, length = %u\n", seq, length);
  p_fw_data += 2;
  
  if (trans_buf.buf[CMD_SEQ_MSG_LENGTH] != 0x86) {
    EPT("Upgrade data length invalid\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Upgrade data length invalid\n");
    FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }
  if (up_state.run != RUN_MODE_UPGRADE) {
    EPT("Cannot excute command because of wrong mode\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Cannot excute command because of wrong mode\n");
    FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  if (seq == 0x1) {
    if (strncmp((char*)&p_fw_data[FW_HEAD_MODULE_PN], pn, 10) || 
          strncmp((char*)&p_fw_data[FW_HEAD_MODULE_HW], hw_version, 3)) {
      EPT("The file is not the firmware corresponding to this module\n");
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "The file is not the firmware corresponding to this module\n");
      FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
    up_state.pre_seq = seq;
    up_state.recvd_length = 0;
    length = 0;
    
    // Ensure the flash for Application is empty or erasing when seq = 1
    if (!upgrade_status.flash_empty) {
      EPT("flash is not empty\n");
      // erase flash
      if (flash_in_use) {
        if (up_state.is_erasing) {
          EPT("Flash in using for upgrading...\n");
        } else {
          EPT("Flash in using for other functions...\n");
          THROW_LOG(MSG_TYPE_NORMAL_LOG, "Flash in using for other functions...\n");
          osDelay(pdMS_TO_TICKS(600));
          FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
          return RESPOND_FAILURE;
        }
        EPT("Flash in using\n");
      } else {
        flash_in_use = 1;
        up_state.is_erasing = 1;
        // erase flash
        if (up_state.upgrade_addr != RESERVE_ADDRESS)
          FLASH_If_Erase_IT(up_state.upgrade_sector);
        EPT("erase sector...\n");
      }
    }
  } else if (seq == 0x2) {
    if ((seq == up_state.pre_seq + 1 && up_state.pre_state == UPGRADE_SUCCESS) || \
      (seq == up_state.pre_seq && up_state.pre_state == UPGRADE_FAILURE)) {
      up_state.fw_size = (p_fw_data[FW_HEAD_FW_LENGTH + 0 - 0x80] << 24) | \
                        (p_fw_data[FW_HEAD_FW_LENGTH + 1 - 0x80] << 16) | \
                        (p_fw_data[FW_HEAD_FW_LENGTH + 2 - 0x80] <<  8 )| \
                        (p_fw_data[FW_HEAD_FW_LENGTH + 3 - 0x80] <<  0 );
        
      up_state.crc32 =  (p_fw_data[FW_HEAD_CRC + 0 - 0x80] << 24) | \
                        (p_fw_data[FW_HEAD_CRC + 1 - 0x80] << 16) | \
                        (p_fw_data[FW_HEAD_CRC + 2 - 0x80] <<  8 )| \
                        (p_fw_data[FW_HEAD_CRC + 3 - 0x80] <<  0 );
      EPT("Fw size = %u, crc = %#X\n", up_state.fw_size, up_state.crc32);
      if (up_state.fw_size > 0x18000) {
        EPT("File size(%#X) exceeds limit\n", up_state.fw_size);
        THROW_LOG(MSG_TYPE_NORMAL_LOG, "File size(%#X) exceeds limit\n", up_state.fw_size);
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
        return RESPOND_FAILURE;
      }
      up_state.pre_seq = seq;
      length = 0;
    } else {
      EPT("Seq invalid : %u\tpre_seq : %u\n", seq, up_state.pre_seq);
      THROW_LOG(MSG_TYPE_NORMAL_LOG, "Seq invalid : %u\tpre_seq : %u\n", seq, up_state.pre_seq);
      FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
  } else if ((seq == up_state.pre_seq + 1 && up_state.pre_state == UPGRADE_SUCCESS) || \
      (seq == up_state.pre_seq && up_state.pre_state == UPGRADE_FAILURE)) {
    up_state.pre_seq = seq;
    length = 128;
  } else {
    EPT("Seq invalid : %u\tpre_seq : %u\n", seq, up_state.pre_seq);
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Seq invalid : %u\tpre_seq : %u\n", seq, up_state.pre_seq);
    FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  while (flash_in_use && to++ < 6) {
    osDelay(pdMS_TO_TICKS(100));
  }
  if (to >= 6) {
    if (seq == 1) {
      up_state.pre_state = UPGRADE_SUCCESS;
      FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_SUCCESS, 0);
      return RESPOND_SUCCESS;
    }
    EPT("Waiting flash timeout\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Waiting flash timeout\n");
    up_state.pre_state = UPGRADE_FAILURE;
    FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 8);
    return RESPOND_FAILURE;
  }

  if (upgrade_status.flash_empty && length > 0) {
    upgrade_status.flash_empty = 0;
    if (Update_Up_Status(&upgrade_status) != osOK) {
      EPT("Update upgrade status to eeprom failed\n");
      up_state.pre_state = UPGRADE_FAILURE;
      FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_FAILURE, 0);
      return RESPOND_FAILURE;
    }
  }
  if (length > 0) {
    FLASH_If_Write(up_state.upgrade_addr + up_state.recvd_length, (uint32_t*)p_fw_data, length / 4);
    up_state.recvd_length += length;
  }
  up_state.pre_state = UPGRADE_SUCCESS;
  FILL_RESP_MSG(CMD_UPGRADE_DATA, RESPOND_SUCCESS, 0);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Upgrade_Install()
{
  uint8_t *p = resp_buf.buf;

  memset(p, 4, 0);

  if (up_state.run != RUN_MODE_UPGRADE) {
    EPT("Cannot excute cmd beacuse of wrong mode\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Cannot excute command because of wrong mode\n");
    FILL_RESP_MSG(CMD_UPGRADE_RUN, RESPOND_FAILURE, 0);
    return RESPOND_UNKNOWN_CMD;
  }

  if (up_state.recvd_length < up_state.fw_size) {
    EPT("The received length %u is less than the length in header %u.\n", up_state.recvd_length, up_state.fw_size);
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "The received length %u is less than the length in header %u.\n", up_state.recvd_length, up_state.fw_size);
    FILL_RESP_MSG(CMD_UPGRADE_RUN, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  if (up_state.pre_seq < 3) {
    EPT("No valid data.\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "No valid data.\n");
    FILL_RESP_MSG(CMD_UPGRADE_RUN, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  uint8_t *pdata = (uint8_t*)up_state.upgrade_addr;
  uint32_t crc = Cal_CRC32(pdata, up_state.fw_size);
  if (crc ^ up_state.crc32) {
    EPT("CRC verified failed. %#X != %#X\n", crc, up_state.crc32);
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "CRC verified failed. %#X != %#X\n", crc, up_state.crc32);
    FILL_RESP_MSG(CMD_UPGRADE_RUN, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  upgrade_status.flash_empty = 0;
  upgrade_status.run = up_state.upgrade_addr == APPLICATION_1_ADDRESS ? 1 : 2;
  upgrade_status.length = up_state.fw_size;
  upgrade_status.crc32 = up_state.crc32;
  EPT("upgrade_status: %#X, %u, %u, %u, %#X, %u, %#X\n", upgrade_status.magic, upgrade_status.run, upgrade_status.flash_empty, upgrade_status.length, upgrade_status.crc32,\
                upgrade_status.factory_length, upgrade_status.factory_crc32);
  if (Update_Up_Status(&upgrade_status) != osOK) {
    EPT("Update upgrade status to eeprom failed\n");
    THROW_LOG(MSG_TYPE_NORMAL_LOG, "Update upgrade status to eeprom failed\n");
    FILL_RESP_MSG(CMD_UPGRADE_RUN, RESPOND_FAILURE, 0);
    return RESPOND_FAILURE;
  }

  THROW_LOG(MSG_TYPE_NORMAL_LOG, "Upgrade firmware success\n");
  Uart_Respond(CMD_UPGRADE_RUN, RESPOND_SUCCESS, NULL, 0);
  __NVIC_SystemReset();
  while (1) {
  }
}

uint8_t Cmd_Softreset(void)
{
  // TODO: Save system configuration data
  run_status.uart_reset = 1;
  Uart_Respond(CMD_SOFTRESET, RESPOND_SUCCESS, NULL, 0);

  __NVIC_SystemReset();

  while (1) {
  }
}

uint8_t Cmd_Performance(void)
{
  uint8_t per_count = trans_buf.buf[CMD_SEQ_MSG_LENGTH] - 4;
  uint8_t i;

  if (per_count == 0 || per_count > 0x13) {
    FILL_RESP_MSG(CMD_QUERY_PERFORMANCE, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  for (i = 0; i < per_count; ++i) {
    resp_buf.buf[i * 5] = trans_buf.buf[CMD_SEQ_MSG_DATA + i];
    if (Get_Performance(trans_buf.buf[CMD_SEQ_MSG_DATA + i], resp_buf.buf + i * 5 + 1)) {
      FILL_RESP_MSG(CMD_QUERY_PERFORMANCE, RESPOND_INVALID_PARA, 0);
      return RESPOND_INVALID_PARA;
    }
  }

  FILL_RESP_MSG(CMD_QUERY_PERFORMANCE, RESPOND_SUCCESS, per_count * 5);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Set_Threshold(void)
{
  uint8_t alarm_id = trans_buf.buf[CMD_SEQ_MSG_DATA];
  int32_t val32_1, val32_2;
  
  val32_1 = (int32_t)Buffer_To_BE32(&trans_buf.buf[CMD_SEQ_MSG_DATA + 1]);
  val32_2 = (int32_t)Buffer_To_BE32(&trans_buf.buf[CMD_SEQ_MSG_DATA + 5]);
  if (val32_2 < val32_1) {
    FILL_RESP_MSG(CMD_SET_THRESHOLD, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  if (Set_Threshold(alarm_id, val32_1, val32_2)) {
    FILL_RESP_MSG(CMD_SET_THRESHOLD, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  FILL_RESP_MSG(CMD_SET_THRESHOLD, RESPOND_SUCCESS, 0);
  return RESPOND_SUCCESS;
}

uint8_t Cmd_Query_Threshold(void)
{
  uint8_t alarm_id = trans_buf.buf[CMD_SEQ_MSG_DATA];
  
  resp_buf.buf[0] = alarm_id;
  if (Get_Threshold(alarm_id, resp_buf.buf + 1)) {
    FILL_RESP_MSG(CMD_QUERY_THRESHOLD, RESPOND_INVALID_PARA, 0);
    return RESPOND_INVALID_PARA;
  }
  FILL_RESP_MSG(CMD_QUERY_THRESHOLD, RESPOND_SUCCESS, 9);
  return RESPOND_SUCCESS;
}

#if 0
uint32_t Cmd_LOG_Content()
{
  uint8_t *prdata = trans_buf.buf + CMD_SEQ_MSG_DATA;
  uint32_t packet_count, packet_num;
  uint32_t addr;
  int32_t size;

  size = log_file_state.offset - log_file_state.header;
  if (log_file_state.offset < log_file_state.header) {
    size += (file_flash_end + 1) - file_flash_addr[0];
  }
  packet_count = size / LOG_PACKET_LENGTH;
  if (size)
    packet_count += 1;

  //packet_count = Buffer_To_BE32(prdata);
  if (packet_count != Buffer_To_BE32(prdata)) {
    EPT("Wrong number of packages\n");
    THROW_LOG("Wrong number of packages\n");
    BE32_To_Buffer((uint32_t)packet_count, resp_buf.buf);
    BE32_To_Buffer((uint32_t)0, resp_buf.buf + 4);
    BE32_To_Buffer((uint32_t)0, resp_buf.buf + 8);
    size = 0;
    FILL_RESP_MSG(CMD_QUERY_LOG, RESPOND_SUCCESS, 12 + size);
    return RESPOND_SUCCESS;
  }
  packet_num = Buffer_To_BE32(prdata + 4);
  if (packet_num > packet_count || packet_num == 0) {
    EPT("packet number = %u is invalid\n", packet_num);
    THROW_LOG("packet number = %u is invalid\n", packet_num);
    BE32_To_Buffer((uint32_t)packet_count, resp_buf.buf);
    BE32_To_Buffer((uint32_t)0, resp_buf.buf + 4);
    BE32_To_Buffer((uint32_t)0, resp_buf.buf + 8);
    size = 0;
    FILL_RESP_MSG(CMD_QUERY_LOG, RESPOND_SUCCESS, 12 + size);
    return RESPOND_SUCCESS;
  }
  addr = log_file_state.header + (packet_num - 1) * LOG_PACKET_LENGTH;
  if (addr >= file_flash_end + 1) {
    addr = addr - (file_flash_end + 1) + file_flash_addr[0];
  }
  if (packet_count == packet_num) {
    size = log_file_state.offset - addr;
    if (size > LOG_PACKET_LENGTH)
      size = LOG_PACKET_LENGTH;
  } else {
    size = LOG_PACKET_LENGTH;
  }
  if (flash_in_use) {
    THROW_LOG("Reading log when flash in using\n");
    size = 0;
  }
  EPT("packet_num = %u, dest addr = %#X, size = %u\n", packet_num, addr, size);
  BE32_To_Buffer((uint32_t)packet_count, resp_buf.buf);
  BE32_To_Buffer((uint32_t)packet_num, resp_buf.buf + 4);
  BE32_To_Buffer((uint32_t)size, resp_buf.buf + 8);
  Log_Read(addr, resp_buf.buf + 12, size);
  FILL_RESP_MSG(CMD_QUERY_LOG, RESPOND_SUCCESS, 12 + size);
  return RESPOND_SUCCESS;
}
#endif
uint8_t Cmd_For_Debug()
{
  uint8_t *prdata = trans_buf.buf + CMD_SEQ_MSG_DATA;
  uint32_t sw_num, ret, u_val, u_val2;
  int32_t val_x, val_y, val;

  uint32_t temp = Buffer_To_BE32(prdata);
  memset(resp_buf.buf, 0, 4);
  if (temp != 0x5A5AA5A5) {
    FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_UNKNOWN_CMD, 4);
    return RESPOND_UNKNOWN_CMD;
  }
  
  temp = Buffer_To_BE32(prdata + 4);
  if (temp == CMD_DEBUG_SW_DAC) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    val_x = (int32_t)Buffer_To_BE32(prdata + 12);
    val_y = (int32_t)Buffer_To_BE32(prdata + 16);
    ret = debug_sw_dac(sw_num, val_x, val_y);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_SW_ADC) {
    memset(resp_buf.buf, 0, 8);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_sw_adc(sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 8);
    return ret;
  } else if (temp == CMD_DEBUG_VOL_ADC) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_vol_adc(sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_TAG) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = Buffer_To_BE32(prdata + 12);
    ret = debug_tag(sw_num, prdata + 16, ret);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    return ret;
  } else if (temp == CMD_DEBUG_PIN) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = Buffer_To_BE32(prdata + 12);
    ret = debug_pin(sw_num, ret);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_CAL_SW) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    val = Buffer_To_BE32(prdata + 12);
    val_x = (int32_t)Buffer_To_BE32(prdata + 16);
    val_y = (int32_t)Buffer_To_BE32(prdata + 20);
    ret = debug_cal_switch(sw_num, val, val_x, val_y);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_CAL_TOSA) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = Buffer_To_BE32(prdata + 12);
    u_val = Buffer_To_BE32(prdata + 16);
    u_val2 = Buffer_To_BE32(prdata + 20);
    val = (int32_t)Buffer_To_BE32(prdata + 24);
    ret = debug_cal_tosa(sw_num, ret, u_val, u_val2, val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_GET_TOSA_VAL) {
    memset(resp_buf.buf, 0, 4);
    val = (int32_t)Buffer_To_BE32(prdata + 8);
    ret = debug_get_tosa_val(val, &u_val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, u_val);
    return ret;
  } else if (temp == CMD_DEBUG_CAL_IL) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    val = (int32_t)Buffer_To_BE32(prdata + 12);
    ret = debug_cal_il(sw_num, val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_CAL_DEF_TEMP) {
    memset(resp_buf.buf, 0, 4);
    val = (int32_t)Buffer_To_BE32(prdata + 8);
    ret = debug_cal_default_temp(val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_CAL_RX_PD) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = Buffer_To_BE32(prdata + 12);
    val = (int32_t)Buffer_To_BE32(prdata + 16);
    ret = debug_cal_rx_pd(sw_num, ret, val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_DUMP) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_cal_dump(sw_num, &u_val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, u_val);
    return ret;
  } else if (temp == CMD_DEBUG_EEPROM) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    u_val = Buffer_To_BE32(prdata + 12);
    ret = debug_eeprom(sw_num, &u_val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, u_val);
    return ret;
  } else if (temp == CMD_DEBUG_RESET_LOG) {
    memset(resp_buf.buf, 0, 4);
    if (Reset_Log_Status() == osOK) {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_SUCCESS, 4);
      return RESPOND_SUCCESS;
    }
    else {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_FAILURE, 4);
      return RESPOND_FAILURE;
    }
  } else if (temp == CMD_DEBUG_RESET_FW) {
    memset(resp_buf.buf, 0, 4);
    if (Reset_Up_Status() == osOK) {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_SUCCESS, 4);
      return RESPOND_SUCCESS;
    }
    else {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_FAILURE, 4);
      return RESPOND_FAILURE;
    }
  } else if (temp == CMD_DEBUG_TEST_TOSA) {
    memset(resp_buf.buf, 0, 4);
    ret = debug_test_tosa();
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    return ret;
  } else if (temp == CMD_DEBUG_SET_TOSA) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    u_val = Buffer_To_BE32(prdata + 12);
    ret = debug_set_tosa((uint16_t)sw_num, (uint16_t)u_val);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    return ret;
  } else if (temp == CMD_DEBUG_SET_TEC) {
    memset(resp_buf.buf, 0, 4);
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_set_tec((uint16_t)sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    return ret;
  } else if (temp == CMD_DEBUG_GET_TOSA_TMP) {
    memset(resp_buf.buf, 0, 40);
    ret = debug_get_tosa_tmp();
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 48);
    return ret;
  } else if (temp == CMD_DEBUG_GET_PD) {
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_get_pd(sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 8);
    return ret;
  } else if (temp == CMD_DEBUG_SET_LP) {
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_set_lp(sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    return ret;
  } else if (temp == CMD_DEBUG_GET_SW_CHAN) {
    sw_num = Buffer_To_BE32(prdata + 8);
    ret = debug_get_switch_channel(sw_num);
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 2);
    return ret;
  } else if (temp == CMD_DEBUG_RESET_ALARM) {
    memset(resp_buf.buf, 0, 4);
    if (Reset_EEPROM_Alarm_Status() == osOK) {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_SUCCESS, 4);
      return RESPOND_SUCCESS;
    }
    else {
      FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_FAILURE, 4);
      return RESPOND_FAILURE;
    }
  } else if (temp == CMD_DEBUG_INTER_EXP) {
    ret = debug_get_inter_exp();
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  } else if (temp == CMD_DEBUG_SET_SW_ADC) {
    u_val = Buffer_To_BE32(prdata + 8);
    run_status.sw_adc_int = (uint16_t)u_val;
    u_val = Buffer_To_BE32(prdata + 12);
    run_status.sw_adc_double = (double)u_val / 1000;
    FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_SUCCESS, 0);
    return RESPOND_SUCCESS;
  } else if (temp == CMD_DEBUG_DATE) {
    sw_num = Buffer_To_BE32(prdata + 8);
    if (sw_num == 0) {
      ret = debug_Cmd_Set_Time(prdata + 12);
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    } else if (sw_num == 1) {
      ret = debug_Cmd_Get_Time();
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 6);
    } else {
      ret = RESPOND_FAILURE;
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    }
    return ret;
  } else if (temp == CMD_DEBUG_LOG) {
    sw_num = Buffer_To_BE32(prdata + 8);
    if (sw_num == 0) {
      ret = debug_Cmd_LOG_Size(prdata + 12);
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 5);
    } else if (sw_num == 1) {
      ret = debug_Cmd_LOG_Content(prdata + 12, &u_val);
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, u_val);
    } else {
      ret = RESPOND_FAILURE;
      FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 0);
    }
    return ret;
  } else if (temp == CMD_DEBUG_HISTORY_ALARM) {
    ret = debug_Cmd_Query_Alarm_History();
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 80);
    return ret;
  } else if (temp == CMD_DEBUG_CHECK_CALI) {
    ret = debug_Cmd_Check_Cali();
    FILL_RESP_MSG(CMD_FOR_DEBUG, ret, 4);
    return ret;
  }

  FILL_RESP_MSG(CMD_FOR_DEBUG, RESPOND_UNKNOWN_CMD, 4);
  return RESPOND_UNKNOWN_CMD;
}


