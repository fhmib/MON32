#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmd.h"
#include "function.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "xmodem.h"
#include "flash_if.h"

uint8_t fw_buf[96 * 1024];
uint8_t rBuf[TRANS_MAX_LENGTH];
uint8_t rLen;
uint8_t txBuf[TRANS_MAX_LENGTH];

#define FW_BLOCK_MAX_SIZE 128
#define TO_STR(s) #s
#if 0
#define GET_RESULT()  do {\
                        if (ret >= 0) {\
                          if (rBuf[rLen - 2] != 0) {\
                            PRINT(CMD_FAILED, rBuf[rLen - 2]);\
                          } else {\
                            PRINT(CMD_SUCCESS);\
                          }\
                        } else {\
                          PRINT(CMD_FAILED, ret);\
                        }\
                      } while(0)
#endif

uint32_t block_size = FW_BLOCK_MAX_SIZE;


int8_t cmd_power(uint8_t argc, char **argv)
{
  if (!strcasecmp(argv[0], "poweron")) {
    HAL_GPIO_WritePin(OUT_VOL_5_0_GPIO_Port, OUT_VOL_5_0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUT_VOL_3_3_GPIO_Port, OUT_VOL_3_3_Pin, GPIO_PIN_SET);
  } else if (!strcasecmp(argv[0], "poweroff")) {
    HAL_GPIO_WritePin(OUT_VOL_5_0_GPIO_Port, OUT_VOL_5_0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUT_VOL_3_3_GPIO_Port, OUT_VOL_3_3_Pin, GPIO_PIN_RESET);
  } else {
    cmd_help2(argv[0]);
  }
  
  return 0;
}

int8_t cmd_get_sn(uint8_t argc, char **argv)
{
  int8_t ret;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_SN, NULL, 0, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  rBuf[CMD_SEQ_MSG_DATA + 10] = 0;
  PRINT("Serial Number: %s\r\n", (char*)&rBuf[CMD_SEQ_MSG_DATA]);

  return ret;
}

int8_t cmd_get_date(uint8_t argc, char **argv)
{
  int8_t ret;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_MDATE, NULL, 0, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  rBuf[CMD_SEQ_MSG_DATA + 9] = 0;
  PRINT("Menufacture Date: %s\r\n", (char*)&rBuf[CMD_SEQ_MSG_DATA]);

  return ret;
}

int8_t cmd_get_pn(uint8_t argc, char **argv)
{
  int8_t ret;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_PN, NULL, 0, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  rBuf[CMD_SEQ_MSG_DATA + 17] = 0;
  PRINT("Part Number: %s\r\n", (char*)&rBuf[CMD_SEQ_MSG_DATA]);

  return ret;
}

int8_t cmd_get_vendor(uint8_t argc, char **argv)
{
  int8_t ret;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_VENDOR, NULL, 0, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  rBuf[CMD_SEQ_MSG_DATA + 17] = 0;
  PRINT("Vendor: %s\r\n", (char*)&rBuf[CMD_SEQ_MSG_DATA]);

  return ret;
}

int8_t cmd_version(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t *p;
  uint32_t i = 5000;

  if (argc == 2 && !strcasecmp(argv[1], "repeat")) {
    while (i--) {
      ret = process_command(CMD_QUERY_VERSION, txBuf, 0, rBuf, &rLen);
      if (ret) {
        PRINT("i = %u\r\n", i);
        return ret;
      }
    }
  } else if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_VERSION, txBuf, 0, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  //PRINT_HEX("Received", rBuf, rLen);

  p = rBuf + CMD_SEQ_MSG_DATA;
  p[8] = 0;
  PRINT("Firmware Version: %s\r\n", (char*)p);

  return ret;
}

int8_t cmd_switch(uint8_t argc, char **argv)
{
  if (argc >= 3 && !strcasecmp(argv[1], "write")) {
    return set_switch(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "read")) {
    return get_switch(argc, argv);
  } else {
    cmd_help2(argv[0]);
  }
  return 0;
}

int8_t set_switch(uint8_t argc, char **argv)
{
  uint8_t switch_channel, switch_pos, which;
  uint8_t ret;
  
  if (argc == 3 && !strcasecmp(argv[0], "rxsw")) {
    switch_channel = 1;
    switch_pos = strtoul(argv[2], NULL, 10);
    switch_pos -= 1;
  } else if (argc == 5 && !strcasecmp(argv[0], "txsw") && !strcasecmp(argv[3], "to")) {
    switch_channel = 0;
    which = strtoul(argv[2], NULL, 10);
    switch_pos = strtoul(argv[4], NULL, 10);
    switch_pos -= 1;
    if (which == 1) {
    } else if (which == 2) {
      switch_pos += 0x20;
    } else {
      cmd_help2(argv[0]);
      return 0;
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  txBuf[0] = switch_channel;
  txBuf[1] = switch_pos;
  ret = process_command(CMD_SET_SWITCH, txBuf, 2, rBuf, &rLen);

  return ret;
}

int8_t get_switch(uint8_t argc, char **argv)
{
  int8_t ret;

  if (!strcasecmp(argv[0], "rxsw")) {
    txBuf[0] = 1;
  } else if (!strcasecmp(argv[0], "txsw")) {
    txBuf[0] = 0;
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_SWITCH, txBuf, 1, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  if (!strcasecmp(argv[0], "rxsw")) {
    PRINT("RX switch channel is %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] + 1);
  } else if (!strcasecmp(argv[0], "txsw")) {
    if (rBuf[CMD_SEQ_MSG_DATA + 1] == 0xFF) {
      PRINT("TX switch channel is blocked\r\n");
    } else if (rBuf[CMD_SEQ_MSG_DATA + 1] < 0x20) {
      PRINT("TX switch channel is 1 to %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] + 1);
    } else {
      PRINT("TX switch channel is 2 to %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] - 0x20 + 1);
    }
  }

  return ret;
}

int8_t cmd_temp(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t *p;
  int16_t temp;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_TEMP, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  p = rBuf + CMD_SEQ_MSG_DATA;
  temp = (int16_t)switch_endian_16(*(uint16_t*)p);
  PRINT("Temperature: %.1lfC\r\n", (double)temp/10);

  return ret;
}

int8_t cmd_IL(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t i;
  int16_t val;
  double val_f;

  if (argc == 2 && !strcasecmp(argv[1], "0")) {
    txBuf[0] = 0;
  } else if (argc == 2 && !strcasecmp(argv[1], "1")) {
    txBuf[0] = 1;
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_IL, txBuf, 1, rBuf, &rLen);
  
  if (ret) {
    return ret;
  }

  PRINT("Insertion Loss %u:\r\n", rBuf[CMD_SEQ_MSG_DATA]);
  for (i = 0; i < 64; ++i) {
    val = (int16_t)Buffer_To_BE16(&rBuf[CMD_SEQ_MSG_DATA + 1 + i * 2]);
    val_f = (double)val / 100;
    PRINT("%u,%.2lf\r\n", i + 1, val_f);
  }

  return ret;
}

int8_t cmd_tosa(uint8_t argc, char **argv)
{
  if (argc == 2 && !strcasecmp(argv[1], "get")) {
    get_tosa(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "thr")) {
    get_tosa_thr(argc, argv);
  } else {
    cmd_help2(argv[0]);
    return 0;
  }
  
  return 0;
}

int8_t get_tosa(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t *p;
  int16_t temp;

  ret = process_command(CMD_QUERY_TOSA, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  p = rBuf + CMD_SEQ_MSG_DATA;
  temp = (int16_t)switch_endian_16(*(uint16_t*)p);
  PRINT("Power High: %.1lfdBm\r\n", (double)temp/100);
  temp = (int16_t)switch_endian_16(*(uint16_t*)(p + 2));
  PRINT("Power Low: %.1lfdBm\r\n", (double)temp/100);

  return ret;
}

int8_t get_tosa_thr(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t *p;
  int16_t temp;

  ret = process_command(CMD_QUERY_TOSA_THR, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  p = rBuf + CMD_SEQ_MSG_DATA;
  temp = (int16_t)switch_endian_16(*(uint16_t*)p);
  PRINT("Power High Max Threshold: %.2lfdBm\r\n", (double)temp/100);
  temp = (int16_t)switch_endian_16(*(uint16_t*)(p + 2));
  PRINT("Power High Min Threshold: %.2lfdBm\r\n", (double)temp/100);
  temp = (int16_t)switch_endian_16(*(uint16_t*)(p + 4));
  PRINT("Power Low Max Threshold: %.2lfdBm\r\n", (double)temp/100);
  temp = (int16_t)switch_endian_16(*(uint16_t*)(p + 6));
  PRINT("Power Low Min Threshold: %.2lfdBm\r\n", (double)temp/100);

  return ret;
}

int8_t cmd_rx_pd_cali(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t i;
  int16_t val;
  uint16_t adc;
  double val_f;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_RX_PD_CALI, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  PRINT("RX PD Cali Data Count: %u\r\n", rBuf[CMD_SEQ_MSG_DATA]);
  for (i = 0; i < rBuf[CMD_SEQ_MSG_DATA]; ++i) {
    val = (int16_t)Buffer_To_BE16(&rBuf[CMD_SEQ_MSG_DATA + 1 + i * 4]);
    val_f = (double)val / 100;
    PRINT("%u,%.2lf, ", i + 1, val_f);
    adc = (uint16_t)Buffer_To_BE16(&rBuf[CMD_SEQ_MSG_DATA + 1 + i * 4 + 2]);
    PRINT("%u\r\n", adc);
  }

  return ret;
}

int8_t cmd_tap_pd_cali(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t i;
  int16_t val;
  uint16_t adc;
  double val_f;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_TAP_PD_CALI, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  PRINT("TAP PD Cali Data Count: %u\r\n", rBuf[CMD_SEQ_MSG_DATA]);
  for (i = 0; i < rBuf[CMD_SEQ_MSG_DATA]; ++i) {
    val = (int16_t)Buffer_To_BE16(&rBuf[CMD_SEQ_MSG_DATA + 1 + i * 4]);
    val_f = (double)val / 100;
    PRINT("%u,%.2lf, ", i + 1, val_f);
    adc = (uint16_t)Buffer_To_BE16(&rBuf[CMD_SEQ_MSG_DATA + 1 + i * 4 + 2]);
    PRINT("%u\r\n", adc);
  }

  return ret;
}

int8_t cmd_voltage(uint8_t argc, char **argv)
{
  if (argc == 2 && !strcasecmp(argv[1], "get")) {
    return get_voltage();
  } else {
    cmd_help2(argv[0]);
  }
  return 0;
}

int8_t get_voltage()
{
  int8_t ret;
  uint8_t *p = rBuf + CMD_SEQ_MSG_DATA;
  int16_t i, value;
  double voltage;

  ret = process_command(CMD_QUERY_VOLTAGE, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  PRINT("Valid voltage count : %u\r\n", *p);
  p += 1;
  for (i = 0; i < 6; ++i) {
    value = (int16_t)Buffer_To_BE16(p);
    voltage = (double)value / 10;
    PRINT("Voltage target Value : %.1lfV\r\n", voltage);
    value = (int16_t)Buffer_To_BE16(p + 2);
    voltage = (double)value / 10;
    PRINT("Voltage Current Value : %.1lfV\r\n", voltage);
    p += 4;
  }

  return ret;
}

int8_t cmd_alarm(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t exp;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_ALARM, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  exp = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA);
  PRINT("Alarm: %#X\r\n", exp);

  return ret;
}

int8_t cmd_modulation(uint8_t argc, char **argv)
{
  int8_t ret;

  if (argc == 2 && !strcasecmp(argv[1], "on")) {
    txBuf[0] = 1;
    ret = process_command(CMD_SET_MODULATION, txBuf, 1, rBuf, &rLen);
    if (ret) {
      return ret;
    }
  } else if (argc == 2 && !strcasecmp(argv[1], "off")) {
    txBuf[0] = 0;
    ret = process_command(CMD_SET_MODULATION, txBuf, 1, rBuf, &rLen);
    if (ret) {
      return ret;
    }
  } else if (argc == 2 && !strcasecmp(argv[1], "mode")) {
    ret = process_command(CMD_QUERY_MODULATION, txBuf, 0, rBuf, &rLen);
    if (ret) {
      return ret;
    }
    
    if (rBuf[CMD_SEQ_MSG_DATA] == 0) {
      PRINT("Module in Non-modulation mode\r\n");
    } else if (rBuf[CMD_SEQ_MSG_DATA] == 1) {
      PRINT("Module in modulation mode\r\n");
    } else {
      PRINT("Invalid value returned from module\r\n");
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }


  return 0;
}

int8_t cmd_device_status(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t status;

  if (argc > 1) {
    cmd_help2(argv[0]);
    return 0;
  }

  ret = process_command(CMD_QUERY_STATUS, txBuf, 0, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  status = rBuf[CMD_SEQ_MSG_DATA];
  if (status == 0) {
    PRINT("Device is idle\r\n");
  } else if (status == 1) {
    PRINT("Device is busy\r\n");
  } else {
    PRINT("Unknown device status code\r\n");
  }

  return ret;
}

int8_t cmd_upgrade(uint8_t argc, char **argv)
{
  if (argc == 2 && !strcasecmp(argv[1], "init")) {
    return upgrade_init();
  } else if (argc == 2 && !strcasecmp(argv[1], "file")) {
    return upgrade_file(1);
  } else if (argc == 3 && !strcasecmp(argv[1], "file") && !strcasecmp(argv[2], "no_verify")) {
    return upgrade_file(0);
  } else if (argc == 3 && !strcasecmp(argv[1], "file") && !strcasecmp(argv[2], "xmodem")) {
    return upgrade_file_xmodem();
  } else if (argc == 2 && !strcasecmp(argv[1], "run")) {
    return upgrade_install();
  } else {
    cmd_help2(argv[0]);
  }
  return 0;
}

int8_t upgrade_init()
{
  block_size = FW_BLOCK_MAX_SIZE;
  PRINT("Initialize Upgrade\r\n");
  txBuf[0] = 1;
  return process_command(CMD_UPGRADE_MODE, txBuf, 1, rBuf, &rLen);
}

int8_t upgrade_file(uint8_t verify)
{
  int8_t ret;
  uint32_t fw_crc;
  uint16_t seq = 1;
  uint32_t fw_length, every_size, send_size = 0;
  uint8_t retry = 0;

  HAL_Delay(1);
  __HAL_UART_DISABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
  CLEAR_BIT(TERMINAL_UART.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&TERMINAL_UART);
  uart1_irq_sel = 0;
  PRINT2("Download image...\r\n");

  if (HAL_UART_Receive(&TERMINAL_UART, fw_buf, 256, 1000 * 60) != HAL_OK) {
    PRINT2("UART Timeout\r\n");
    __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
    uart1_irq_sel = 1;
    return 1;
  }
  fw_length = (fw_buf[FW_HEAD_FW_LENGTH] << 24) | (fw_buf[FW_HEAD_FW_LENGTH + 1] << 16) |\
           (fw_buf[FW_HEAD_FW_LENGTH + 2] << 8) | (fw_buf[FW_HEAD_FW_LENGTH + 3] << 0);
  fw_crc = (fw_buf[FW_HEAD_CRC] << 24) | (fw_buf[FW_HEAD_CRC + 1] << 16) |\
           (fw_buf[FW_HEAD_CRC + 2] << 8) | (fw_buf[FW_HEAD_CRC + 3] << 0);
  if (fw_length > 0xFFFF) {
    every_size = 0xFF00;
    while (send_size < fw_length) {
      if (send_size + every_size > fw_length) {
        HAL_UART_Receive(&TERMINAL_UART, fw_buf + 256 + send_size, fw_length - send_size, 1000 * 10);
        send_size += fw_length - send_size;
      } else {
        HAL_UART_Receive(&TERMINAL_UART, fw_buf + 256 + send_size, every_size, 1000 * 10);
        send_size += every_size;
      }
    }
  } else
    HAL_UART_Receive(&TERMINAL_UART, fw_buf + 256, fw_length, 1000 * 10);
  PRINT2("Download success, Length = %u, crc = %#X\r\n", fw_length, fw_crc);
  if (verify) {
    if (Cal_CRC32(&fw_buf[256], fw_length) == fw_crc) {
      PRINT2("CRC32 success\r\n");
    } else {
      PRINT2("CRC32 failed\r\n");
      __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
      uart1_irq_sel = 1;
      return 2;
    }
  } else {
    PRINT2("Skip CRC32\r\n");
  }

  PRINT2("Sending image...\r\n");
  every_size = FW_BLOCK_MAX_SIZE;
  send_size = 0;
  while (send_size < fw_length + FW_FILE_HEADER_LENGTH) {
    *(uint16_t*)(&txBuf[0]) = switch_endian_16(seq);

    if (send_size + every_size > fw_length + FW_FILE_HEADER_LENGTH) {
      memset(&txBuf[2], 0, every_size);
      memcpy(&txBuf[2], &fw_buf[send_size], fw_length + FW_FILE_HEADER_LENGTH - send_size);
    } else {
      memcpy(&txBuf[2], &fw_buf[send_size], every_size);
    }
    ret = process_command(CMD_UPGRADE_DATA, txBuf, 2 + every_size, rBuf, &rLen);
    if (ret) {
      if (retry < 3) {
        PRINT2("Retry, seq = %u\r\n", seq);
        retry++;
        continue;
      }
      break;
    }
    seq++;
    send_size += every_size;
  }

  if (send_size >= fw_length + FW_FILE_HEADER_LENGTH)
    PRINT2("Send fw success\r\n");
  else
    PRINT2("Send fw failed\r\n");

  __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
  uart1_irq_sel = 1;

  return ret;
}

int8_t upgrade_file_xmodem(void)
{
  uint8_t ret;
  uint32_t fw_length, every_size = FW_BLOCK_MAX_SIZE, send_size = 0;
  uint16_t seq = 1;
  uint8_t retry = 0;

  PRINT("Erasing falsh");
  HAL_Delay(10);
  FLASH_If_Erase(FLASH_SECTOR_17);
  PRINT(".");
  HAL_Delay(10);
  FLASH_If_Erase(FLASH_SECTOR_18);
  PRINT(".");
  HAL_Delay(10);
  FLASH_If_Erase(FLASH_SECTOR_19);
  PRINT("Done.\r\n");
  PRINT("Download image...\r\n");
  ret = xmodem_receive(&fw_length);
  if (ret) {
    PRINT("xmodem failed, ret = %u\r\n", ret);
    return 0;
  }
  PRINT("Download success, Size = %u(%#X))\r\n", fw_length, fw_length);
  PRINT("\r\nSending image...\r\n");
  while (send_size < fw_length) {
    *(uint16_t*)(&txBuf[0]) = switch_endian_16(seq);

    if (send_size + every_size > fw_length) {
      memset(&txBuf[2], 0, every_size);
      memcpy(&txBuf[2], (uint8_t*)(IMAGE_ADDRESS + send_size), fw_length - send_size);
    } else {
      memcpy(&txBuf[2], (uint8_t*)(IMAGE_ADDRESS + send_size), every_size);
    }
    ret = process_command(CMD_UPGRADE_DATA, txBuf, 2 + every_size, rBuf, &rLen);
    if (ret) {
      if (retry < 3) {
        PRINT2("Retry, seq = %u\r\n", seq);
        retry++;
        continue;
      }
      break;
    }
    seq++;
    send_size += every_size;
  }

  return 0;
}

int8_t upgrade_install()
{
  return process_command(CMD_UPGRADE_RUN, txBuf, 0, rBuf, &rLen);
}

int8_t cmd_reset(uint8_t argc, char **argv)
{
  int8_t ret = 0;

  if (argc == 2 && !strcasecmp(argv[1], "hard")) {
    HAL_GPIO_WritePin(HARD_RESET_GPIO_Port, HARD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(HARD_RESET_GPIO_Port, HARD_RESET_Pin, GPIO_PIN_SET);
  } else if (argc == 2 && !strcasecmp(argv[1], "master")) {
    HAL_GPIO_WritePin(MASTER_RESET_GPIO_Port, MASTER_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MASTER_RESET_GPIO_Port, MASTER_RESET_Pin, GPIO_PIN_SET);
  } else if (argc == 2 && !strcasecmp(argv[1], "soft")) {
    ret = process_command(CMD_SOFTRESET, txBuf, 0, rBuf, &rLen);
    if (ret) {
      return ret;
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  return ret;
}

int8_t cmd_performance(uint8_t argc, char **argv)
{
  int8_t ret, i;
  uint8_t *p = rBuf + CMD_SEQ_MSG_DATA;
  uint32_t u_val32;
  int32_t val32;
  double d_val;

  if (argc == 2 && !strcasecmp(argv[1], "all")) {
    for (i = 0; i <= 0x12; ++i) {
      txBuf[i] = i;
    }
    ret = process_command(CMD_QUERY_PERFORMANCE, txBuf, 0x12 + 1, rBuf, &rLen);
  } else {
    for (i = 0; i < argc - 1; ++i) {
      txBuf[i] = (uint8_t)strtoul(argv[i + 1], NULL, 0);
    }
    ret = process_command(CMD_QUERY_PERFORMANCE, txBuf, argc - 1, rBuf, &rLen);
  }
  if (ret) {
    return ret;
  }

  while (i--) {
    switch (*p) {
      case 0:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        PRINT("LD_Currnet : %d(%#X)mA\r\n", val32, val32);
        break;
      case 1:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 10;
        PRINT("LD_Temperature : %.1lf(%#X)C\r\n", d_val, val32);
        break;
      case 2:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        PRINT("TEC_Current : %d(%#X)mA\r\n", val32, val32);
        break;
      case 3:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        PRINT("TEC_Voltage : %d(%#X)mV\r\n", val32, val32);
        break;
      case 4:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        PRINT("LD_BACKLIGHT_VOL : %d(%#X)mV\r\n", val32, val32);
        break;
      case 5:
        u_val32 = Buffer_To_BE32(p + 1);
        PRINT("LD_LOCK_STATE : %d(%#X)\r\n", u_val32, u_val32);
        break;
      case 6:
        u_val32 = Buffer_To_BE32(p + 1);
        PRINT("TEC_LOCK_STATE : %d(%#X)\r\n", u_val32, u_val32);
        break;
      case 7:
        u_val32 = Buffer_To_BE32(p + 1);
        PRINT("LD_ON_OFF_STATE : %d(%#X)\r\n", u_val32, u_val32);
        break;
      case 8:
        u_val32 = Buffer_To_BE32(p + 1);
        PRINT("LD_Modulation_Mode : %d(%#X)\r\n", u_val32, u_val32);
        break;
      case 9:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("TAP PD Power : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      case 0xA:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("Rev Pd Power : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      case 0xB:
        u_val32 = (uint32_t)Buffer_To_BE16(p + 1);
        PRINT("2x32 X1 : %u(%#X)\r\n", u_val32, u_val32);
        u_val32 = (uint32_t)Buffer_To_BE16(p + 3);
        PRINT("2x32 X2 : %u(%#X)\r\n", u_val32, u_val32);
        break;
      case 0xC:
        u_val32 = (uint32_t)Buffer_To_BE16(p + 1);
        PRINT("2x32 Y1 : %u(%#X)\r\n", u_val32, u_val32);
        u_val32 = (uint32_t)Buffer_To_BE16(p + 3);
        PRINT("2x32 Y2 : %u(%#X)\r\n", u_val32, u_val32);
        break;
      case 0xD:
        u_val32 = (uint32_t)Buffer_To_BE16(p + 1);
        PRINT("1x32 X1 : %u(%#X)\r\n", u_val32, u_val32);
        u_val32 = (uint32_t)Buffer_To_BE16(p + 3);
        PRINT("1x32 X2 : %u(%#X)\r\n", u_val32, u_val32);
        break;
      case 0xE:
        u_val32 = (uint32_t)Buffer_To_BE16(p + 1);
        PRINT("1x32 Y1 : %u(%#X)\r\n", u_val32, u_val32);
        u_val32 = (uint32_t)Buffer_To_BE16(p + 3);
        PRINT("1x32 Y2 : %u(%#X)\r\n", u_val32, u_val32);
        break;
      case 0xF:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("TAP PD Power + 2x32 chan IL : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      case 0x10:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("Rev Pd Power + 1x32 chan IL : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      case 0x11:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("2x32 Current chan IL : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      case 0x12:
        val32 = (int32_t)Buffer_To_BE32(p + 1);
        d_val = (double)val32 / 100;
        PRINT("1x32 Current chan IL : %.2lf(%#X)dBm\r\n", d_val, val32);
        break;
      default:
        PRINT("Unknown performance id %u\r\n", *p);
        break;
    }
    p += 5;
  }

  return ret;
}

int8_t cmd_threshold(uint8_t argc, char **argv)
{
  if (argc == 5 && !strcasecmp(argv[1], "set")) {
    return set_threshold(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "get")) {
    return get_threshold(argc, argv);
  } else {
    cmd_help2(argv[0]);
  }
  
  return 0;
}
  
int8_t set_threshold(uint8_t argc, char **argv)
{
  int32_t val32_low, val32_high;
  int8_t ret;
  
  sscanf(argv[3], "%d", &val32_low);
  sscanf(argv[4], "%d", &val32_high);

  txBuf[0] = (uint8_t)strtoul(argv[2], NULL, 0);
  BE32_To_Buffer(val32_low, txBuf + 1);
  BE32_To_Buffer(val32_high, txBuf + 5);
  ret = process_command(CMD_SET_THRESHOLD, txBuf, 9, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  return ret;
}
  
int8_t get_threshold(uint8_t argc, char **argv)
{
  int32_t val32_low, val32_high;
  int8_t ret, i;

  if (!strcasecmp(argv[2], "all")) {
    for (i = 0; i <= 0xA; ++i) {
      txBuf[0] = i;
      ret = process_command(CMD_QUERY_THRESHOLD, txBuf, 1, rBuf, &rLen);
      if (ret) {
        return ret;
      }
      
      val32_low = (int32_t)Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + 1);
      val32_high = (int32_t)Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + 5);
      PRINT("%u(%#X): %d(%#X) %d(%#X)\r\n", rBuf[CMD_SEQ_MSG_DATA], rBuf[CMD_SEQ_MSG_DATA], val32_low, val32_low, val32_high, val32_high);
    }
  } else {
    txBuf[0] = (uint8_t)strtoul(argv[2], NULL, 0);
    ret = process_command(CMD_QUERY_THRESHOLD, txBuf, 1, rBuf, &rLen);
    if (ret) {
      return ret;
    }
    
    val32_low = (int32_t)Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + 1);
    val32_high = (int32_t)Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + 5);
    PRINT("%u(%#X): %d(%#X) %d(%#X)\r\n", rBuf[CMD_SEQ_MSG_DATA], rBuf[CMD_SEQ_MSG_DATA], val32_low, val32_low, val32_high, val32_high);
  }
  return ret;
}

int8_t cmd_for_debug(uint8_t argc, char **argv)
{
  if (argc == 5 && !strcasecmp(argv[1], "dac")) {
    return debug_dac(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "adc")) {
    return debug_adc(argc, argv);
  } else if (argc == 4 && !strcasecmp(argv[1], "tag")) {
    return debug_tag(argc, argv);
  } else if (argc >= 4 && !strcasecmp(argv[1], "cal")) {
    return debug_cal(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "get_tosa_val")) {
    return debug_get_tosa_val(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "dump")) {
    return debug_dump(argc, argv);
  } else if (argc >= 3 && !strcasecmp(argv[1], "eeprom")) {
    return debug_eeprom(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "log") && !strcasecmp(argv[2], "reset")) {
    return debug_reset_log(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "fw") && !strcasecmp(argv[2], "reset")) {
    return debug_reset_fw(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "monitor")) {
    return debug_monitor(argc, argv);
  } else if (argc == 4 && !strcasecmp(argv[1], "pin")) {
    return debug_pin(argc, argv);
  } else if (argc == 5 && !strcasecmp(argv[1], "txsw_io") && !strcasecmp(argv[3], "to")) {
    return debug_switch_io(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "rxsw_io")) {
    return debug_switch_io(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "print_hex")) {
    return debug_print_hex(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "test_tosa")) {
    return debug_test_tosa(argc, argv);
  } else if (argc == 4 && !strcasecmp(argv[1], "set_tosa")) {
    return debug_set_tosa(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "set_tec")) {
    return debug_set_tec(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "timer")) {
    return debug_set_tim(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "get_temp")) {
    return debug_get_tmp_all(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "set_freq")) {
    return debug_set_freq(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "spi")) {
    return debug_spi(argc, argv);
  } else if (argc == 2 && !strncasecmp(argv[1], "pd", 2)) {
    return debug_get_pd(argc, argv);
  } else if (argc == 3 && !strncasecmp(argv[1], "loopback", 2)) {
    return debug_set_lp(argc, argv);
  } else if (argc == 2 && !strncasecmp(argv[1], "switch_channel", 2)) {
    return debug_get_switch_channel();
  } else if (argc == 3 && !strcasecmp(argv[1], "alarm") && !strcasecmp(argv[2], "reset")) {
    return debug_reset_alarm(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "inter_exp")) {
    return debug_get_inter_exp();
  } else if (argc == 4 && !strcasecmp(argv[1], "sw_adc")) {
    return debug_set_sw_adc(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "send_hex")) {
    return debug_send_hex(argc, argv);
  } else if (argc == 3 && !strcasecmp(argv[1], "send_hex") && !strcasecmp(argv[2], "no_check")) {
    return debug_send_hex(argc, argv);
  } else if (argc == 2 && !strcasecmp(argv[1], "history_alarm")) {
    return debug_cmd_history_alarm(argc, argv);
  } else if (argc >= 2 && !strcasecmp(argv[1], "date")) {
    return debug_cmd_time(argc - 1, argv + 1);
  } else if (argc >= 4 && !strcasecmp(argv[1], "log")) {
    return debug_cmd_log(argc - 1, argv + 1);
  } else if (argc == 2 && !strcasecmp(argv[1], "check_cali")) {
    return debug_check_cali();
  } else if (argc == 2 && !strcasecmp(argv[1], "crc32_test")) {
    uint8_t buf[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    uint32_t crc;
                        
    crc = Cal_CRC32(buf, 16);
    PRINT("CRC32 : %#X\r\n", crc);

    crc = 0xFFFFFFFF;
    crc = Cal_CRC32_2(buf, 4, crc);
    crc = Cal_CRC32_2(buf + 4, 4, crc);
    crc = Cal_CRC32_2(buf + 8, 4, crc);
    crc = Cal_CRC32_2(buf + 12, 4, crc);
    crc = ~crc;
    PRINT("CRC32_2 : %#X\r\n", crc);

    return 0;
  } else if (argc == 2 && !strcasecmp(argv[1], "unlock")) {
    return debug_unlock();
  } else {
    cmd_help2(argv[0]);
    return 0;
  }
}

int8_t debug_dac(uint8_t argc, char **argv)
{
//  uint32_t sw_num, temp1, temp2;
  uint32_t sw_num;
  int32_t val_x, val_y;
  int8_t ret;

  if (!strcasecmp(argv[2], "all")) {
    sw_num = 0;
  } else if (!strncasecmp(argv[2], "sw", 2)) {
    sw_num = strtoul(argv[2] + 2, NULL, 10);
    if (sw_num < 1 || sw_num > 8) {
      PRINT("Switch number invalid\r\n");
      return 1;
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }
  
  sscanf(argv[3], "%d", &val_x);
  if (val_x <= -16384 || val_x >= 16384) {
    PRINT("Value x invalid\r\n");
    return 2;
  }
  
  sscanf(argv[4], "%d", &val_y);
  if (val_y <= -16384 || val_y >= 16384) {
    PRINT("Value y invalid\r\n");
    return 3;
  }

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_SW_DAC, txBuf + 4);
  BE32_To_Buffer(sw_num, txBuf + 8);
  BE32_To_Buffer((uint32_t)val_x, txBuf + 12);
  BE32_To_Buffer((uint32_t)val_y, txBuf + 16);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 20, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  return ret;

}

int8_t debug_adc(uint8_t argc, char **argv)
{
  uint32_t sw_num, temp;
  int8_t ret;

  if (!strncasecmp(argv[2], "sw", 2)) {
    sw_num = strtoul(argv[2] + 2, NULL, 10);
    if (sw_num < 1 || sw_num > 8) {
      PRINT("Switch number invalid\r\n");
      return 1;
    }
    
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_SW_ADC, txBuf + 4);
    BE32_To_Buffer(sw_num, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
    if (ret) {
      return ret;
    }

    temp = Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA]);
    PRINT("Positive x = %u, Negative x = %u\r\n", (temp >> 16) & 0xFFFF, (temp >> 0) & 0xFFFF);
    temp = Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 4]);
    PRINT("Positive y = %u, Negative y = %u\r\n", (temp >> 16) & 0xFFFF, (temp >> 0) & 0xFFFF);
  } else if (!strncasecmp(argv[2], "vol", 3)) {
    sw_num = strtoul(argv[2] + 3, NULL, 10);
    if (sw_num > 7) {
      PRINT("ADC number invalid\r\n");
      return 2;
    }
    
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_VOL_ADC, txBuf + 4);
    BE32_To_Buffer(sw_num, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
    if (ret) {
      return ret;
    }

    temp = Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA]);
    PRINT("Value = %u\r\n", temp & 0xFFFF);
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  return ret;
}

int8_t debug_tag(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t len = strlen(argv[3]), type;

  if (!strcasecmp(argv[2], "pn")) {
    type = 2;
    if (len > 16) {
      PRINT("PN must less than 8 bytes\r\n");
      return 1;
    }
  } else if (!strcasecmp(argv[2], "date")) {
    type = 1;
    if (len > 8) {
      PRINT("Date must less than 8 bytes\r\n");
      return 1;
    }
  } else if (!strcasecmp(argv[2], "sn")) {
    type = 0;
    if (len > 9) {
      PRINT("SN must less than 9 bytes\r\n");
      return 1;
    }
  } else if (!strcasecmp(argv[2], "vendor")) {
    type = 3;
    if (len > 16) {
      PRINT("Vendor must less than 16 bytes\r\n");
      return 1;
    }
  } else if (!strcasecmp(argv[2], "hw_version")) {
    type = 4;
    if (len > 4) {
      PRINT("Vendor must less than 4 bytes\r\n");
      return 1;
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_TAG, txBuf + 4);
  BE32_To_Buffer(type, txBuf + 8);
  if (!strcasecmp(argv[3], "clear")) {
    BE32_To_Buffer(0, txBuf + 12);
    len = 0;
  } else {
    BE32_To_Buffer(len, txBuf + 12);
    strcpy((char*)txBuf + 16, argv[3]);
  }
  ret = process_command(CMD_FOR_DEBUG, txBuf, 16 + len, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  return ret;
}

int8_t debug_cal(uint8_t argc, char **argv)
{
  uint32_t sw_num, num, u_val, u_val1;
  int32_t val_x, val_y;
  double d_val;
  int8_t ret;

  if (argc == 6 && !strncasecmp(argv[2], "sw", 2)) {
    sw_num = strtoul(argv[2] + 2, NULL, 10);
    if (sw_num < 1 || sw_num > 8) {
      PRINT("Invalid Switch number\r\n");
      return 1;
    }
    num = strtoul(argv[3], NULL, 10);
    sscanf(argv[4], "%d", &val_x);
    sscanf(argv[5], "%d", &val_y);
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_CAL_SW, txBuf + 4);
    BE32_To_Buffer(sw_num, txBuf + 8);
    BE32_To_Buffer(num, txBuf + 12);
    BE32_To_Buffer(val_x, txBuf + 16);
    BE32_To_Buffer(val_y, txBuf + 20);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 24, rBuf, &rLen);
  } else if (argc == 8 && !strcasecmp(argv[2], "tosa")) {
    sw_num = strtoul(argv[3], NULL, 10);
    if (sw_num < 1 || sw_num > 10) {
      PRINT("Invalid number\r\n");
      return 2;
    }
    sw_num -= 1;
    num = strtoul(argv[4], NULL, 10);
    u_val = strtoul(argv[5], NULL, 10);
    u_val1 = strtoul(argv[6], NULL, 10);
    sscanf(argv[7], "%lf", &d_val);
    val_x = (int32_t)(d_val * 100);
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_CAL_TOSA, txBuf + 4);
    BE32_To_Buffer(sw_num, txBuf + 8);
    BE32_To_Buffer(num, txBuf + 12);
    BE32_To_Buffer(u_val, txBuf + 16);
    BE32_To_Buffer(u_val1, txBuf + 20);
    BE32_To_Buffer((uint32_t)val_x, txBuf + 24);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 28, rBuf, &rLen);
  } else if (argc == 5 && !strcasecmp(argv[2], "il")) {
    num = strtoul(argv[3], NULL, 10);
    d_val = atof(argv[4]);
    val_x = d_val * 100;
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_CAL_IL, txBuf + 4);
    BE32_To_Buffer(num, txBuf + 8);
    BE32_To_Buffer(val_x, txBuf + 12);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 16, rBuf, &rLen);
  } else if (argc == 6 && !strcasecmp(argv[2], "pd2")) {
    num = strtoul(argv[3], NULL, 10);
    u_val = strtoul(argv[4], NULL, 10);
    sscanf(argv[5], "%lf", &d_val);
    val_x = (int32_t)(d_val * 100);
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_CAL_RX_PD, txBuf + 4);
    BE32_To_Buffer(num, txBuf + 8);
    BE32_To_Buffer(u_val, txBuf + 12);
    BE32_To_Buffer(val_x, txBuf + 16);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 20, rBuf, &rLen);
  } else if (argc == 4 && !strcasecmp(argv[2], "tec_temp")) {
    sscanf(argv[3], "%lf", &d_val);
    val_x = (int32_t)(d_val * 10);
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_CAL_DEF_TEMP, txBuf + 4);
    BE32_To_Buffer(val_x, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  return ret;
}

int8_t debug_get_tosa_val(uint8_t argc, char **argv)
{
  double d_val;
  uint32_t u_val;
  //int32_t val;
  int8_t ret;

  //sscanf(argv[2], "%lf", &d_val);
  //val = (int32_t)(d_val * 100);
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_GET_TOSA_VAL, txBuf + 4);
  //BE32_To_Buffer((uint32_t)val, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  PRINT("Returned value:\r\n");
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA]);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 4]);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 8]);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 12]);
  d_val = (double)((int32_t)u_val) / 100;
  PRINT("%.2lf\r\n", d_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA] + 16);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 20]);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 24]);
  PRINT("%u,", u_val);
  u_val = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 28]);
  d_val = (double)((int32_t)u_val) / 100;
  PRINT("%.2lf\r\n", d_val);

  return ret;
}

int8_t debug_dump(uint8_t argc, char **argv)
{
  uint32_t which, count;
  uint32_t i;
  int8_t ret;
  int32_t val_x, val_y;
  double d_val;

  if (!strncasecmp(argv[2], "sw", 2)) {
    which = strtoul(argv[2] + 2, NULL, 10);
    if (which < 1 || which > 8) {
      PRINT("Switch number invalid\r\n");
      return 1;
    }
    if (which == 1) count = 6;
    else if (which == 5) count = 3;
    else count = 11;
  } else if (!strcasecmp(argv[2], "tosa")) {
    which = 10;
    count = 10;
  } else if (!strcasecmp(argv[2], "il")) {
    which = 11;
  } else if (!strcasecmp(argv[2], "pd2")) {
    which = 15;
    count = 10;
  } else if (!strcasecmp(argv[2], "tec_temp")) {
    which = 16;
    count = 1;
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_DUMP, txBuf + 4);
  BE32_To_Buffer(which, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  if (which >= 1 && which <= 8) {
    PRINT("Switch:\r\n");
    for (i = 0; i < count; ++i) {
      val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 8]);
      val_y = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 8 + 4]);
      PRINT("%u,%d,%d\r\n", i + 1, val_x, val_y);
    }
  } else if (which == 10) {
    PRINT("Tosa:\r\n");
    for (i = 0; i < count; ++i) {
      PRINT("%u,", i + 1);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 16]);
      PRINT("%u,", which);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 16 + 4]);
      PRINT("%u,", which);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 16 + 8]);
      PRINT("%u,", which);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 16 + 12]);
      d_val = (double)((int32_t)which) / 100;
      PRINT("%.2lf\r\n", d_val);
    }
  } else if (which == 11) {
    PRINT("2x32 Insertion Loss:\r\n");
    for (i = 0; i < 32; ++i) {
      val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 4]);
      d_val = (double)val_x / 100;
      PRINT("%u,%.2lf\r\n", i + 1, d_val);
    }

    which = 12;
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_DUMP, txBuf + 4);
    BE32_To_Buffer(which, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
    if (ret) {
      return ret;
    }
    for (i = 0; i < 32; ++i) {
      val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 4]);
      d_val = (double)val_x / 100;
      PRINT("%u,%.2lf\r\n", i + 1 + 32, d_val);
    }

    which = 13;
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_DUMP, txBuf + 4);
    BE32_To_Buffer(which, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
    if (ret) {
      return ret;
    }
    PRINT("1x32 Insertion Loss:\r\n");
    for (i = 0; i < 32; ++i) {
      val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 4]);
      d_val = (double)val_x / 100;
      PRINT("%u,%.2lf\r\n", i + 1, d_val);
    }

    which = 14;
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_DUMP, txBuf + 4);
    BE32_To_Buffer(which, txBuf + 8);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
    if (ret) {
      return ret;
    }
    PRINT("Loopback Insertion Loss:\r\n");
    val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA]);
    d_val = (double)val_x / 100;
    PRINT("%.2lf\r\n", d_val);
  } else if (which == 15) {
    PRINT("PD2:\r\n");
    for (i = 0; i < count; ++i) {
      PRINT("%u,", i + 1);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 8]);
      PRINT("%u,", which);
      which = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + i * 8 + 4]);
      d_val = (double)((int32_t)which) / 100;
      PRINT("%.2lf\r\n", d_val);
    }
  } else if (which == 16) {
    val_x = (int32_t)Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA]);
    d_val = (double)val_x / 10;
    PRINT("TEC_TEMP: %.1lf\r\n", d_val);
  }

  return ret;
}

int8_t debug_eeprom(uint8_t argc, char **argv)
{
  uint32_t addr, len = 0;
  int8_t ret;
  
  if (argc == 5 && !strcasecmp(argv[2], "read")) {
    addr = strtoul(argv[3], NULL, 0);
    len = strtoul(argv[4], NULL, 0);
    if (addr & 0xFFFF0000 || len > 1024) {
      PRINT("Args invalid\r\n");
      return 1;
    }
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_EEPROM, txBuf + 4);
  BE32_To_Buffer(addr, txBuf + 8);
  BE32_To_Buffer(len, txBuf + 12);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 16, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  if (len)
    PRINT_HEX("EEPROM DUMP", rBuf + CMD_SEQ_MSG_DATA, len);
  
  return ret;
}

int8_t debug_reset_log(uint8_t argc, char **argv)
{
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_RESET_LOG, txBuf + 4);
  return process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
}

int8_t debug_reset_fw(uint8_t argc, char **argv)
{
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_RESET_FW, txBuf + 4);
  return process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
}

int8_t debug_monitor(uint8_t argc, char **argv)
{
  GPIO_PinState state;

  state = HAL_GPIO_ReadPin(IN_ALARM_GPIO_Port, IN_ALARM_Pin);
  PRINT("Alarm signal is %s\r\n", state == GPIO_PIN_SET ? "not set" : "set");
  state = HAL_GPIO_ReadPin(SW1_READY_GPIO_Port, SW1_READY_Pin);
  PRINT("TX Switch Ready signal is %s\r\n", state == GPIO_PIN_SET ? "not set" : "set");
  state = HAL_GPIO_ReadPin(SW2_READY_GPIO_Port, SW2_READY_Pin);
  PRINT("RX Switch Ready signal is %s\r\n", state == GPIO_PIN_SET ? "not set" : "set");
  state = HAL_GPIO_ReadPin(L_READY_GPIO_Port, L_READY_Pin);
  PRINT("L_Ready signal is %s\r\n", state == GPIO_PIN_SET ? "not set" : "set");

  return 0;
}

int8_t debug_print_hex(uint8_t argc, char **argv)
{
  if (!strcasecmp(argv[2], "on")) {
    print_trans_data = 1;
  } else if (!strcasecmp(argv[2], "off")) {
    print_trans_data = 0;
  } else {
    cmd_help2(argv[0]);
    return 0;
  }
  
  return 0;
}

int8_t debug_pin(uint8_t argc, char **argv)
{
  uint32_t val, type = 0xFF;
  GPIO_TypeDef *port = NULL;
  uint16_t pin = 0;
  int8_t ret;

  if (!strcasecmp(argv[2], "latch")) {
    type = 0;
  } else if (!strcasecmp(argv[2], "alarm")) {
    type = 1;
  } else if (!strcasecmp(argv[2], "txsw_ready")) {
    type = 2;
  } else if (!strcasecmp(argv[2], "rxsw_ready")) {
    type = 3;
  } else if (!strcasecmp(argv[2], "l_ready")) {
    type = 4;
  } else if (!strcasecmp(argv[2], "test_block")) {
    type = 5;
  } else if (!strcasecmp(argv[2], "pro")) {
    port = PRO_CTL_GPIO_Port;
    pin = PRO_CTL_Pin;
  } else if (!strcasecmp(argv[2], "master_reset")) {
    port = MASTER_RESET_GPIO_Port;
    pin = MASTER_RESET_Pin;
  } else if (!strcasecmp(argv[2], "hard_reset")) {
    port = HARD_RESET_GPIO_Port;
    pin = HARD_RESET_Pin;
  } else if (!strcasecmp(argv[2], "sw1_block")) {
    port = SW1_BLOCK_GPIO_Port;
    pin = SW1_BLOCK_Pin;
  } else if (!strcasecmp(argv[2], "pro_dis_n")) {
    port = PRO_DIS_N_GPIO_Port;
    pin = PRO_DIS_N_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D0")) {
    port = SW1_D0_GPIO_Port;
    pin = SW1_D0_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D1")) {
    port = SW1_D1_GPIO_Port;
    pin = SW1_D1_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D2")) {
    port = SW1_D2_GPIO_Port;
    pin = SW1_D2_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D3")) {
    port = SW1_D3_GPIO_Port;
    pin = SW1_D3_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D4")) {
    port = SW1_D4_GPIO_Port;
    pin = SW1_D4_Pin;
  } else if (!strcasecmp(argv[2], "sw1_D5")) {
    port = SW1_D5_GPIO_Port;
    pin = SW1_D5_Pin;
  } else if (!strcasecmp(argv[2], "sw1_mode")) {
    port = SW1_MODE_SEL_GPIO_Port;
    pin = SW1_MODE_SEL_Pin;
  } else if (!strcasecmp(argv[2], "sw1_strobe")) {
    port = SW1_STROBE_GPIO_Port;
    pin = SW1_STROBE_Pin;
  } else if (!strcasecmp(argv[2], "sw2_D0")) {
    port = SW2_D0_GPIO_Port;
    pin = SW2_D0_Pin;
  } else if (!strcasecmp(argv[2], "sw2_D1")) {
    port = SW2_D1_GPIO_Port;
    pin = SW2_D1_Pin;
  } else if (!strcasecmp(argv[2], "sw2_D2")) {
    port = SW2_D2_GPIO_Port;
    pin = SW2_D2_Pin;
  } else if (!strcasecmp(argv[2], "sw2_D3")) {
    port = SW2_D3_GPIO_Port;
    pin = SW2_D3_Pin;
  } else if (!strcasecmp(argv[2], "sw2_D4")) {
    port = SW2_D4_GPIO_Port;
    pin = SW2_D4_Pin;
  } else if (!strcasecmp(argv[2], "sw2_mode")) {
    port = SW2_MODE_SEL_GPIO_Port;
    pin = SW2_MODE_SEL_Pin;
  } else if (!strcasecmp(argv[2], "sw2_strobe")) {
    port = SW2_STROBE_GPIO_Port;
    pin = SW2_STROBE_Pin;
  } else {
    cmd_help2(argv[0]);
    return 0;
  }

  val = strtoul(argv[3], NULL, 10);
  if (type == 0xFF) {
    if (val) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
  } else {
    BE32_To_Buffer(0x5A5AA5A5, txBuf);
    BE32_To_Buffer(CMD_DEBUG_PIN, txBuf + 4);
    BE32_To_Buffer(type, txBuf + 8);
    BE32_To_Buffer(val, txBuf + 12);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 16, rBuf, &rLen);
    if (ret) {
      return ret;
    }
  }

  return 0;
}

int8_t debug_switch_io(uint8_t argc, char **argv)
{
  uint32_t chan;

  if (!strcasecmp(argv[1], "txsw_io")) {
    chan = strtoul(argv[4], NULL, 10);
    if (chan > 32 || chan == 0) {
      PRINT("Channel number invalid\r\n");
      return 1;
    }
    chan -= 1;
    if (!strcasecmp(argv[2], "1")) {
      chan = (chan << 1) & (~0x1);
    } else if (!strcasecmp(argv[2], "2")) {
      chan = (chan << 1) | 0x1;
    } else {
      cmd_help2(argv[0]);
      return 0;
    }
    PRINT("Set value: %#X\r\n", chan);
    HAL_GPIO_WritePin(SW1_STROBE_GPIO_Port, SW1_STROBE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SW1_MODE_SEL_GPIO_Port, SW1_MODE_SEL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW1_D0_GPIO_Port, SW1_D0_Pin, (GPIO_PinState)((chan >> 0) & 0x1));
    HAL_GPIO_WritePin(SW1_D1_GPIO_Port, SW1_D1_Pin, (GPIO_PinState)((chan >> 1) & 0x1));
    HAL_GPIO_WritePin(SW1_D2_GPIO_Port, SW1_D2_Pin, (GPIO_PinState)((chan >> 2) & 0x1));
    HAL_GPIO_WritePin(SW1_D3_GPIO_Port, SW1_D3_Pin, (GPIO_PinState)((chan >> 3) & 0x1));
    HAL_GPIO_WritePin(SW1_D4_GPIO_Port, SW1_D4_Pin, (GPIO_PinState)((chan >> 4) & 0x1));
    HAL_GPIO_WritePin(SW1_D5_GPIO_Port, SW1_D5_Pin, (GPIO_PinState)((chan >> 5) & 0x1));
    HAL_GPIO_WritePin(SW1_STROBE_GPIO_Port, SW1_STROBE_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SW1_MODE_SEL_GPIO_Port, SW1_MODE_SEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SW1_STROBE_GPIO_Port, SW1_STROBE_Pin, GPIO_PIN_SET);
  } else {
    chan = strtoul(argv[2], NULL, 10);
    if (chan > 32 || chan == 0) {
      PRINT("Channel number invalid\r\n");
      return 1;
    }
    chan -= 1;
    PRINT("Set value: %#X\r\n", chan);
    HAL_GPIO_WritePin(SW2_STROBE_GPIO_Port, SW2_STROBE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SW2_MODE_SEL_GPIO_Port, SW2_MODE_SEL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW2_D0_GPIO_Port, SW2_D0_Pin, (GPIO_PinState)((chan >> 0) & 0x1));
    HAL_GPIO_WritePin(SW2_D1_GPIO_Port, SW2_D1_Pin, (GPIO_PinState)((chan >> 1) & 0x1));
    HAL_GPIO_WritePin(SW2_D2_GPIO_Port, SW2_D2_Pin, (GPIO_PinState)((chan >> 2) & 0x1));
    HAL_GPIO_WritePin(SW2_D3_GPIO_Port, SW2_D3_Pin, (GPIO_PinState)((chan >> 3) & 0x1));
    HAL_GPIO_WritePin(SW2_D4_GPIO_Port, SW2_D4_Pin, (GPIO_PinState)((chan >> 4) & 0x1));
    HAL_GPIO_WritePin(SW2_STROBE_GPIO_Port, SW2_STROBE_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SW2_MODE_SEL_GPIO_Port, SW2_MODE_SEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SW2_STROBE_GPIO_Port, SW2_STROBE_Pin, GPIO_PIN_SET);
  }

  return 0;
}

int8_t debug_test_tosa(uint8_t argc, char **argv)
{
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_TEST_TOSA, txBuf + 4);
  return process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
}

int8_t debug_set_tosa(uint8_t argc, char **argv)
{
  uint32_t high = strtoul(argv[2], NULL, 10);
  uint32_t low = strtoul(argv[3], NULL, 10);
  
  if (low >= high) {
    PRINT("Error! low lager than high value.\r\n");
    return -1;
  }
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_SET_TOSA, txBuf + 4);
  BE32_To_Buffer(low, txBuf + 8);
  BE32_To_Buffer(high, txBuf + 12);
  return process_command(CMD_FOR_DEBUG, txBuf, 16, rBuf, &rLen);
}

int8_t debug_set_tec(uint8_t argc, char **argv)
{
  uint32_t value = strtoul(argv[2], NULL, 10);
  
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_SET_TEC, txBuf + 4);
  BE32_To_Buffer(value, txBuf + 8);
  return process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
}

int8_t debug_set_tim(uint8_t argc, char **argv)
{
  if (!strcasecmp(argv[2], "on")) {
    HAL_TIM_Base_Start_IT(&htim3);
  } else if (!strcasecmp(argv[2], "off")) {
    HAL_TIM_Base_Stop_IT(&htim3);
  } else {
    cmd_help2(argv[0]);
    return 0;
  }
  
  return 0;
}

int8_t debug_get_tmp_all(uint8_t argc, char **argv)
{
  int8_t ret;
  uint8_t *p;
  uint32_t value;
  int16_t temp;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_GET_TOSA_TMP, txBuf + 4);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  p = rBuf + CMD_SEQ_MSG_DATA;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 4;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("Tosa Temperature: %.2lfC, Original value: %u\r\n", (double)temp/100, value);

  p = rBuf + CMD_SEQ_MSG_DATA + 8;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 12;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("TEC Voltage: %dmV, Original value: %u\r\n", temp, value);

  p = rBuf + CMD_SEQ_MSG_DATA + 16;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 20;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("TEC Current: %dmA, Original value: %u\r\n", temp, value);

  p = rBuf + CMD_SEQ_MSG_DATA + 24;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 28;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("LD Current: %dmA, Original value: %u\r\n", temp, value);

  p = rBuf + CMD_SEQ_MSG_DATA + 32;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 36;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("LD Voltage: %dmV, Original value: %u\r\n", temp, value);

  p = rBuf + CMD_SEQ_MSG_DATA + 40;
  value = switch_endian(*(uint32_t*)p);
  p = rBuf + CMD_SEQ_MSG_DATA + 44;
  temp = (int16_t)switch_endian(*(uint32_t*)p);
  PRINT("MPD_Current: %duA, Original value: %u\r\n", temp, value);

  return 0;
}

int8_t debug_set_freq(uint8_t argc, char **argv)
{
  uint32_t value = strtoul(argv[2], NULL, 10);
  if (value == 0) {
    return 1;
  }
  tim_counter_max = value;
  return 0;
}

int8_t debug_spi(uint8_t argc, char **argv)
{
  uint8_t chan;
  uint8_t txbuf[2], rxbuf[2], chan_rb;
  HAL_StatusTypeDef hal_status;
  uint16_t val;
  
  chan = (uint8_t)strtoul(argv[2], NULL, 10);
  if (chan > 15) {
    PRINT("Channel number invalid\r\n");
    return 1;
  }
  txbuf[0] = (0x1 << 4) | (0x0 << 3) | (chan >> 1);
  txbuf[1] = chan << 7;
  PRINT("txbuf: %#X, %#X\r\n", txbuf[0], txbuf[1]);

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  //osDelay(1);
  PRINT("rxbuf: %#X, %#X\r\n", rxbuf[0], rxbuf[1]);

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  //osDelay(1);
  PRINT("rxbuf: %#X, %#X\r\n", rxbuf[0], rxbuf[1]);

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  PRINT("rxbuf: %#X, %#X\r\n", rxbuf[0], rxbuf[1]);

  if (hal_status != HAL_OK) {
    PRINT("Read ADC7953_SPI5 failed, ErrorCode = %#X\r\n", hspi5.ErrorCode);
    return 2;
  }

  chan_rb = rxbuf[0] >> 4;
  if (chan_rb != chan) {
    PRINT("Read ADC7953_SPI5 failed, rChanIdx != chanIdx\r\n");
    return 3;
  } else {
    val = ((rxbuf[0] & 0xf) << 8) | rxbuf[1];
    PRINT("Value = %u\r\n", val);
  }

  return 0;
}

int8_t debug_get_pd(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t value, which;
  int32_t val;

  value = strtoul(argv[1] + 2, NULL, 10);
  if (value == 1) {
    which = 0;
  } else if (value == 2) {
    which = 1;
  } else {
    cmd_help2(argv[0]);
    return 0; 
  }
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_GET_PD, txBuf + 4);
  BE32_To_Buffer(which, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    value = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA);
    PRINT("PD error code is %#X\r\n", value);
    return ret;
  }

  value = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA);
  val = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + 4);
  PRINT("PD adc: %u\r\n", value);
  PRINT("PD power: %.2lf\r\n", (double)val / 100);
  return ret;
}

int8_t debug_set_lp(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t value, which;

  value = strtoul(argv[2], NULL, 10);
  if (value == 1) {
    which = 0;
  } else if (value == 2) {
    which = 1;
  } else if (value == 3) {
    which = 2;
  } else {
    cmd_help2(argv[0]);
    return 0; 
  }
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_SET_LP, txBuf + 4);
  BE32_To_Buffer(which, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  return ret;
}

int8_t debug_get_switch_channel()
{
  int8_t ret;
  uint32_t which;

  which = 0;
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_GET_SW_CHAN, txBuf + 4);
  BE32_To_Buffer(which, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  if (rBuf[CMD_SEQ_MSG_DATA + 1] < 32) {
    PRINT("TX channel is 1 to %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] + 1);
  } else if (rBuf[CMD_SEQ_MSG_DATA + 1] < 64) {
    PRINT("TX channel is 2 to %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] - 32 + 1);
  } else if (rBuf[CMD_SEQ_MSG_DATA + 1] < 65) {
    PRINT("TX channel is 1 to 33\r\n");
  } else if (rBuf[CMD_SEQ_MSG_DATA + 1] < 66) {
    PRINT("TX channel is 2 to 33\r\n");
  } else {
    PRINT("Invalid TX channel\r\n");
  }

  which = 1;
  BE32_To_Buffer(which, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  if (rBuf[CMD_SEQ_MSG_DATA + 1] < 32) {
    PRINT("RX channel is %u\r\n", rBuf[CMD_SEQ_MSG_DATA + 1] + 1);
  } else if (rBuf[CMD_SEQ_MSG_DATA + 1] < 33) {
    PRINT("RX channel is 33\r\n");
  } else {
    PRINT("Invalid RX channel\r\n");
  }


  return ret;
}

int8_t debug_reset_alarm(uint8_t argc, char **argv)
{
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_RESET_ALARM, txBuf + 4);
  return process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
}

int8_t debug_get_inter_exp()
{
  int8_t ret;
  uint32_t value;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_INTER_EXP, txBuf + 4);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
   if (ret) {
    return ret;
  }

  value = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA);
  PRINT("Internal Exception Code is %#X\r\n", value);
  return ret;
}

int8_t debug_set_sw_adc(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t value;
  double d_val;

  value = strtoul(argv[2], NULL, 10);
  d_val = atof(argv[3]);

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_SET_SW_ADC, txBuf + 4);
  BE32_To_Buffer(value, txBuf + 8);
  BE32_To_Buffer((uint32_t)(d_val * 1000), txBuf + 12);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 16, rBuf, &rLen);
   if (ret) {
    return ret;
  }

  return ret;
}

int8_t debug_send_hex(uint8_t argc, char **argv)
{
  char *p;
  uint8_t *p_data = fw_buf + 1024 * 10;
  uint32_t i;
  uint8_t rcv_crc;
  uint8_t rcv_len, err_code;

  HAL_Delay(1);
  __HAL_UART_DISABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
  CLEAR_BIT(TERMINAL_UART.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&TERMINAL_UART);
  uart1_irq_sel = 0;
  PRINT("Put data in 15 seconds...\r\n");

  memset(fw_buf, 0, 1024 * 10);
  if (HAL_UART_Receive(&TERMINAL_UART, fw_buf, 1024 * 10, 1000 * 15) != HAL_TIMEOUT) {
    PRINT("Failed\r\n");
    __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
    uart1_irq_sel = 1;
    return 1;
  }

  __HAL_UART_ENABLE_IT(&TERMINAL_UART, UART_IT_RXNE);
  uart1_irq_sel = 1;

  if (strlen((char*)fw_buf) == 0) {
    PRINT("No data received\r\n");
  } else {
    for (p = strtok((char*)fw_buf, " \t\r\n,"), i = 0; p != NULL; ++i, p = strtok(NULL, " \t\r\n,")) {
      p_data[i] = (uint8_t)strtoul(p, NULL, 0);
    }
    if (argc == 3 && !strcasecmp(argv[2], "no_check")) {
      rcv_crc = Cal_Check((uint8_t*)&p_data[1], i - 1);
      p_data[i] = rcv_crc;
      i += 1;
    }
    PRINT_HEX("tx_buf", p_data, i);
    if (HAL_UART_Transmit(&COMMUNICATION_UART, p_data, i, 0xFF) != HAL_OK) {
      EPT("Transmit failed\r\n");
      return 100;
    }
    rBuf[0] = 0;
    while (rBuf[0] != TRANS_START_BYTE) {
      if (HAL_UART_Receive(&COMMUNICATION_UART, rBuf, 1, 950) != HAL_OK) {
        EPT("Receive failed : Received timeout 1\r\n");
        return 101;
      }
    }
    if (HAL_UART_Receive(&COMMUNICATION_UART, rBuf + 1, 1, 0xFF) != HAL_OK) {
      EPT("Receive failed : Received timeout 2\r\n");
      return 102;
    }
    rcv_len = rBuf[1];
    if (HAL_UART_Receive(&COMMUNICATION_UART, rBuf + 2, rcv_len - 1, 0xFF) != HAL_OK) {
      EPT("Receive failed : Received timeout 3\r\n");
      PRINT_HEX("Received messages", rBuf, 2);
      return 103;
    }
    PRINT_HEX("rx_buf", rBuf, rcv_len + 1);

    rcv_crc = Cal_Check(&rBuf[1], rcv_len - 1);
    if (rcv_crc != rBuf[1 + rcv_len - 1]) {
      EPT("Checksum failed\r\n");
      return 104;
    }

    err_code = rBuf[1 + rcv_len - 2];

    PRINT("Returned status from module is %d (= %#X)\r\n", err_code, err_code);
    if (err_code != 0) {
      return 105;
    }
  }

  return 0;
}

int8_t debug_cmd_history_alarm(uint8_t argc, char **argv)
{
  int8_t ret, i;
  uint32_t seq, exp;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_HISTORY_ALARM, txBuf + 4);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  for (i = 0; i < 10; i++) {
    seq = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + i * 8);
    exp = Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA + i * 8 + 4);
    PRINT("Seq : %u, Alarm: %#X\r\n", seq, exp);
  }

  return ret;
}

int8_t debug_cmd_time(uint8_t argc, char **argv)
{
  if (argc == 8 && !strcasecmp(argv[1], "set")) {
    return debug_set_log_time(argc - 2, argv + 2);
  } else if (argc == 2 && !strcasecmp(argv[1], "get")) {
    return debug_get_log_time();
  } else {
    cmd_help2("debug");
  }
  return 0;
}

int8_t debug_set_log_time(uint8_t argc, char **argv)
{
  txBuf[12] = strtoul(argv[0], NULL, 10) - 2000;
  txBuf[13] = strtoul(argv[1], NULL, 10);
  txBuf[14] = strtoul(argv[2], NULL, 10);
  txBuf[15] = strtoul(argv[3], NULL, 10);
  txBuf[16] = strtoul(argv[4], NULL, 10);
  txBuf[17] = strtoul(argv[5], NULL, 10);

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_DATE, txBuf + 4);
  BE32_To_Buffer(0, txBuf + 8);
  return process_command(CMD_FOR_DEBUG, txBuf, 18, rBuf, &rLen);
}

int8_t debug_get_log_time(void)
{
  int8_t ret;
  uint8_t *p = rBuf + CMD_SEQ_MSG_DATA;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_DATE, txBuf + 4);
  BE32_To_Buffer(1, txBuf + 8);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 12, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  PRINT("Received date %u-%u-%u %u:%u:%u\r\n", *p + 2000, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5));
  
  return ret;
}

int8_t debug_cmd_log(uint8_t argc, char **argv)
{
  if (argc == 3 && !strcasecmp(argv[1], "size")) {
    return debug_log_size(argc, argv);
  } else if (argc == 5 && !strcasecmp(argv[1], "get")) {
    return debug_log_content(argc, argv);
  } else {
    cmd_help2("debug");
  }
  return 0;
}

int8_t debug_log_size(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t size;
  uint8_t *p;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_LOG, txBuf + 4);
  BE32_To_Buffer(0, txBuf + 8);
  txBuf[12] = (uint8_t)strtoul(argv[2], NULL, 0);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 13, rBuf, &rLen);
  if (ret) {
    return ret;
  }

  p = rBuf + CMD_SEQ_MSG_DATA + 1;
  size = Buffer_To_BE32(p);
  PRINT("type = %u, size = %u\r\n", rBuf[CMD_SEQ_MSG_DATA], size);

  return ret;
}

int8_t debug_log_content(uint8_t argc, char **argv)
{
  int8_t ret;
  uint32_t offset, offset_returned;
  uint32_t size, cplt_size;
  
  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_LOG, txBuf + 4);
  BE32_To_Buffer(1, txBuf + 8);
  txBuf[12] = (uint8_t)strtoul(argv[2], NULL, 0);
  offset = strtoul(argv[3], NULL, 0);
  size = strtoul(argv[4], NULL, 0);
  if (size > 200) {
    for (cplt_size = 0; cplt_size < size; ) {
      if (cplt_size + 200 > size) {
        txBuf[13] = size - cplt_size;    
        BE32_To_Buffer(offset + cplt_size, txBuf + 14);
        cplt_size += txBuf[13];
      } else {
        txBuf[13] = 200;    
        BE32_To_Buffer(offset + cplt_size, txBuf + 14);
        cplt_size += txBuf[13];
      }
      ret = process_command(CMD_FOR_DEBUG, txBuf, 18, rBuf, &rLen);
      if (ret) {
        return ret;
      }

      offset_returned = Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 2]);
      PRINT("type = %u, offset = %u, length = %u\r\n", rBuf[CMD_SEQ_MSG_DATA], offset_returned, rBuf[CMD_SEQ_MSG_DATA + 1]);
      if (rBuf[CMD_SEQ_MSG_DATA] == 0) {
        PRINT_HEX("LOG", &rBuf[CMD_SEQ_MSG_DATA + 6], rBuf[CMD_SEQ_MSG_DATA + 1]);
      } else {
        PRINT_CHAR("LOG", &rBuf[CMD_SEQ_MSG_DATA + 6], rBuf[CMD_SEQ_MSG_DATA + 1]);
      }
    }
  } else {
    txBuf[13] = (uint8_t)size;    
    BE32_To_Buffer(offset, txBuf + 14);
    ret = process_command(CMD_FOR_DEBUG, txBuf, 18, rBuf, &rLen);
    if (ret) {
      return ret;
    }

    offset_returned = Buffer_To_BE32(&rBuf[CMD_SEQ_MSG_DATA + 2]);
    PRINT("type = %u, offset = %u, length = %u\r\n", rBuf[CMD_SEQ_MSG_DATA], offset_returned, rBuf[CMD_SEQ_MSG_DATA + 1]);
    if (rBuf[CMD_SEQ_MSG_DATA] == 0) {
      PRINT_HEX("LOG", &rBuf[CMD_SEQ_MSG_DATA + 6], rBuf[CMD_SEQ_MSG_DATA + 1]);
    } else {
      PRINT_CHAR("LOG", &rBuf[CMD_SEQ_MSG_DATA + 6], rBuf[CMD_SEQ_MSG_DATA + 1]);
    }
  }

  return ret;
}

int8_t debug_check_cali(void)
{
  int8_t ret;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_CHECK_CALI, txBuf + 4);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
  if (ret) {
    return ret;
  }
  
  PRINT("CRC32 : %#X\r\n", Buffer_To_BE32(rBuf + CMD_SEQ_MSG_DATA));

  return ret;
}

int8_t debug_unlock(void)
{
  int8_t ret;

  BE32_To_Buffer(0x5A5AA5A5, txBuf);
  BE32_To_Buffer(CMD_DEBUG_UNLOCK, txBuf + 4);
  ret = process_command(CMD_FOR_DEBUG, txBuf, 8, rBuf, &rLen);
   if (ret) {
    return ret;
  }

  return ret;
}


int8_t process_command(uint16_t cmd, uint8_t *pdata, uint8_t len, uint8_t *rx_buf, uint8_t *rx_len)
{
  uint8_t cmd_len = 0;
  uint32_t print_len;
  uint8_t rcv_crc;
  uint8_t rcv_len, err_code;
  uint32_t tim2_counter_value;

  CLEAR_BIT(COMMUNICATION_UART.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&COMMUNICATION_UART);

  communication_buf[cmd_len++] = TRANS_START_BYTE;
  communication_buf[cmd_len++] = 1 + 2 + len + 1; // Length + command + data + chk
  communication_buf[cmd_len++] = (uint8_t)(cmd >> 8);
  communication_buf[cmd_len++] = (uint8_t)cmd;
  if (len) {
    memcpy(&communication_buf[cmd_len], pdata, len);
    cmd_len += len;
  }
  communication_buf[cmd_len++] = Cal_Check((uint8_t*)&communication_buf[1], 1 + 2 + len);

  //PRINT("tx crc32 = %#X\r\n", rcv_crc);
  if (print_trans_data && switch_endian_16(*(uint16_t*)&communication_buf[CMD_SEQ_MSG_ID]) != CMD_FOR_DEBUG) {
    if (cmd_len > 0xFF) print_len = 0x100;
    else print_len = cmd_len;
#if 0
    if (switch_endian(*(uint32_t*)&communication_buf[CMD_SEQ_MSG_ID]) != CMD_UPGRADE_DATA) {
      //PRINT_HEX("tx_buf", communication_buf, print_len);
    }
#endif
    PRINT_HEX("tx_buf", communication_buf, print_len);
  }

  HAL_TIM_Base_Init(&htim2);
  tim2_counter = 0;
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
  if (HAL_UART_Transmit(&COMMUNICATION_UART, communication_buf, cmd_len, 0xFF) != HAL_OK) {
    EPT("Transmit failed\r\n");
    return 100;
  }

  rx_buf[0] = 0;
  while (rx_buf[0] != TRANS_START_BYTE) {
    if (HAL_UART_Receive(&COMMUNICATION_UART, rx_buf, 1, 950) != HAL_OK) {
      EPT("Receive failed : Received timeout 1\r\n");
      return 101;
    }
  }
  if (HAL_UART_Receive(&COMMUNICATION_UART, rx_buf + 1, 1, 0xFF) != HAL_OK) {
    EPT("Receive failed : Received timeout 2\r\n");
    return 102;
  }
  rcv_len = rx_buf[1];
  if (HAL_UART_Receive(&COMMUNICATION_UART, rx_buf + 2, rcv_len - 1, 0xFF) != HAL_OK) {
    EPT("Receive failed : Received timeout 3\r\n");
    PRINT_HEX("Received messages", rx_buf, 2);
    return 103;
  }
  HAL_TIM_Base_Stop_IT(&htim2);
  tim2_counter_value = __HAL_TIM_GET_COUNTER(&htim2);
  if (print_trans_data && switch_endian_16(*(uint16_t*)&communication_buf[CMD_SEQ_MSG_ID]) != CMD_FOR_DEBUG) {
    print_len = rcv_len + 1;
#if 0
    if (switch_endian(*(uint32_t*)&rx_buf[CMD_SEQ_MSG_ID]) != CMD_UPGRADE_DATA && 
      switch_endian(*(uint32_t*)&rx_buf[CMD_SEQ_MSG_ID]) != CMD_QUERY_LOG) {
      //PRINT_HEX("rx_buf", rx_buf, print_len);
    }
#endif
    PRINT_HEX("rx_buf", rx_buf, print_len);
  }

  rcv_crc = Cal_Check(&rx_buf[1], rcv_len - 1);
  if (rcv_crc != rx_buf[1 + rcv_len - 1]) {
    EPT("Check failed\r\n");
    return 104;
  }

  err_code = rx_buf[1 + rcv_len - 2];

  PRINT("Time is %ums\r\n", tim2_counter * 1500 + tim2_counter_value);
  PRINT("Returned status from module is %d (= %#X)\r\n", err_code, err_code);
  if (err_code != 0) {
    return 105;
  }
  *rx_len = rcv_len + 1;

  return 0;
}

uint8_t Cal_Check(uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  uint8_t chk = 0;

  for (i = 0; i < len; ++i) {
    chk ^= pdata[i];
  }
  
  return (chk + 1);
}

/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = UpdateCRC16(crc, *p_data++);
 
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return crc&0xffffu;
}

static uint32_t CRC32_TABLE[] = {
  /* CRC polynomial 0xedb88320 */
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
  0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
  0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
  0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
  0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
  0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
  0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
  0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
  0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
  0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
  0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
  0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
  0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
  0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
  0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
  0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
  0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
  0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
  0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
  0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
  0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
  0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t Cal_CRC32(uint8_t* packet, uint32_t length)
{
  uint32_t CRC32 = 0xFFFFFFFF;
  for (uint32_t i = 0; i < length; i++) {
    CRC32 = (CRC32_TABLE[((CRC32) ^ (packet[i])) & 0xff] ^ ((CRC32) >> 8));
  }

  return ~CRC32;
}

uint32_t Cal_CRC32_2(uint8_t* packet, uint32_t length, uint32_t CRC32)
{
  for (uint32_t i = 0; i < length; i++) {
    CRC32 = (CRC32_TABLE[((CRC32) ^ (packet[i])) & 0xff] ^ ((CRC32) >> 8));
  }

  return CRC32;
}

void PRINT_HEX(char *head, uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  
  PRINT("************* PRINT HEX *************\r\n");
  PRINT("%s:\r\n", head);
  for (i = 0; i < len; ++i) {
    if (i % 0x10 == 0) {
      HAL_Delay(5);
      PRINT("%08X : ", i / 0x10);
    }
    PRINT("0x%02X%s", pdata[i], (i + 1) % 0x10 == 0 ? "\r\n" : i == len - 1 ? "\r\n" : " ");
  }
  PRINT("************* PRINT END *************\r\n");
}

void PRINT_CHAR(char *head, uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  
  PRINT("************* PRINT CHAR *************\r\n");
  PRINT("%s:\r\n", head);
  for (i = 0; i < len; ++i) {
    if (i % 0x40 == 0) {
      HAL_Delay(5);
      PRINT("%08X : ", i / 0x40);
    }
    PRINT("%c%s", pdata[i] == '\n' ? 'N' : pdata[i] == '\r' ? 'R' : pdata[i],\
          (i + 1) % 0x40 == 0 ? "\r\n" : i == len - 1 ? "\r\n" : "");
  }
  PRINT("************* PRINT CHAR *************\r\n");
}

uint32_t switch_endian(uint32_t i)
{
  return (((i>>24)&0xff) | ((i>>8)&0xff00) | ((i<<8)&0xff0000) | ((i<<24)&0xff000000));
}

uint16_t switch_endian_16(uint16_t i)
{
  return (((i>>8)&0xff) | ((i<<8)&0xff00));
}

void BE32_To_Buffer(uint32_t data, uint8_t *pbuf)
{
  pbuf[0] = (uint8_t)(data >> 24);
  pbuf[1] = (uint8_t)(data >> 16);
  pbuf[2] = (uint8_t)(data >> 8);
  pbuf[3] = (uint8_t)(data);
}

uint32_t Buffer_To_BE32(uint8_t *pbuf)
{
  return (pbuf[0] << 24) | (pbuf[1] << 16) | (pbuf[2] << 8) | (pbuf[3]);
}

void BE16_To_Buffer(uint16_t data, uint8_t *pbuf)
{
  pbuf[0] = (uint8_t)(data >> 8);
  pbuf[1] = (uint8_t)(data);
}

uint16_t Buffer_To_BE16(uint8_t *pbuf)
{
  return (pbuf[0] << 8) | (pbuf[1]);
}



