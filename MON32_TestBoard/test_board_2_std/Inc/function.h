#ifndef __FUNCTION_H
#define __FUNCTION_H

#include <stdint.h>

#define EPT(format, ...)	do{\
                      sprintf((char*)terminal_buf, "%s,%d: " format, __func__, __LINE__, ##__VA_ARGS__);\
                      if (uart1_irq_sel) {\
                        serial_tx(terminal_buf, strlen((char*)terminal_buf));\
                      } else {\
                        HAL_UART_Transmit(&TERMINAL_UART, terminal_buf, strlen((char*)terminal_buf), 0xFF);\
                      }\
                    } while(0)
#define PRINT(format, ...)	do{\
                      sprintf((char*)terminal_buf, format, ##__VA_ARGS__);\
                      if (uart1_irq_sel) {\
                        serial_tx(terminal_buf, strlen((char*)terminal_buf));\
                      } else {\
                        HAL_UART_Transmit(&TERMINAL_UART, terminal_buf, strlen((char*)terminal_buf), 0xFF);\
                      }\
                    } while(0)
#define PRINT2(format, ...)	do{\
                      sprintf((char*)terminal_buf, format, ##__VA_ARGS__);\
                      HAL_UART_Transmit(&TERMINAL_UART, terminal_buf, strlen((char*)terminal_buf), 0xFF);\
                    } while(0)

#define TRANS_START_BYTE            0x55
#define TRANS_MAX_LENGTH            256

typedef enum {
  CMD_SEQ_MSG_LENGTH      = 0x01,
  CMD_SEQ_MSG_ID          = 0x02,
  CMD_SEQ_MSG_DATA        = 0x04,
} CmdSeq;

typedef enum {
  CMD_QUERY_SN            = 0x02,
  CMD_QUERY_VERSION       = 0x03,
  CMD_QUERY_MDATE         = 0x06,
  CMD_QUERY_PN            = 0x07,
  CMD_QUERY_VENDOR        = 0x08,
  CMD_SET_SWITCH          = 0x10,
  CMD_QUERY_SWITCH        = 0x11,
  CMD_QUERY_TEMP          = 0x17,
  CMD_QUERY_IL            = 0x18,
  CMD_QUERY_TOSA_THR      = 0x19,
  CMD_SET_TOSA            = 0x1A,
  CMD_QUERY_TOSA          = 0x1B,
  CMD_RX_PD_CALI          = 0x1C,
  CMD_TAP_PD_CALI         = 0x1D,
  CMD_QUERY_VOLTAGE       = 0x1F,
  CMD_LOOPBACK_TEST       = 0x20,
  CMD_LOOPBACK_RESULT     = 0x21,
  CMD_QUERY_ALARM         = 0x22,
  CMD_SET_MODULATION      = 0x25,
  CMD_QUERY_MODULATION    = 0x26,
  CMD_QUERY_STATUS        = 0x27,
  CMD_QUERY_ALARM_HISTORY = 0x28,
  CMD_LOG_NUMBER          = 0x75,
  CMD_LOG_CONTENT         = 0x76,
  CMD_UPGRADE_MODE        = 0x80,
  CMD_UPGRADE_DATA        = 0x81,
  CMD_UPGRADE_RUN         = 0x83,
  CMD_SOFTRESET           = 0x84,
  CMD_SET_LOG_TIME        = 0x85,
  CMD_QUERY_LOG_TIME      = 0x86,
  CMD_QUERY_PERFORMANCE   = 0x87,
  CMD_SET_THRESHOLD       = 0x88,
  CMD_QUERY_THRESHOLD     = 0x89,
  CMD_QUERY_LOG_SIZE      = 0x8A,
  CMD_GET_LOG_CONTENT     = 0x8B,

  // for test
  CMD_FOR_DEBUG           = 0x7FFF,
} CmdId;

typedef enum {
  CMD_DEBUG_SW_DAC        = 0x01,
  CMD_DEBUG_SW_ADC        = 0x02,
  CMD_DEBUG_VOL_ADC       = 0x03,
  CMD_DEBUG_TAG           = 0x04,
  CMD_DEBUG_PIN           = 0x05,
  CMD_DEBUG_CAL_SW        = 0x06,
  CMD_DEBUG_CAL_IL        = 0x07,
  CMD_DEBUG_DUMP          = 0x09,
  CMD_DEBUG_EEPROM        = 0x0A,
  CMD_DEBUG_RESET_LOG     = 0x0B,
  CMD_DEBUG_RESET_FW      = 0x0D,
  CMD_DEBUG_TEST_TOSA     = 0x0E,
  CMD_DEBUG_SET_TOSA      = 0x0F,
  CMD_DEBUG_SET_TEC       = 0x10,
  CMD_DEBUG_GET_TOSA_TMP  = 0x11,
  CMD_DEBUG_CAL_TOSA      = 0x12,
  CMD_DEBUG_GET_TOSA_VAL  = 0x13,
  CMD_DEBUG_CAL_RX_PD     = 0x14,
  CMD_DEBUG_GET_PD        = 0x15,
  CMD_DEBUG_SET_LP        = 0x16,
  CMD_DEBUG_GET_SW_CHAN   = 0x17,
  CMD_DEBUG_RESET_ALARM   = 0x18,
  CMD_DEBUG_INTER_EXP     = 0xFF,
} CmdDebugId;

typedef enum {
  FW_HEAD_MODULE_NAME    = 0x00,
  FW_HEAD_FW_LENGTH      = 0xC0,
  FW_HEAD_CRC            = 0xC4,
  FW_HEAD_END            = 0xFF,
  FW_FILE_HEADER_LENGTH,
} FwFileHeader;

int8_t cmd_get_sn(uint8_t argc, char **argv);
int8_t cmd_get_date(uint8_t argc, char **argv);
int8_t cmd_get_pn(uint8_t argc, char **argv);
int8_t cmd_get_vendor(uint8_t argc, char **argv);
int8_t cmd_version(uint8_t argc, char **argv);
int8_t cmd_switch(uint8_t argc, char **argv);
int8_t set_switch(uint8_t argc, char **argv);
int8_t get_switch(uint8_t argc, char **argv);
int8_t cmd_temp(uint8_t argc, char **argv);
int8_t cmd_IL(uint8_t argc, char **argv);
int8_t cmd_tosa(uint8_t argc, char **argv);
int8_t set_tosa(uint8_t argc, char **argv);
int8_t get_tosa(uint8_t argc, char **argv);
int8_t get_tosa_thr(uint8_t argc, char **argv);
int8_t cmd_rx_pd_cali(uint8_t argc, char **argv);
int8_t cmd_tap_pd_cali(uint8_t argc, char **argv);
int8_t cmd_voltage(uint8_t argc, char **argv);
int8_t get_voltage(void);
int8_t cmd_loopback(uint8_t argc, char **argv);
int8_t loopback_test(void);
int8_t get_loopback_status(void);
int8_t cmd_alarm(uint8_t argc, char **argv);
int8_t cmd_modulation(uint8_t argc, char **argv);
int8_t cmd_device_status(uint8_t argc, char **argv);
int8_t cmd_history_alarm(uint8_t argc, char **argv);
int8_t cmd_upgrade(uint8_t argc, char **argv);
int8_t upgrade_init(void);
int8_t upgrade_file(uint8_t verify);
int8_t upgrade_install(void);
int8_t cmd_reset(uint8_t argc, char **argv);
int8_t cmd_time(uint8_t argc, char **argv);
int8_t set_log_time(uint8_t argc, char **argv);
int8_t get_log_time(void);
int8_t cmd_performance(uint8_t argc, char **argv);
int8_t cmd_threshold(uint8_t argc, char **argv);
int8_t set_threshold(uint8_t argc, char **argv);
int8_t get_threshold(uint8_t argc, char **argv);
int8_t cmd_log(uint8_t argc, char **argv);
int8_t log_size(uint8_t argc, char **argv);
int8_t log_content(uint8_t argc, char **argv);

int8_t cmd_for_debug(uint8_t argc, char **argv);
int8_t debug_dac(uint8_t argc, char **argv);
int8_t debug_adc(uint8_t argc, char **argv);
int8_t debug_tag(uint8_t argc, char **argv);
int8_t debug_cal(uint8_t argc, char **argv);
int8_t debug_get_tosa_val(uint8_t argc, char **argv);
int8_t debug_dump(uint8_t argc, char **argv);
int8_t debug_eeprom(uint8_t argc, char **argv);
int8_t debug_reset_log(uint8_t argc, char **argv);
int8_t debug_reset_fw(uint8_t argc, char **argv);
int8_t debug_monitor(uint8_t argc, char **argv);
int8_t debug_print_hex(uint8_t argc, char **argv);
int8_t debug_pin(uint8_t argc, char **argv);
int8_t debug_switch_io(uint8_t argc, char **argv);
int8_t debug_test_tosa(uint8_t argc, char **argv);
int8_t debug_set_tosa(uint8_t argc, char **argv);
int8_t debug_set_tec(uint8_t argc, char **argv);
int8_t debug_set_tim(uint8_t argc, char **argv);
int8_t debug_get_tmp_all(uint8_t argc, char **argv);
int8_t debug_set_freq(uint8_t argc, char **argv);
int8_t debug_spi(uint8_t argc, char **argv);
int8_t debug_get_pd(uint8_t argc, char **argv);
int8_t debug_set_lp(uint8_t argc, char **argv);
int8_t debug_get_switch_channel(void);
int8_t debug_reset_alarm(uint8_t argc, char **argv);
int8_t debug_get_inter_exp(void);
int8_t debug_send_hex(uint8_t argc, char **argv);

int8_t process_command(uint16_t cmd, uint8_t *pdata, uint8_t len, uint8_t *rx_buf, uint8_t *rx_len);
uint8_t Cal_Check(uint8_t *pdata, uint32_t len);
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);
uint32_t Cal_CRC32(uint8_t* packet, uint32_t length);
void PRINT_HEX(char *head, uint8_t *pdata, uint32_t len);
void PRINT_CHAR(char *head, uint8_t *pdata, uint32_t len);
uint32_t switch_endian(uint32_t i);
uint16_t switch_endian_16(uint16_t i);
void BE32_To_Buffer(uint32_t data, uint8_t *pbuf);
uint32_t Buffer_To_BE32(uint8_t *pbuf);
void BE16_To_Buffer(uint16_t data, uint8_t *pbuf);
uint16_t Buffer_To_BE16(uint8_t *pbuf);

#endif
