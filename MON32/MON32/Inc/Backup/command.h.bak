#ifndef __command_H
#define __command_H

#include "main.h"
#include "cmsis_os.h"

extern char pn[];
extern char hw_version[];
extern char *fw_version;

#define TAG_MAX_SPACE               0x10

#define TRANS_START_BYTE            0x55
#define TRANS_MAX_LENGTH            256
#define TRANS_WAIT_HEAD_TIME        20    //ms
#define TRANS_WAIT_TIME             200   //ms
#define COMMAND_PROCESS_TIME        900   //ms

// Respond Code
#define  RESPOND_SUCCESS            0x00
#define  RESPOND_FAILURE            0x01
#define  RESPOND_CHECK_ERR          0x02
#define  RESPOND_INVALID_PARA       0x03
#define  RESPOND_UNKNOWN_CMD        0x04

#define FILL_RESP_MSG(c, s, l)        do {\
                                        resp_buf.cmd = c;\
                                        resp_buf.status = s;\
                                        resp_buf.length = l;\
                                      } while(0)

typedef struct {
  uint8_t stage;
  uint8_t length;
  uint8_t buf[TRANS_MAX_LENGTH + 1];
} TransStu;

typedef enum {
  TRANS_WAIT_START,
  TRANS_WAIT_HEADER,
  TRANS_RECV_DATA,
} TransStage;

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
  CMD_DEBUG_SET_SW_ADC    = 0x19,
  CMD_DEBUG_CHECK_CALI    = 0x33,
  CMD_DEBUG_CAL_DEF_TEMP  = 0x34,
  CMD_DEBUG_INTER_EXP     = 0xFF,
  CMD_DEBUG_UNLOCK        = 0x12345678,
} CmdDebugId;

typedef enum {
  CMD_SEQ_MSG_LENGTH      = 0x01,
  CMD_SEQ_MSG_ID          = 0x02,
  CMD_SEQ_MSG_DATA        = 0x04,
} CmdSeq;

typedef struct {
  uint16_t cmd;
  uint8_t status;
  uint8_t length;
  uint8_t buf[TRANS_MAX_LENGTH + 1];
} RespondStu;

typedef uint8_t (*cmdFunc)(void);

typedef struct {
  uint16_t cmd_id;
  uint16_t cmd_std_len;
  cmdFunc func;
}CmdStruct;

typedef enum {
  FW_HEAD_MODULE_NAME    = 0x00,
	FW_HEAD_MODULE_PN      = 0x20,
	FW_HEAD_MODULE_HW      = 0x40,
  FW_HEAD_FW_LENGTH      = 0xC0,
  FW_HEAD_CRC            = 0xC4,
  FW_HEAD_END            = 0xFF,
  FW_HEAD_HEADER_LENGTH,
} FwFileHeader;

typedef struct {
  uint8_t run;
  uint8_t is_erasing;
  
  uint32_t upgrade_addr;
  uint32_t upgrade_sector;
  
  uint8_t pre_state;
  uint32_t pre_seq;
  
  uint32_t crc32;
  uint32_t fw_size;
  uint32_t block_size;
  uint32_t recvd_length;
} UpgradeStruct;

typedef enum {
  RUN_MODE_APPLICATION = 0x0,
  RUN_MODE_UPGRADE = 0x1,
} RunMode;

typedef enum {
  UPGRADE_UNUSABLE       = 0x00,
  UPGRADE_RESET          = 0x01,
  UPGRADE_FAILURE        = 0x02,
  UPGRADE_SUCCESS        = 0x03,
} UpgradeState;

typedef enum {
  TX_SWITCH_CHANNEL      = 0x0,
  RX_SWITCH_CHANNEL      = 0x1,
} SWITCH_CHANNEL;

uint8_t Cmd_Process(void);
int Uart_Respond(uint16_t cmd, uint8_t status, uint8_t *pdata, uint8_t len);

uint8_t Cmd_Query_SN(void);
uint8_t Cmd_Query_MDate(void);
uint8_t Cmd_Query_PN(void);
uint8_t Cmd_Query_Vendor(void);
uint8_t Cmd_Get_Version(void);
uint8_t Cmd_Set_Switch(void);
uint8_t Cmd_Get_Switch(void);
uint8_t Cmd_Get_Temperature(void);
uint8_t Cmd_Get_IL(void);
uint8_t Cmd_Query_Tosa_Thr(void);
uint8_t Cmd_Set_Tosa(void);
uint8_t Cmd_Query_Tosa(void);
uint8_t Cmd_RX_PD_CALI(void);
uint8_t Cmd_TAP_PD_CALI(void);
uint8_t Cmd_Voltage(void);
uint8_t Cmd_Loopback_Test(void);
uint8_t Cmd_Loopback_Result(void);
uint8_t Cmd_Query_Alarm(void);
uint8_t Cmd_Set_Modulation(void);
uint8_t Cmd_Query_Modulation(void);
uint8_t Cmd_Query_Status(void);
uint8_t Cmd_Query_Alarm_History(void);
uint8_t Cmd_Upgrade_Init(void);
uint8_t Cmd_Upgrade_Data(void);
uint8_t Cmd_Upgrade_Install(void);
uint8_t Cmd_Softreset(void);
uint8_t Cmd_Set_Time(void);
uint8_t Cmd_Get_Time(void);
uint8_t Cmd_Performance(void);
uint8_t Cmd_Set_Threshold(void);
uint8_t Cmd_Query_Threshold(void);
uint8_t Cmd_LOG_Size(void);
uint8_t Cmd_LOG_Content(void);
uint8_t Cmd_For_Debug(void);

#endif
