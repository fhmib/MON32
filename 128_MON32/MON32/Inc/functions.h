#ifndef __functions_H
#define __functions_H

#include "main.h"

typedef struct {
  uint8_t type;
  void *pbuf;
  uint32_t length;
} MsgStruct;

typedef enum {
  MSG_TYPE_ERROR_LOG,
  MSG_TYPE_NORMAL_LOG,
  MSG_TYPE_FLASH_ISR,
  MSG_TYPE_SWITCH1_ISR,
  MSG_TYPE_SWITCH2_ISR,
  MSG_TYPE_SWITCH_DAC_ISR,
  MSG_TYPE_SELF_CHECK_SWITCH,
  MSG_TYPE_SELF_TEST,
  MSG_TYPE_SELF_TEST_STEP_2,
  MSG_TYPE_CMD_PROCESS,
  MSG_TYPE_LAZER_ENABLE,
  MSG_TYPE_LAZER_DISABLE,
  MSG_TYPE_MODULATION_ON,
  MSG_TYPE_MODULATION_OFF,
} MsgType;

// For Log File
typedef struct {
  uint32_t magic;
  uint32_t magic_2;
  uint32_t error_header;
  uint32_t error_offset;
  uint8_t error_cur_sector;
  uint32_t normal_header;
  uint32_t normal_offset;
  uint8_t normal_cur_sector;
} LogFileState;

typedef struct {
  uint32_t magic;
  uint8_t run;
  uint8_t flash_empty;
  uint32_t length;
  uint32_t crc32;
  uint32_t factory_length;
  uint32_t factory_crc32;
} UpgradeFlashState;

typedef enum {
  // tag
  EE_TAG_SN               = 0x0000,
  EE_TAG_DATE             = 0x0010,
  EE_TAG_PN               = 0x0020,
  EE_TAG_VENDOR           = 0x0030,
  EE_TAG_HW_VERSION       = 0x0040,
  // calibration
  EE_CAL_SWITCH1          = 0x1000,
  EE_CAL_SWITCH2          = 0x1030,
  EE_CAL_SWITCH3          = 0x1090,
  EE_CAL_SWITCH4          = 0x10F0,
  EE_CAL_SWITCH5          = 0x1150,
  EE_CAL_SWITCH6          = 0x1170,
  EE_CAL_SWITCH7          = 0x11D0,
  EE_CAL_SWITCH8          = 0x1230,
  EE_CAL_TOSA             = 0x1290,
  EE_CAL_TX_IL            = 0x1350,
  EE_CAL_RX_IL            = 0x1450,
  EE_CAL_LB_IL            = 0x14D0,
  EE_CAL_RX_PD            = 0x1500,
  EE_TEC_DEF_TEMP         = 0x1550,
  // Parameter Table end Address
  EE_PARA_TABLE_END       = 0x156F,
  // Alarm
  EE_ALARM_HISTORY        = 0x1570,
  // upgrade
  EE_UP_MAGIC             = 0x3000, // 0x3000 ~ 0x3003
  EE_UP_RUN               = 0x3004, // 0x3004 ~ 0x3004 Which fw is running
  EE_UP_FLASH_EMPTY       = 0x3005, // 0x3005 ~ 0x3005 If flash is empty
  EE_UP_LENGTH            = 0x3006, // 0x3006 ~ 0x3009
  EE_UP_CRC32             = 0x300A, // 0x300A ~ 0x300D
  EE_UP_FACTORY_LENGTH    = 0x300E, // 0x300E ~ 0x3011
  EE_UP_FACTORY_CRC32     = 0x3012, // 0x3012 ~ 0x3015
  // log
  EE_LOG_MAGIC            = 0x3020, // 0x3020 ~ 0x3023
  EE_ERROR_LOG_OFFSET     = 0x3024, // 0x3024 ~ 0x3027
  EE_ERROR_LOG_HEADER     = 0x3028, // 0x3028 ~ 0x302B
  EE_NORMAL_LOG_OFFSET    = 0x302C, // 0x302C ~ 0x302F
  EE_NORMAL_LOG_HEADER    = 0x3030, // 0x3030 ~ 0x3033
  EE_LOG_MAGIC_2          = 0x303C, // 0x303C ~ 0x303F
  // Alarm
  EE_ALARM_MAGIC          = 0x3040,
  EE_ALARM_START          = 0x3044,
  EE_ALARM_END            = 0x3045,
  // Calibration CRC Check
  EE_CALI_CHECK           = 0x3048,
} EepromAddrMap;

typedef enum {
  SWITCH_ALL_CHANNEL = 0,
  SWITCH_NUM_1 = 1,
  SWITCH_NUM_2 = 2,
  SWITCH_NUM_3 = 3,
  SWITCH_NUM_4 = 4,
  SWITCH_NUM_5 = 5,
  SWITCH_NUM_6 = 6,
  SWITCH_NUM_7 = 7,
  SWITCH_NUM_8 = 8,
  SWITCH_NUM_TOTAL,
} SwitchNumber;

typedef struct {
  uint8_t sw_num;
  uint8_t px_chan; // dac channel correspond to positive x
  uint8_t nx_chan; // dac channel correspond to negatve x
  uint8_t py_chan; // dac channel correspond to positive y
  uint8_t ny_chan; // dac channel correspond to negatve y
  uint8_t adc_px_chan; // adc channel correspond to positive x
  uint8_t adc_nx_chan; // adc channel correspond to negatve x
  uint8_t adc_py_chan; // adc channel correspond to positive y
  uint8_t adc_ny_chan; // adc channel correspond to negatve y
} SwitchDacMapStruct;

typedef struct {
  uint8_t switch_number;
  uint8_t min_channel_num;
  uint8_t max_channel_num;
  uint8_t first_switch;
  uint16_t first_eeprom_addr;
  uint8_t second_switch;
  uint16_t second_eeprom_addr;
} ChannelSwitchMapStruct;

typedef enum {
  TAP_PD_CHANNEL      = 0,
  RX_PD_CHANNEL       = 1,
  VOLTAGE_3_3_CHANNEL = 2,
  VOLTAGE_4_4_CHANNEL = 3,
  VOLTAGE_5_0_CHANNEL = 4,
  VOLTAGE_61_0_CHANNEL = 5,
  TEMPERATURE_CHANNEL = 6,
} VoltageAdcChannel;

typedef enum {
  TEC_ADC_TEC_TEMP_CHANNEL    = 0,
  TEC_ADC_TEC_VOLTAGE_CHANNEL = 1,
  TEC_ADC_TEC_CURRENT_CHANNEL = 2,
  TEC_ADC_LD_CURRENT_CHANNEL  = 3,
  TEC_ADC_LD_VOLTAGE_CHANNEL  = 4,
  TEC_ADC_MPD_CURRENT_CHANNEL = 5,
} TecAdcChannel;

typedef enum {
  DAC128S085_TEC_VALUE_CHANNEL = 0,
  DAC128S085_TEC_SWITCH_CHANNEL = 1,
  DAC128S085_TOSA_SWITCH_CHANNEL = 2,
} DAC128S085Channel;

typedef enum {
  EXP_SELFCHECK           =    0,
  EXP_TEMPERATURE         =    1,
  EXP_VOLTAGE_61_0        =    4,
  EXP_VOLTAGE_4_4         =    5,
  EXP_VOLTAGE_3_3         =    6,
  EXP_VOLTAGE_5_0         =    7,
  EXP_LAZER_POWER         =    8,
  EXP_SWICH_VOLTAGE4      =    9,
  EXP_SWICH_VOLTAGE3      =   10,
  EXP_SWICH_VOLTAGE2      =   11,
  EXP_SWICH_VOLTAGE1      =   12,
  EXP_DAC                 =   13,
  EXP_ADC_2               =   14,
  EXP_ADC_1               =   15,
  EXP_SWITCH              =   17,
  EXP_RX_PD               =   18,
  EXP_LAZER_CURRENT       =   21,
  EXP_LAZER_AGING         =   22,
  EXP_TEC_TEMP_LOSS       =   23,
  EXP_TEC_VOLTAGE         =   28,
  EXP_TEC_CURRENT         =   29,
  EXP_TEC_TEMP            =   30,
  EXP_CALI_CHK            =   31,
} ExptoinValue;

typedef enum {
  INT_EXP_UP_ERASE        =  0,
  INT_EXP_UP_PROGRAM      =  1,
  INT_EXP_LOG_ERASE       =  2,
  INT_EXP_LOG_PROGRAM     =  3,
  INT_EXP_INIT            =  4,
  INT_EXP_OS_ERR          =  5,
  // INT_EXP_TMPGD           =  6,
  INT_EXP_UP_ALARM        =  7,
  INT_EXP_LOG_TASK        =  8,
  INT_EXP_TAP_PD          =  9,
  INT_EXP_RX_PD           = 10,
  INT_EXP_TOSA            = 11,
  INT_EXP_TOSA_DATA       = 12,
  INT_EXP_CONST           = 31,
} InternalExptoinValue;

#define TOSA_TABLE_COUNT  10
typedef struct {
  uint32_t tosa_dac;
  uint32_t tec_dac;
  uint32_t tap_adc;
  double tap_power;
} TosaCalData;

typedef struct {
  double vol_3_3_high_alarm;
  double vol_3_3_high_clear;
  double vol_3_3_low_alarm;
  double vol_3_3_low_clear;

  double vol_4_4_high_alarm;
  double vol_4_4_high_clear;
  double vol_4_4_low_alarm;
  double vol_4_4_low_clear;

  double vol_5_0_high_alarm;
  double vol_5_0_high_clear;
  double vol_5_0_low_alarm;
  double vol_5_0_low_clear;

  double vol_61_0_high_alarm;
  double vol_61_0_high_clear;
  double vol_61_0_low_alarm;
  double vol_61_0_low_clear;

  double temp_high_alarm;
  double temp_high_clear;
  double temp_low_alarm;
  double temp_low_clear;

  double tec_cur_high_alarm;
  double tec_cur_high_clear;
  double tec_cur_low_alarm;
  double tec_cur_low_clear;

  double tec_vol_high_alarm;
  double tec_vol_high_clear;
  double tec_vol_low_alarm;
  double tec_vol_low_clear;

  double tec_temp_high_alarm;
  double tec_temp_high_clear;
  double tec_temp_low_alarm;
  double tec_temp_low_clear;

  double LD_cur_high_alarm;
  double LD_cur_high_clear;
} ThresholdStruct;

typedef enum {
  OSC_SUCCESS = 0,
  OSC_FAILURE = 1,
  OSC_ONGOING = 2,
} OpticalSelfCheckStatus;

typedef struct {
  uint32_t maigc;
  uint8_t uart_reset; // Indicate if reset by uart communication
  uint8_t tx_switch_channel; // 0-31: 1 to 1-32   32-63: 2 to 1-32   64-65: 1-2 to 33   0xFF: none
  uint8_t rx_switch_channel; // 0-31: 1 to 1-32   32: 1 to 33   0xFF: none
  uint8_t tx_block;
  uint8_t tosa_enable;
  uint8_t lazer_ready;
  uint8_t osc_status; // Optical path self-check status 0:OK 1:FAIL 2:ongoing
  uint8_t modulation;
  uint8_t pre_alarm;
  uint32_t pre_exp_value;
  uint32_t exp; // exception
  uint32_t internal_exp;
  TosaCalData tosa_low;
  TosaCalData tosa_high;
  double tosa_dst_power_low;
  double tosa_dst_power_high;
  ThresholdStruct thr_table;
  uint8_t allow_monitor;
  uint8_t power_mode;
  uint16_t sw_adc_int;
  double sw_adc_double;
} RunTimeStatus;

// For Alarm
typedef struct {
  uint32_t magic;
  uint8_t start;
  uint8_t end;
} AlarmHistoryState;

typedef struct {
  uint32_t counter;
  uint32_t time; // Unit: 100us
  int32_t step;
  uint8_t sw_num;
  int32_t dst_x;
  int32_t dst_y;
  int32_t cur_x;
  int32_t cur_y;
} SwTimControl;

void Throw_Log(uint8_t type, uint8_t *buf, uint32_t length);
uint32_t Log_Write(uint32_t addr, uint8_t *pbuf, uint32_t length);
uint32_t Log_Write_byte(uint32_t addr, uint8_t ch, uint32_t length);
uint32_t Log_Read(uint32_t addr, uint8_t *pbuf, uint32_t length);
osStatus_t Get_Log_Status(LogFileState *log_status);
osStatus_t Update_Log_Status(LogFileState *log_status);
osStatus_t Reset_Log_Status(void);


osStatus_t Get_Up_Status(UpgradeFlashState *status);
osStatus_t Update_Up_Status(UpgradeFlashState *status);
osStatus_t Reset_Up_Status(void);
osStatus_t Get_Threshold_Table(ThresholdStruct *table);
osStatus_t Update_Tec_Dest_Temp(ThresholdStruct *table);
osStatus_t Reset_Tec_Dest_Temp(ThresholdStruct *table);
void Check_Cali(void);

osStatus_t Get_EEPROM_Alarm_Status(AlarmHistoryState *alarm);
osStatus_t Update_EEPROM_Alarm_Status(AlarmHistoryState *alarm);
osStatus_t Reset_EEPROM_Alarm_Status(void);
osStatus_t Update_History_Alarm(uint32_t exp);

uint8_t Get_Switch_Position_By_IO(uint8_t switch_channel);
int8_t Set_Switch(uint8_t switch_channel, uint8_t switch_pos);
int8_t Get_Current_Switch_Channel(uint8_t switch_channel);
void Set_Switch_Alarm(uint8_t sw_num);
void Clear_Switch_Alarm(uint8_t sw_num);
int8_t Get_Current_Switch_ADC(uint8_t switch_channel, int16_t *x1, int16_t *y1, int16_t *x2, int16_t *y2);
int8_t Clear_Switch_Dac(uint32_t switch_id);
int8_t Get_Switch_Adc(uint32_t switch_id, uint16_t *px, uint16_t *nx, uint16_t *py, uint16_t *ny);
int32_t Get_Index_Of_Channel_Map(uint8_t switch_channel, uint8_t switch_pos);
void Reset_Switch(uint8_t switch_channel);
void Reset_Switch_Only(uint8_t switch_channel);

void Set_Alarm(void);
void Clear_Alarm(void);
void Set_Lazer_Ready(void);
void Clear_Lazer_Ready(void);
void Init_Run_Status(void);
void Set_Switch_Ready(uint8_t switch_channel);
void Clear_Switch_Ready(uint8_t switch_channel);
void Set_Flag(uint32_t *status, uint32_t bit_addr);
void Clear_Flag(uint32_t *status, uint32_t bit_addr);
uint8_t Is_Flag_Set(uint32_t *status, uint32_t bit_addr);

double Cal_Temp(uint16_t adc);
double Cal_Tosa_Temp(uint16_t adc);

uint8_t Enable_Tosa(void);
uint8_t Disable_Tosa(void);
void update_tosa_table(void);
TosaCalData Get_Tosa_Data(double power);
TosaCalData Cal_Tosa_Data(TosaCalData x1, TosaCalData x2, double power);
uint8_t Get_Tap_Power(double *cur_power);
uint8_t Get_Rx_Power(double *cur_power);
uint8_t Is_Tec_Lock(void);
uint8_t Cali_Power(double power_dst);

uint8_t cal_tap_pd_by_power(uint16_t *adc, double power);
uint8_t get_tap_pd_power(uint16_t *adc, double *power);
uint8_t get_rx_pd_power(uint16_t *adc, double *power);

uint8_t Get_Performance(uint8_t per_id, uint8_t *pBuf);
uint8_t Set_Threshold(uint8_t alarm_id, int32_t val32_low, int32_t val32_high);
uint8_t Get_Threshold(uint8_t alarm_id, uint8_t *pBuf);

uint8_t debug_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y);
int8_t set_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y);
void set_sw_dac_2(uint8_t sw_num, int32_t val_x, int32_t val_y);
uint8_t debug_sw_adc(uint8_t sw_num);
uint8_t debug_vol_adc(uint8_t chan);
uint8_t debug_tag(uint8_t type, uint8_t *p, uint32_t length);
uint8_t debug_pin(uint8_t pin, uint8_t val);
uint8_t debug_cal_switch(uint8_t sw_num, uint32_t chan, int32_t val_x, int32_t val_y);
uint8_t debug_cal_tosa(uint8_t num, uint32_t tosa_dac, uint32_t tec_dac, uint32_t pd_adc, int32_t pd);
uint8_t debug_get_tosa_val(int32_t val, uint32_t *resp_len);
uint8_t debug_cal_il(uint8_t num, int32_t val);
uint8_t debug_cal_default_temp(int32_t val);
uint8_t debug_cal_rx_pd(uint8_t num, uint32_t adc, int32_t val);
uint8_t debug_cal_dump(uint32_t which, uint32_t *resp_len);
uint8_t debug_eeprom(uint32_t addr, uint32_t *len);
uint8_t debug_test_tosa(void);
uint8_t debug_set_tosa(uint16_t low, uint16_t high);
uint8_t debug_set_tec(uint16_t value);
uint8_t debug_get_tosa_tmp(void);
uint8_t debug_get_pd(uint32_t which);
uint8_t debug_set_lp(uint32_t which);
uint8_t debug_get_switch_channel(uint8_t switch_channel);
uint8_t debug_get_inter_exp(void);
uint8_t debug_Cmd_Set_Time(uint8_t *buf);
uint8_t debug_Cmd_Get_Time(void);
uint8_t debug_Cmd_LOG_Size(uint8_t *buf);
uint8_t debug_Cmd_LOG_Content(uint8_t *buf, uint32_t *rLen);
uint8_t debug_Cmd_Query_Alarm_History(void);
uint8_t debug_Cmd_Check_Cali(void);

void write_cali_to_e2(void);

#endif
