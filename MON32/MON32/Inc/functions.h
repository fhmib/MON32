#ifndef __functions_H
#define __functions_H

#include "main.h"

typedef struct {
  uint8_t type;
  void *pbuf;
  uint32_t length;
} MsgStruct;

typedef enum {
  MSG_TYPE_LOG,
  MSG_TYPE_FLASH_ISR,
  MSG_TYPE_SWITCH1_ISR,
  MSG_TYPE_SWITCH2_ISR,
} MsgType;

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
#if 0
  // threshold
  EE_VOLTAGE_2_5_THR      = 0x1460,
  EE_VOLTAGE_3_3_THR      = 0x1468,
  EE_VOLTAGE_5_0_THR      = 0x1470,
  EE_VOLTAGE_64_0_THR     = 0x1478,
  EE_TEMPERATURE_THR      = 0x1480,
#endif
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
  EE_LOG_OFFSET           = 0x3024, // 0x3024 ~ 0x3027
  EE_LOG_HEADER           = 0x3028, // 0x3028 ~ 0x302B
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
  INT_EXP_UP_ERASE        =  0,
  INT_EXP_UP_PROGRAM      =  1,
  INT_EXP_LOG_ERASE       =  2,
  INT_EXP_LOG_PROGRAM     =  3,
  INT_EXP_INIT            =  4,
  INT_EXP_OS_ERR          =  5,
  INT_EXP_TMPGD           =  6,
  INT_EXP_CONST           = 31,
} InternalExptoinValue;

#define TOSA_TABLE_COUNT  10
typedef struct {
  uint16_t tosa_dac;
  uint16_t tec_dac;
  uint16_t tap_adc;
  double tap_power;
} TosaCalData;

typedef struct {
  uint32_t maigc;
  uint8_t uart_reset; // Indicate if reset by uart communication
  uint8_t tx_switch_channel; // 0-31: 1 to 1-32   32-63: 2 to 1-32   64-65: 1-2 to 33   0xFF: none
  uint8_t rx_switch_channel; // 0-31: 1 to 1-32   32: 1 to 33   0xFF: none
  uint8_t tx_block;
  uint8_t tosa_enable;
  uint8_t modulation;
  uint32_t exp; // exception
  uint32_t internal_exp;
  TosaCalData tosa_low;
  TosaCalData tosa_high;
} RunTimeStatus;


osStatus_t Get_Up_Status(UpgradeFlashState *status);
osStatus_t Update_Up_Status(UpgradeFlashState *status);
osStatus_t Reset_Up_Status(void);

uint8_t Get_Switch_Position_By_IO(uint8_t switch_channel);
int8_t Set_Switch(uint8_t switch_channel, uint8_t switch_pos);
int8_t Get_Current_Switch_Channel(uint8_t switch_channel);
int8_t Clear_Switch_Dac(uint32_t switch_id);
int8_t Get_Switch_Adc(uint32_t switch_id, uint16_t *px, uint16_t *nx, uint16_t *py, uint16_t *ny);
int32_t Get_Index_Of_Channel_Map(uint8_t switch_channel, uint8_t switch_pos);
void Reset_Switch(uint8_t switch_channel);
void Reset_Switch_Only(uint8_t switch_channel);

void Set_Alarm(void);
void Clear_Alarm(void);
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

uint8_t get_tap_pd_power(uint16_t *adc, double *power);
uint8_t get_rx_pd_power(uint16_t *adc, double *power);

uint8_t debug_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y);
int8_t set_sw_dac(uint8_t sw_num, int32_t val_x, int32_t val_y);
uint8_t debug_sw_adc(uint8_t sw_num);
uint8_t debug_vol_adc(uint8_t chan);
uint8_t debug_tag(uint8_t type, uint8_t *p, uint32_t length);
uint8_t debug_pin(uint8_t pin, uint8_t val);
uint8_t debug_cal_switch(uint8_t sw_num, uint32_t chan, int32_t val_x, int32_t val_y);
uint8_t debug_cal_tosa(uint8_t num, uint32_t tosa_dac, uint32_t tec_dac, uint32_t pd_adc, int32_t pd);
uint8_t debug_get_tosa_val(int32_t val, uint32_t *resp_len);
uint8_t debug_cal_il(uint8_t num, int32_t val);
uint8_t debug_cal_rx_pd(uint8_t num, uint32_t adc, int32_t val);
uint8_t debug_cal_dump(uint32_t which, uint32_t *resp_len);
uint8_t debug_test_tosa(void);
uint8_t debug_set_tosa(uint16_t low, uint16_t high);
uint8_t debug_set_tec(uint16_t value);
uint8_t debug_get_tosa_tmp(void);
uint8_t debug_get_pd(uint32_t which);
uint8_t debug_get_inter_exp(void);


#endif
