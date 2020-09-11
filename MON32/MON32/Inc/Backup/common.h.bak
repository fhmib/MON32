#ifndef __common_H
#define __common_H

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define MY_MIN(a,b)                ((a)<(b)?(a):(b))

#define I2C_WAIT_TIMEOUT          500   //ms

#undef PRINT_DEBUG_MESSAGE

#define DEBUG_BUF_LEN 1024

#ifdef PRINT_DEBUG_MESSAGE
#define EPT(format, ...)	do{\
                      sprintf((char*)debug_buf, "%s,%d: " format, __func__, __LINE__, ##__VA_ARGS__);\
                      HAL_UART_Transmit(&huart3, debug_buf, strlen((char*)debug_buf), 0xFF);\
                    } while(0)
#define PRINT(format, ...)	do{\
                      sprintf((char*)debug_buf, format, ##__VA_ARGS__);\
                      HAL_UART_Transmit(&huart3, debug_buf, strlen((char*)debug_buf), 0xFF);\
                    } while(0)
#else
#define EPT(format, ...)	do{\
                    } while(0)
#define PRINT(format, ...)	do{\
                    } while(0)
#endif
#define THROW_LOG(format, ...) do {\
                    } while(0)

extern uint8_t debug_buf[];
                    
typedef enum {
  DAC128S085_MODE_NORMAL = 0x0,
  DAC128S085_MODE_WRM = 0x1,
  DAC128S085_MODE_WTM = 0x2,
} DAC128S085_MODE;

uint32_t switch_endian(uint32_t i);
uint16_t switch_endian_16(uint16_t i);
uint32_t my_abs(int32_t val);
void BE32_To_Buffer(uint32_t data, uint8_t *pbuf);
uint32_t Buffer_To_BE32(uint8_t *pbuf);
void BE16_To_Buffer(uint16_t data, uint8_t *pbuf);
uint16_t Buffer_To_BE16(uint8_t *pbuf);
uint8_t Is_Value_Approximate(int32_t src, int32_t dst, double factor);

void PRINT_HEX(char *head, uint8_t *pdata, uint32_t len);

uint8_t Cal_Check(uint8_t *pdata, uint32_t len);
uint32_t Cal_CRC32(uint8_t* packet, uint32_t length);

osStatus_t get_8_from_eeprom(int16_t bus_addr, uint16_t addr, uint8_t *pval);
osStatus_t write_8_to_eeprom(int16_t bus_addr, uint16_t addr, uint8_t data);
osStatus_t get_32_from_eeprom(int16_t bus_addr, uint16_t addr, uint32_t *pval);
osStatus_t write_32_to_eeprom(int16_t bus_addr, uint16_t addr, uint32_t data);
void write_byte_to_eeprom(int16_t bus_addr, uint16_t mem_addr, uint8_t byte, uint32_t count);
osStatus_t RTOS_EEPROM_Write(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length);
osStatus_t RTOS_EEPROM_Read(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length);
HAL_StatusTypeDef I2cEepromWrite(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length);
HAL_StatusTypeDef I2cEepromRead(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length);
osStatus_t RTOS_DAC5535_Write(uint8_t chan, uint16_t val);
osStatus_t RTOS_DAC128S085_Write(uint8_t chan, uint16_t val, uint8_t mode);
void DAC5541_Write(uint16_t val);
osStatus_t RTOS_SWITCH_ADC7953_Read(uint8_t chan, uint16_t *val);
osStatus_t RTOS_ADC7953_SPI4_Read(uint8_t chan, uint16_t *val);
osStatus_t RTOS_ADC7953_SPI5_Read(uint8_t chan, uint16_t *val);
osStatus_t RTOS_ADC7953_SPI6_Read(uint8_t chan, uint16_t *val);
osStatus_t RTOS_ADC7828_Read(uint8_t chan, uint16_t *val);

#if 0
#define THROW_LOG(format, ...) do {\
                      sprintf((char*)debug_buf, format, ##__VA_ARGS__);\
                      Throw_Log(debug_buf, strlen((char*)debug_buf));\
                    } while(0)

uint32_t my_abs(int32_t val);
uint8_t Is_Value_Approximate(int32_t src, int32_t dst, double factor);

uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);


osStatus_t RTOS_ADC7828_Read(uint8_t chan, uint16_t *val);
#endif
#endif
