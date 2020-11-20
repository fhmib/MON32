#include "common.h"
#include "i2c.h"
#include "spi.h"
#include "cmsis_os.h"

#define EEPROM_PAGESIZE 64

uint8_t debug_buf[DEBUG_BUF_LEN] = {0}; // for print debug info

extern osMutexId_t i2c1Mutex;
extern osMutexId_t i2c2Mutex;
extern osMutexId_t spi2Mutex;
extern osMutexId_t spi3Mutex;
extern osMutexId_t spi4Mutex;
extern osMutexId_t spi5Mutex;
extern osMutexId_t spi6Mutex;

uint32_t switch_endian(uint32_t i)
{
  return (((i>>24)&0xff) | ((i>>8)&0xff00) | ((i<<8)&0xff0000) | ((i<<24)&0xff000000));
}

uint16_t switch_endian_16(uint16_t i)
{
  return (((i>>8)&0xff) | ((i<<8)&0xff00));
}

uint32_t my_abs(int32_t val)
{
  char buf[32];
  uint32_t p_val;
  
  sprintf(buf, "%d", val);
  if (buf[0] == '-') {
    p_val = strtoul(buf + 1, NULL, 10);
  } else {
    p_val = val;
  }
  
  return p_val;
}


uint8_t Is_Date_Valid(uint8_t year, uint8_t month, uint8_t date)
{
  uint8_t Month_buf[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  if (month == 2) {
    (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) ? Month_buf[1] += 1 : Month_buf[1];
  }

  if (month > 12 || month < 1 || date > Month_buf[month - 1] || date < 1)
    return 0;

  return 1;
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

uint8_t Is_Value_Approximate(uint32_t src, uint32_t dst)
{
#if 0
  double src_rate = (double)src / 0x1000;
  double dst_rate = (double)dst / 0x4000;
  
  if (src_rate < dst_rate - factor || src_rate > dst_rate + factor) {
    return 0;
  }
  
  return 1;
#else
  double temp;
  uint32_t sub;

  src = src * 4;
  sub = MY_MAX(src, dst) - MY_MIN(src, dst);

  if (dst <= 4000) {
    if (sub > run_status.sw_adc_int)
      return 0;
    else
      return 1;
  } else {
    temp = (double)sub / (double)dst;
    if (temp > run_status.sw_adc_double)
      return 0;
    else
      return 1;
  }
#endif
}

void PRINT_HEX(char *head, uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  
  PRINT("************* PRINT HEX *************\n");
  PRINT("%s:\n", head);
  for (i = 0; i < len; ++i) {
    if (i % 0x10 == 0) {
      PRINT("%08X : ", i / 0x10);
    }
    PRINT("0x%02X%s", pdata[i], (i + 1) % 0x10 == 0 ? "\n" : i == len - 1 ? "\n" : " ");
    osDelay(pdMS_TO_TICKS(10));
  }
  PRINT("************* PRINT END *************\n");
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


osStatus_t get_8_from_eeprom(int16_t bus_addr, uint16_t addr, uint8_t *pval)
{
  return RTOS_EEPROM_Read(bus_addr, addr, pval, 1);
}

osStatus_t write_8_to_eeprom(int16_t bus_addr, uint16_t addr, uint8_t data)
{
  return RTOS_EEPROM_Write(bus_addr, addr, &data, 1);
}

osStatus_t get_32_from_eeprom(int16_t bus_addr, uint16_t addr, uint32_t *pval)
{
  uint32_t val;
  osStatus_t status;
  status = RTOS_EEPROM_Read(bus_addr, addr, (uint8_t*)&val, 4);
  *pval = switch_endian(val);
  return status;
}

osStatus_t write_32_to_eeprom(int16_t bus_addr, uint16_t addr, uint32_t data)
{
  uint32_t val = switch_endian(data);
  return RTOS_EEPROM_Write(bus_addr, addr, (uint8_t*)&val, 4);
}

void write_byte_to_eeprom(int16_t bus_addr, uint16_t addr, uint8_t byte, uint32_t count)
{
  uint32_t offset, every_len;
  unsigned char buf[128];

  memset(buf, byte, 128);
  offset = 0;
  while (offset < count) {
    every_len = (offset + 128 < count) ? 128 : count - offset;
    /*
    Serial.print("Write ");
    Serial.print(every_len, DEC);
    Serial.print(" 0x");
    Serial.print(byte, HEX);
    Serial.print(" to 0x");
    Serial.println(addr + offset, HEX);
    */
    RTOS_EEPROM_Write(bus_addr, addr + offset, buf, every_len);
    offset += every_len;
  }
}

osStatus_t RTOS_EEPROM_Write(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length)
{
  osStatus_t status = osOK;
  HAL_StatusTypeDef ret;

  if ((status = osMutexAcquire(i2c2Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of i2c2 failed\n");
    return status;
  }

  if ((ret = I2cEepromWrite(dev_addr, mem_addr, buf, length)) != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Write EEPROM failed, ret = %u, ErrorCode = %#X\n", ret, hi2c2.ErrorCode);
    if (ret == HAL_ERROR) status = osError;
    else if (ret == HAL_BUSY) status = osErrorResource;
    else if (ret == HAL_TIMEOUT) status = osErrorTimeout;
  }

  osMutexRelease(i2c2Mutex);
  return status;
}

osStatus_t RTOS_EEPROM_Read(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length)
{
  osStatus_t status = osOK;
  HAL_StatusTypeDef ret;

  if ((status = osMutexAcquire(i2c2Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of i2c2 failed\n");
    return status;
  }
  
  if ((ret = I2cEepromRead(dev_addr, mem_addr, buf, length)) != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read EEPROM failed, ret = %u, ErrorCode = %#X\n", ret, hi2c2.ErrorCode);
    if (ret == HAL_ERROR) status = osError;
    else if (ret == HAL_BUSY) status = osErrorResource;
    else if (ret == HAL_TIMEOUT) status = osErrorTimeout;
  }

  osMutexRelease(i2c2Mutex);
  return status;
}

// Not safe
HAL_StatusTypeDef I2cEepromWrite(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length)
{
  int32_t offset = 0, write_length = 0, byte_addr = 0;
  HAL_StatusTypeDef status = HAL_OK;

  HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_RESET);

  while (length > 0) {
    byte_addr = mem_addr % EEPROM_PAGESIZE;
    write_length = MY_MIN(EEPROM_PAGESIZE - byte_addr, length);

    if ((status = HAL_I2C_Mem_Write(&hi2c2, dev_addr, mem_addr, I2C_MEMADD_SIZE_16BIT, buf + offset, write_length, 100)) != HAL_OK) {
      break;
    }

    length -= write_length;  
    offset += write_length;
    mem_addr += write_length;

    osDelay(pdMS_TO_TICKS(10));
  }

  HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_SET);
  
  return status;
}

// Not safe
HAL_StatusTypeDef I2cEepromRead(int16_t dev_addr, uint16_t mem_addr, uint8_t *buf, int32_t length)
{
  return HAL_I2C_Mem_Read(&hi2c2, dev_addr, mem_addr, I2C_MEMADD_SIZE_16BIT, buf, length, 100);
}

osStatus_t RTOS_DAC5535_Write(uint8_t chan, uint16_t val)
{
  osStatus_t status;
  uint8_t buf[3];

  if ((status = osMutexAcquire(spi3Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of spi3 failed\n");
    return status;
  }

  buf[0] = ((chan & 0x1F) << 3) | ((val >> 11) & 0x7);
  buf[1] = val >> 3;
  buf[2] = (val & 0x7) << 5;
  //EPT("Dac value: %#X %#X %#X\n", buf[0], buf[1], buf[2]);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi3, buf, 3, 10) != HAL_OK) {
    EPT("Write DAC5535 failed, ErrorCode = %#X\n", hspi3.ErrorCode);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Write DAC5535 failed, ErrorCode = %#X\n", hspi3.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_DAC)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "DAC5535 abnormal\n");
      Set_Flag(&run_status.exp, EXP_DAC);
    }
    status = osError;
  }

  osMutexRelease(spi3Mutex);

  return status;
}

osStatus_t RTOS_DAC128S085_Write(uint8_t chan, uint16_t val, uint8_t mode)
{
  osStatus_t status;
  uint8_t buf[2];

  if ((status = osMutexAcquire(spi2Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of spi2 failed\n");
    return status;
  }

  if (mode == DAC128S085_MODE_NORMAL) {
    buf[0] = (uint8_t)((chan & 0x7) << 4) | ((val >> 8) & 0xF);
    buf[1] = (uint8_t)(val);
  } else if (mode == DAC128S085_MODE_WRM) {
    buf[0] = (uint8_t)(0x8 << 4);
    buf[1] = 0;
  } else if (mode == DAC128S085_MODE_WTM) {
    buf[0] = (uint8_t)(0x9 << 4);
    buf[1] = 0;
  }

  EPT("Dac128S085 value: %#X %#X\n", buf[0], buf[1]);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi2, buf, 2, 10) != HAL_OK) {
    EPT("Write DAC128S085 failed, ErrorCode = %#X\n", hspi2.ErrorCode);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Write DAC128S085 failed, ErrorCode = %#X\n", hspi2.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_DAC)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "DAC128S085 abnormal\n");
      Set_Flag(&run_status.exp, EXP_DAC);
    }
    status = osError;
  }
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  osMutexRelease(spi2Mutex);

  return status;
}

void DAC5541_Write(uint16_t val)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(val >> 8);
  buf[1] = (uint8_t)val;
  //EPT("Dac value: %#X %#X %#X\n", buf[0], buf[1], buf[2]);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, buf, 2, 1) != HAL_OK) {
    EPT("Write DAC5541 failed, ErrorCode = %#X\n", hspi1.ErrorCode);
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Write DAC5541 failed, ErrorCode = %#X\n", hspi1.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_DAC)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "DAC5541 abnormal\n");
      Set_Flag(&run_status.exp, EXP_DAC);
    }
  }
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

osStatus_t RTOS_SWITCH_ADC7953_Read(uint8_t chan, uint16_t *val)
{
  if (chan < 16)
    return RTOS_ADC7953_SPI4_Read(chan, val);
  else
    return RTOS_ADC7953_SPI6_Read(chan - 16, val);
}

osStatus_t RTOS_ADC7953_SPI4_Read(uint8_t chan, uint16_t *val)
{
  osStatus_t status;
  HAL_StatusTypeDef hal_status;
  uint8_t txbuf[2], rxbuf[2], chan_rb;

  if ((status = osMutexAcquire(spi4Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of spi4 failed\n");
    return status;
  }

  txbuf[0] = (0x1 << 4) | (0x0 << 3) | (chan >> 1);
  txbuf[1] = chan << 7;

  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi4, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi4, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi4, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);

  osMutexRelease(spi4Mutex);

  if (hal_status != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI4 failed, ErrorCode = %#X\n", hspi4.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI4 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  }

  if (hal_status != HAL_OK) {
    EPT("Read ADC7953_SPI4 failed, ErrorCode = %#X\n", hspi4.ErrorCode);
    return osError;
  }

  chan_rb = rxbuf[0] >> 4;
  if (chan_rb != chan) {
    EPT("Read ADC7953_SPI4 failed, rChanIdx != chanIdx\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI4 failed, rChanIdx != chanIdx\n");
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI4 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  } else {
    *val = ((rxbuf[0] & 0xf) << 8) | rxbuf[1];
  }

  return status;
}

osStatus_t RTOS_ADC7953_SPI5_Read(uint8_t chan, uint16_t *val)
{
  osStatus_t status;
  HAL_StatusTypeDef hal_status;
  uint8_t txbuf[2], rxbuf[2], chan_rb;

  if ((status = osMutexAcquire(spi5Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of spi5 failed\n");
    return status;
  }

  txbuf[0] = (0x1 << 4) | (0x0 << 3) | (chan >> 1);
  txbuf[1] = chan << 7;

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi5, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

  osMutexRelease(spi5Mutex);

  if (hal_status != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI5 failed, ErrorCode = %#X\n", hspi5.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI5 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  }

  chan_rb = rxbuf[0] >> 4;
  if (chan_rb != chan) {
    EPT("Read ADC7953_SPI5 failed, rChanIdx != chanIdx\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI5 failed, rChanIdx != chanIdx\n");
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI5 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  } else {
    *val = ((rxbuf[0] & 0xf) << 8) | rxbuf[1];
  }

  return status;
}

osStatus_t RTOS_ADC7953_SPI6_Read(uint8_t chan, uint16_t *val)
{
  osStatus_t status;
  HAL_StatusTypeDef hal_status;
  uint8_t txbuf[2], rxbuf[2], chan_rb;

  if ((status = osMutexAcquire(spi6Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of spi6 failed\n");
    return status;
  }

  txbuf[0] = (0x1 << 4) | (0x0 << 3) | (chan >> 1);
  txbuf[1] = chan << 7;

  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi6, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi6, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_SET);
  //osDelay(1);

  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_RESET);
  hal_status |= HAL_SPI_TransmitReceive(&hspi6, txbuf, rxbuf, 2, 50);
  HAL_GPIO_WritePin(SPI6_CS0_GPIO_Port, SPI6_CS0_Pin, GPIO_PIN_SET);

  osMutexRelease(spi6Mutex);

  if (hal_status != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI6 failed, ErrorCode = %#X\n", hspi6.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI6 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  }

  chan_rb = rxbuf[0] >> 4;
  if (chan_rb != chan) {
    EPT("Read ADC7953_SPI6 failed, rChanIdx != chanIdx\n");
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7953_SPI6 failed, rChanIdx != chanIdx\n");
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_1)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7953_SPI6 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_1);
    }
    return osError;
  } else {
    *val = ((rxbuf[0] & 0xf) << 8) | rxbuf[1];
  }

  return status;
}

osStatus_t RTOS_ADC7828_Read(uint8_t chan, uint16_t *val)
{
  osStatus_t status;
  HAL_StatusTypeDef hal_status;
  uint8_t txbuf, rxbuf[2];

  if ((status = osMutexAcquire(i2c1Mutex, I2C_WAIT_TIMEOUT)) != osOK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Acquire mutex of i2c1 failed\n");
    return status;
  }

  txbuf = 4 * (chan % 2) + chan / 2;
  txbuf = 0x84 | (txbuf << 4);

  hal_status = HAL_I2C_Mem_Read(&hi2c1, ADC7828_ADDR, (uint16_t)txbuf, I2C_MEMADD_SIZE_8BIT, rxbuf, 2, 50);

  osMutexRelease(i2c1Mutex);

  if (hal_status != HAL_OK) {
    THROW_LOG(MSG_TYPE_ERROR_LOG, "Read ADC7828 failed, ErrorCode = %#X\n", hi2c1.ErrorCode);
    if (!Is_Flag_Set(&run_status.exp, EXP_ADC_2)) {
      THROW_LOG(MSG_TYPE_ERROR_LOG, "ADC7828 abnormal\n");
      Set_Flag(&run_status.exp, EXP_ADC_2);
    }
    return osError;
  }

  *val = (rxbuf[0] << 8) | rxbuf[1];

  return status;
}


