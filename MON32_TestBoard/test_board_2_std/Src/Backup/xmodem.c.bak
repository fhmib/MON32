/***************************************************************************
* 
* File Name:    xmodem.c
* 
* Description:  
*   (+) USART_USE is the uart device of STM32F4
*   (+) UART_WAIT_1S should be modified to adapt(depends on code execution frequency)
*   (+) Start xmodem recieve using upgradeFW()
*   (+) XM_SOH: xmodem(128bytes) (no use, it can be modified to adapt)
*   (+) XM_STX: 1k-xmodem(1024bytes)
*   (+) Print CCCC to serial terminal
*   (+) Serial terminal transfer file with (1k-xmodem) 
*
* Revision History:
* 1-24, 2019	Chi Bao	File created
* 
***************************************************************************/
#include "xmodem.h"
#include "flash_if.h"

uint8_t xfer_buffer[1024+10];

char cGlorecv;
uint32_t uiGlocount;
uint8_t ucNAK_Flg;

long xfer_wait_byte(long timeOut)
{
  uint32_t time = timeOut;

  while (time--) {
    if(serial_available()) {                             
      cGlorecv = serial_read();
      return 1;
    }                               
  }

  return 0;
}

void xfer_send_byte(char theChar)
{
  HAL_UART_Transmit(&TERMINAL_UART, (uint8_t*)&theChar, 1, 0xFF);
}

int xfer_recv_byte(const long timeOut)
{
  int result;

  if (xfer_wait_byte(timeOut)) {
    result = cGlorecv;
  } else {
    result = -1;
  }

  return result;
}

void canTrans(void)
{
  int i = 0;

  for (i = 0; i < 9; i++) {
    xfer_send_byte(XM_NAK);
    HAL_Delay(1);
  }
}

uint32_t xmodemSum(uint8_t *ptr, int count)
{
  int crc = 0;
  char i = 0;

  crc = 0;

  while (--count >= 0) {

    crc = crc ^ *ptr++ << 8;

    for (i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = crc << 1 ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }

  return (crc & 0xFFFF);
}

void xmodem_retry(void)
{
  while (xfer_recv_byte(UART_WAIT_1S) != -1) {
    ;
  }

  xfer_send_byte(XM_NAK);
  ucNAK_Flg = NAK_MARK;
}

char xfer_recv_block(uint32_t blocksize)
{
  uint32_t    totalCount;
  uint32_t    count;
  int recvChar;

  count = 0;
  totalCount = blocksize + 4;

  while (count < totalCount)
  {
    recvChar = xfer_recv_byte(UART_WAIT_1S);

    if (recvChar != -1) {
      xfer_buffer[count++] = recvChar;
    } else {
      break;
    }
  }

  return (count == totalCount) ? 1 : 0;
}

char xmodemCrc16checksum(uint32_t blocksize)
{
  uint32_t remotecrc = 0;
  uint32_t crc = 0;

  remotecrc = 0x00ff & xfer_buffer[blocksize + 2];
  remotecrc = remotecrc << 8;
  remotecrc  |= 0x00ff & xfer_buffer[blocksize + 3];


  crc = xmodemSum(&xfer_buffer[2], (blocksize));

  if (crc == remotecrc) {
    return 1;
  } else {
    return 0;
  }
}

char xmodem_check_block(uint32_t blockNo, uint32_t blocksize)
{
  uint8_t ucLocals1;
  uint32_t uiLocal2;

  ucLocals1 =  ~xfer_buffer[1];
  ucLocals1 &= 0x00FF;

  uiLocal2 = blockNo;
  uiLocal2 &= 0x00FF;

  if ((uiLocal2 == xfer_buffer[0]) && (uiLocal2 == ucLocals1) && (xmodemCrc16checksum(blocksize))) {
    return 1;
  } else {
    return 0;
  }
}


char xmodem_receive(uint32_t *length)
{
  uint32_t retris;
  uint32_t offset;
  uint32_t blocksize;
  int recvChar;
  uint32_t blockNo;

  retris = RETRIES;
  recvChar = 0;
  offset = 0;
  blocksize = 0;
  ucNAK_Flg = 0;

  while (retris--) {
    if (!xfer_wait_byte(UART_WAIT_1S)) {
      xfer_send_byte(XM_C);
    } else {
      if (cGlorecv == 27) { // press ESC to exit.  
        return 1;
      } else {
        if (cGlorecv != 13) { // not press CR
          break;
        }
      }
    }

    if (retris == 0) {    
      return 2;
    }
  }

  retris = RETRIES;
  blockNo = 1;

  while (retris) {
    if((blockNo == 1) && (ucNAK_Flg != NAK_MARK)) {
      recvChar = cGlorecv;
    } else {
      recvChar = xfer_recv_byte(UART_WAIT_1S);
      if (recvChar == -1) {
        retris--;
        continue;
      }
    }

    switch (recvChar) {

    case XM_SOH:
      canTrans();
      return 3;

    case XM_STX:
      blocksize = BLOCK_SIZE;
      if (offset + blocksize > 1024 * 128 * 3) {
        canTrans();
        retris = 0;
        return 8;
      }
      if (xfer_recv_block(blocksize)) {
        if (xmodem_check_block(blockNo, blocksize)) {
          // Write data to flash
          if (FLASH_If_Write(IMAGE_ADDRESS + offset, (uint32_t*)(xfer_buffer + 2), blocksize / 4) != FLASHIF_OK) {
            canTrans();
            return 4;
          }

          offset += blocksize;
          xfer_send_byte(XM_ACK);
          blockNo++;
          retris = RETRIES;
        } else {
          canTrans();
          retris = 0;
          return 5;
        }
      } else {
        retris--;
      }
      break;

    case XM_CAN:
      xfer_send_byte(XM_ACK);
      return 6;

    case XM_EOT:
      xfer_send_byte(XM_ACK);
      retris = 0;
      *length = offset;
      return 0;

    default:
      xmodem_retry();
      retris--;
      if (retris == 0) {
        canTrans();
        return 7;
      }
      break;
    }
  }

  return 10; 
}


