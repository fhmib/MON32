#ifndef __XMODEM_H__
#define __XMODEM_H__
#include "main.h"

#define FALSE_CHECKERR			2
#define FALSE_NAK_OVERTIME		3
#define FALSE_OVERTIME			4
#define FALSE_CANCEL			5
#define FALSE_EOL				6

#define XM_SOH 					0x01    //xmodem
#define XM_STX 					0x02    //1k-xmodem
#define XM_EOT 					0x04
#define XM_ACK 					0x06
#define XM_NAK 					0x15
#define XM_CAN 					0x18 
#define XM_PAD 					0x1a
#define XM_C   					0x43

#define BLOCK_SIZE				1024 
#define RETRIES					120

#define UART_WAIT_1S            9000000 
#define UART_WAIT_100mS         900000

#define	NAK_MARK				0x5A


#define 	FLASH_MCU_FILELEN_ADDR		    		0  	 					
#define 	FLASH_MCU_FILELEN_LEN	        		4  	 					
#define 	FLASH_MCU_FILECRC_ADDR		    		4 	
#define 	FLASH_MCU_FILECRC_LEN		    		4  
#define 	FLASH_FPGA_FILELEN_ADDR         		8
#define 	FLASH_FPGA_FILELEN_LEN           		4
#define 	FLASH_FPGA_FILECRC_ADDR		    		12 	
#define 	FLASH_FPGA_FILECRC_LEN		    		4 	
#define 	FLASH_PN_ADDR				    		16
#define 	FLASH_PN_LEN				    		16  
#define 	FLASH_HW_ADDR				    		32 			
#define 	FLASH_HW_LEN				     		6
#define 	FLASH_FILESEL_FLG_ADDR		    		38 			
#define 	FLASH_FILESEL_FLG_LEN		    		1

typedef struct
{
	uint32_t ulMCULen;
	uint32_t ulMCUCrc;
	uint32_t ulFPGALen;
	uint32_t ulFPGACrc;
	uint8_t ucPartNumber[FLASH_PN_LEN];
	uint8_t ucHW[FLASH_HW_LEN];
	uint8_t ucDeviceSel;
}TFileHead;

extern char xmodem_receive(uint32_t *length);

#endif /*__XMODEM_H__*/

