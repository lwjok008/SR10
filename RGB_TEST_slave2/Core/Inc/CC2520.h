/*
 * CC2520.h
 *
 *  Created on: Apr 8, 2020
 *      Author: LWJ
 */

#ifndef INC_CC2520_H_
#define INC_CC2520_H_

#include "main.h"

#include "gpio.h"


#define TRUE    1
#define FALSE  0

//--------------------射频芯片相关定义-------------------
#define CC2520_PSDU_Len        127

#define  REG_READ                     	(0x80)
#define  REG_WRITE                    	(0xC0)
#define  MEM_READ                     	(0x10)
#define  MEM_WRITE                  	(0x20)
#define  RXFIFO_READ                	(0x30)
#define  TXFIFO_WRITE               	(0x3A)
//-------------------- CC2520 命令字定义----------------------------
#define CMD_SNOP    		         	(0x00)
#define CMD_IBUFLD 	   	         		(0x02)
#define CMD_SIBUFEX            	  		(0x03)
#define CMD_SSAMPLECCA          		(0x04)
#define CMD_SRES 	 		  			(0x09)

#define CMD_SXOSCON	          			(0x40)
#define CMD_SXOSCOFF	          		(0x46)
#define CMD_SRXON	                 	(0x42)
#define CMD_STXON	                 	(0x43)
#define CMD_SRFOFF	                 	(0x45)

#define CMD_SFLUSHRX              		(0x47)
#define CMD_SFLUSHTX	 	  			(0x48)
//------------------ CC2520 配置寄存器-----------------------------------
#define FRMFILT0                        (0x00)
#define FRMFILT1                        (0x01)
#define SRCMATCH                       	(0x02)
#define SRCSHORTEN0                 	(0x04)
#define SRCSHORTEN1                 	(0x05)
#define SRCSHORTEN2                 	(0x06)
#define SRCEXTEN0                     	(0x08)
#define SRCEXTEN1                     	(0x09)
#define SRCEXTEN2                     	(0x0A)
#define FRMCTRL0                       	(0x0C)
#define FRMCTRL1                       	(0x0D)
#define RXENABLE0                     	(0x0E)
#define RXENABLE1                     	(0x0F)
#define EXCFLAG0                       	(0x10)
#define EXCFLAG1                       	(0x11)
#define EXCFLAG2                       	(0x12)
#define EXCMASKA0                    	(0x14)
#define EXCMASKA1                    	(0x15)
#define EXCMASKA2                    	(0x16)
#define EXCMASKB0                    	(0x18)
#define EXCMASKB1                    	(0x19)
#define EXCMASKB2                    	(0x1A)
#define EXCBINDX0                    	(0x1C)
#define EXCBINDX1                    	(0x1D)
#define EXCBINDY0                    	(0x1E)
#define EXCBINDY1                    	(0x1F)
#define GPIOCTRL0                    	(0x20)
#define GPIOCTRL1                    	(0x21)
#define GPIOCTRL2                    	(0x22)
#define GPIOCTRL3                    	(0x23)
#define GPIOCTRL4                    	(0x24)
#define GPIOCTRL5                    	(0x25)
#define GPIOPOLARITY              		(0x26)
#define GPIOCTRL                      	(0x28)
#define DPUCON                        	(0x2A)
#define DPUSTAT                      	(0x2C)
#define FREQCTRL                     	(0x2E)
#define FREQTUNE                     	(0x2F)
#define TXPOWER                      	(0x30)
#define FSMSTAT0                     	(0x32)
#define FSMSTAT1                     	(0x33)
#define FIFOPCTRL                    	(0x34)
#define FSMCTRL                       	(0x35)
#define CCACTRL0                     	(0x36)
#define CCACTRL1                     	(0x37)
#define RSSI                            (0x38)
#define RSSISTAT                    	(0x39)

#define RXFIRST                     	(0x3C)
#define RXFIFOCNT                   	(0x3E)
#define TXFIFOCNT                   	(0x3F)
//-------------------PSDU相关定义---------------------------
//FRAME_CONTROL_FIELD
//FRAME_TYPE
#define  FRAME_TYPE_BEACCON    		0x00
#define  FRAME_TYPE_DATA          	0x01
#define  FRAME_TYPE_ACK            	0x02
#define  FRAME_TYPE_MAC            	0x03

#define SECURITY_ENABLE             0x00
#define FRAME_PENDING               0x00
#define ACKNOWLEDGMENT_REQUEST      0x00
#define PAN_ID_COMPRESSION          0x00
#define DEST_ADDRESSING_MODE        0x03
#define FRAME_VERSION               0x01
#define SOURCE_ADDRESSING_MODE      0x03
//
#define SEQUENCE_NUMBER             0x02
//-------------------- CC2520 RAM 地址-----------------------------------
#define RAM_IEEEADR         (0xEA)
#define RAM_PANID		    (0xF2)
#define RAM_SHORTADR        (0xF4)
//---------------------CC2520引脚定义------------------------
#define	CC2520_POWEN_PC7		(1<<4)
#define	CC2520_NRST_PB9			(1<<4)
#define	CC2520_CS_PB12			(1<<10)
#define	CC2520_CLK_PB13			(1<<5)
#define	CC2520_MISO_PB14		(1<<7)
#define	CC2520_MOSI_PB15		(1<<6)
#define	CC2520_GP0_PA11			(1<<12)
#define	CC2520_GP1_PA12			(1<<0)



#define CC2520_NRST_GPIO_Pin GPIO_PIN_4
#define CC2520_NRST_GPIO_GPIO_Port GPIOA
#define CC2520_CLK_GPIO_Pin GPIO_PIN_5
#define CC2520_CLK_GPIO_GPIO_Port GPIOA
#define CC2520_MISO_GPIO_Pin GPIO_PIN_7
#define CC2520_MISO_GPIO_GPIO_Port GPIOA
#define CC2520_MOSI_GPIO_Pin GPIO_PIN_6
#define CC2520_MOSI_GPIO_GPIO_Port GPIOA
#define CC2520_POWEN_GPIO_Pin GPIO_PIN_4
#define CC2520_POWEN_GPIO_GPIO_Port GPIOC
#define CC2520_CS_GPIO_Pin GPIO_PIN_10
#define CC2520_CS_GPIO_GPIO_Port GPIOC
#define CC2520_GP0_GPIO_Pin GPIO_PIN_12
#define CC2520_GP0_GPIO_GPIO_Port GPIOC
#define CC2520_GP1_GPIO_Pin GPIO_PIN_0
#define CC2520_GP1_GPIO_GPIO_Port GPIOB

#define	READ_CC2520_GP0_DATA()		HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin)
#define	READ_CC2520_GP1_DATA()		HAL_GPIO_ReadPin(CC2520_GP1_GPIO_GPIO_Port,CC2520_GP1_GPIO_Pin)

#define PowerOn_CC2520()			{GPIO_ResetBits(CC2520_POWEN_GPIO_PORT, CC2520_POWEN_GPIO_PIN);}
#define PowerOff_CC2520()			{GPIO_SetBits(CC2520_POWEN_GPIO_PORT, CC2520_POWEN_GPIO_PIN);}

#define	POWER_ON_CC2520()	GPIOC->ODR|=1<<4
#define	POWER_OFF_CC2520()	GPIOC->ODR&=(~(1<<4))
#define RESET_ON()    		GPIOA->ODR|=1<<4
#define RESET_OFF()  		GPIOA->ODR&=(~(1<<4))
#define SCLK_ON()    		GPIOA->ODR|=1<<5
#define SCLK_OFF()  		GPIOA->ODR&=(~(1<<5))
#define MOSI_ON()    		GPIOA->ODR|=1<<6
#define MOSI_OFF()  		GPIOA->ODR&=(~(1<<6))
#define CSN_ON()    		GPIOC->ODR|=1<<10
#define CSN_OFF()  			GPIOC->ODR&=(~(1<<10))
#define GPIO0_IN     	   	(GPIOA->IDR&CC2520_GP0_PA11)
#define GPIO1_IN     	   	(GPIOA->IDR&CC2520_GP1_PA12)
#define MISO_IN()			HAL_GPIO_ReadPin(CC2520_MISO_GPIO_GPIO_Port,CC2520_MISO_GPIO_Pin)
//#define MISO_IN     	   	(GPIOA->IDR&CC2520_MISO_PB14)

#define	SET_CC2520_Sleep()	CC2520_Command(CMD_SRFOFF)

extern	uint8 CC2520_PSDU[1+CC2520_PSDU_Len];
extern	uchar	RF_BadCycCnt;
extern	uchar	CC2520_WorkFreqIndex;

//----------------------函数原型------------------------
extern	void CC2520_Init(uchar freq);
extern	uint8 CC2520_ReadReg(uint8  addr);
extern	void CC2520_WriteReg(uint8  addr, uint8 value);
extern	uint8 CC2520_ReadRAM(uint8 addrH,uint8 addrL);
extern	void CC2520_WriteRAM(uint8 addrH,uint8 addrL, uint8 value);
extern	void CC2520_ReadRXFIFO(void);
extern	void CC2520_WriteTXFIFO(uchar txd_len);
extern	void CC2520_Command(uint8 cmd);

extern	void CC2520_SetRxMode(void);
extern	uint8 CC2520_RxPacket(void) ;
extern	void CC2520_TxPacket(void);

extern	void CC2520_SPI_Init(void);
extern	uint8 SPI_Read(void);
extern	void SPI_Write(uint8 txdata);
extern	uint8 SPI_WriteRead(uint8 txdata);

extern	void CC2520_GPIO_Config(void);
extern void cc2520_data_load();		//数据判断装载  是本机配对senser接收   非本机的丢弃
extern void CC2520_Search(void);
extern void cc2520_rx_app();
extern void CC2520_IntoActMode_Init();
extern void temp_send();			//温度发送
extern void delay_init(u8 SYSCLK);
extern uchar ID_ARM_UP_A[6];
extern uint time_cc2520_no_rx;
extern uchar fre_set,fre_set_temp, fre_set_change;
extern uint time_delay;
#endif /* INC_CC2520_H_ */
