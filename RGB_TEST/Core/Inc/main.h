/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"WS2812.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE BEGIN ET */



typedef signed long  					s32;
typedef signed short 					s16;
typedef signed char  					s8;
typedef unsigned char  uint8;                   // 无符�?8位整型变�?
typedef signed   char  int8;                    // 有符�?8位整型变�?
typedef unsigned short uint16;                  //无符�?16位整型变�?
typedef signed   short int16;                   //有符�?16位整型变�?
typedef unsigned long   uint32;                  // 无符�?32位整型变�?
typedef signed   long   int32;                   // 有符�?32位整型变�?
typedef float          fp32;                    // 单精度浮点数�?32位长度）
typedef double         fp64;                    //双精度浮点数�?64位长度）
typedef signed long  const 				sc32;  /* Read Only */
typedef signed short const 				sc16;  /* Read Only */
typedef signed char  const 				sc8;   /* Read Only */

typedef volatile signed long  			vs32;
typedef volatile signed short 			vs16;
typedef volatile signed char  			vs8;

typedef volatile signed long  const 	vsc32;  /* Read Only */
typedef volatile signed short const 	vsc16;  /* Read Only */
typedef volatile signed char  const 	vsc8;   /* Read Only */

typedef unsigned long  					u32;
typedef unsigned short 					u16;
typedef unsigned char  					u8;

typedef unsigned long  const 			uc32;  /* Read Only */
typedef unsigned short const 			uc16;  /* Read Only */
typedef unsigned char  const 			uc8;   /* Read Only */

typedef volatile unsigned long  		vu32;
typedef volatile unsigned short 		vu16;
typedef volatile unsigned char  		vu8;

typedef volatile unsigned long  const 	vuc32;  /* Read Only */
typedef volatile unsigned short const 	vuc16;  /* Read Only */
typedef volatile unsigned char  const 	vuc8;   /* Read Only */

typedef	struct	{
		unsigned char	bit0 : 1;
		unsigned char	bit1 : 1;
		unsigned char	bit2 : 1;
		unsigned char	bit3 : 1;
		unsigned char	bit4 : 1;
		unsigned char	bit5 : 1;
		unsigned char	bit6 : 1;
		unsigned char	bit7 : 1;
		} _8bit;

typedef	union {
		_8bit	bits;
		unsigned char byte;
		}_byte8;


/*
volatile	extern		_byte8		g_u8DspRam[4]; 		//
volatile 	extern		u8 			g_u8SegBuf[4]; 		//
volatile	extern 		u8 			g_u8ScanCoun;		//delay counter
volatile 	extern		u16 		g_u16Num;			//display data,0~999
*/

volatile	_byte8		g_u8Flag0,g_u8Flag1,g_u8Flag2,g_u8Flag3,g_u8Flag4,g_u8Flag5,g_u8Flag6,g_u8Flag_qian,g_u8Flag_hou,g_u8Flag_no_fault;

volatile		extern		uint16		g_u16EC;
volatile		extern		uint16		g_u16Temperiture;

//Flag0
#define			key1_in_Flag			g_u8Flag0.bits.bit0		//LCD掃描標誌�????????????????????
#define			key2_in_Flag			g_u8Flag0.bits.bit1		//按鍵掃描標誌�????????????????????
#define			key3_in_Flag			g_u8Flag0.bits.bit2		//TDS校準標誌�????????????????????
#define			key4_in_Flag			g_u8Flag0.bits.bit3		//NTC校準標誌�????????????????????
#define			toggle_Flag				g_u8Flag0.bits.bit4		//Hold標誌�????????????????????
#define			key5_in_Flag			g_u8Flag0.bits.bit5		//上電標誌�????????????????????
#define			sound_only_one_Flag		g_u8Flag0.bits.bit6		//Hold標誌�????????????????????
#define			sound_stop_Flag			g_u8Flag0.bits.bit7		//上電標誌�????????????????????

#define			FLAG_START_SOUND		g_u8Flag1.bits.bit0
#define			FLAG_OVER_SOUND			g_u8Flag1.bits.bit1
#define			FLAG_TEN_YUAN_SOUND		g_u8Flag1.bits.bit2
#define			FLAG_NO_MONEY_SOUND		g_u8Flag1.bits.bit3
#define			FLAG_MANY_MONEY_SOUND	g_u8Flag1.bits.bit4
#define			FLAG_START_ENABLE		g_u8Flag1.bits.bit5
#define			FLAG_OVER_ENABLE		g_u8Flag1.bits.bit6
#define			FLAG_RECEIVE_OK			g_u8Flag1.bits.bit7

#define			FLAG_START_IN			g_u8Flag2.bits.bit0
#define			FLAG_OVER_IN			g_u8Flag2.bits.bit1
#define			FLAG_RECEIVE_OK1		g_u8Flag2.bits.bit2
#define			FLAG_RECEIVE_OK2		g_u8Flag2.bits.bit3
#define			FLAG_RECEIVE_OK3		g_u8Flag2.bits.bit4
#define			FLAG_RECEIVE_OK4		g_u8Flag2.bits.bit5
#define			FLAG_RECEIVE_OK5		g_u8Flag2.bits.bit6
#define			FLAG_RECEIVE_OK6		g_u8Flag2.bits.bit7

#define			FLAG_RECEIVE_OKa		g_u8Flag3.bits.bit0
#define			FLAG_RECEIVE_OKb		g_u8Flag3.bits.bit1
#define			FLAG_RECEIVE_OKc		g_u8Flag3.bits.bit2
#define			FLAG_RECEIVE_OKd		g_u8Flag3.bits.bit3
#define			FLAG_RECEIVE_OKe		g_u8Flag3.bits.bit4
#define			FLAG_RECEIVE_OKf		g_u8Flag3.bits.bit5
#define			FLAG_RECEIVE_OKg		g_u8Flag3.bits.bit6
#define			FLAG_RECEIVE_OKh		g_u8Flag3.bits.bit7

#define			flag_qian_error1		g_u8Flag_qian.bits.bit0
#define			flag_qian_error2		g_u8Flag_qian.bits.bit1
#define			flag_qian_error3		g_u8Flag_qian.bits.bit2
#define			flag_qian_error4		g_u8Flag_qian.bits.bit3
#define			flag_qian_alarm1		g_u8Flag_qian.bits.bit4
#define			flag_qian_alarm2		g_u8Flag_qian.bits.bit5
#define			flag_qian_alarm3		g_u8Flag_qian.bits.bit6
#define			flag_qian_alarm4		g_u8Flag_qian.bits.bit7

#define			flag_hou_error1			g_u8Flag_hou.bits.bit0
#define			flag_hou_error2			g_u8Flag_hou.bits.bit1
#define			flag_hou_error3			g_u8Flag_hou.bits.bit2
#define			flag_hou_error4			g_u8Flag_hou.bits.bit3
#define			flag_hou_alarm1			g_u8Flag_hou.bits.bit4
#define			flag_hou_alarm2			g_u8Flag_hou.bits.bit5
#define			flag_hou_alarm3			g_u8Flag_hou.bits.bit6
#define			flag_hou_alarm4			g_u8Flag_hou.bits.bit7

#define			flag_stop					g_u8Flag4.bits.bit0
#define 		flag_diandong_qianjin		g_u8Flag4.bits.bit1
#define 		flag_diandong_houtui		g_u8Flag4.bits.bit2
#define 		flag_diandong_enable    	g_u8Flag4.bits.bit3
#define 		flag_diandong_hou_enable	g_u8Flag4.bits.bit4
#define 		flag_stop_ir_qian    		g_u8Flag4.bits.bit5
#define 		flag_stop_ir_hou    		g_u8Flag4.bits.bit6
#define 		flag_flash_led		    	g_u8Flag4.bits.bit7

#define			FLAG_turn_on_ok				g_u8Flag6.bits.bit0
#define 		FLAG_turn_off_ok			g_u8Flag6.bits.bit1
#define 		flag_CC2520_Search			g_u8Flag6.bits.bit2
#define 		flag_cc2520_ok				g_u8Flag6.bits.bit3
#define			FLAG_1						g_u8Flag6.bits.bit4
#define 		FLAG_2						g_u8Flag6.bits.bit5
#define 		FLAG_3						g_u8Flag6.bits.bit6
#define 		FLAG_4						g_u8Flag6.bits.bit7

#define 		FLAG_5			g_u8Flag_no_fault.bits.bit0
#define 		flag_hou_no_fault			g_u8Flag_no_fault.bits.bit1
#define 		flag_qian_fault				g_u8Flag_no_fault.bits.bit2
#define 		flag_hou_fault				g_u8Flag_no_fault.bits.bit3
#define 		FLAG_START_IN_1				g_u8Flag_no_fault.bits.bit4
#define 		FLAG_OVER_IN_1				g_u8Flag_no_fault.bits.bit5
#define 		FLAG_turn_on				g_u8Flag_no_fault.bits.bit6
#define 		FLAG_turn_off				g_u8Flag_no_fault.bits.bit7
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_15
#define KEY3_GPIO_Port GPIOC
#define LED1_DO_Pin GPIO_PIN_1
#define LED1_DO_GPIO_Port GPIOC
#define LED_DO_Pin GPIO_PIN_2
#define LED_DO_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOC
#define RS485_CE3_Pin GPIO_PIN_1
#define RS485_CE3_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_12
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_13
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_15
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOC
#define D6_Pin GPIO_PIN_8
#define D6_GPIO_Port GPIOC
#define D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOC
/*
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
*/
#define SET_D0   GPIOB->ODR|=1<<12
#define CLR_D0   GPIOB->ODR&=(~(1<<12))
#define SET_D1   GPIOB->ODR|=1<<13
#define CLR_D1   GPIOB->ODR&=(~(1<<13))
#define SET_D2   GPIOB->ODR|=1<<14
#define CLR_D2   GPIOB->ODR&=(~(1<<14))
#define SET_D3   GPIOB->ODR|=1<<15
#define CLR_D3   GPIOB->ODR&=(~(1<<15))

#define SET_D4   GPIOC->ODR|=1<<6
#define CLR_D4   GPIOC->ODR&=(~(1<<6))
#define SET_D5   GPIOC->ODR|=1<<7
#define CLR_D5   GPIOC->ODR&=(~(1<<7))
#define SET_D6   GPIOC->ODR|=1<<8
#define CLR_D6   GPIOC->ODR&=(~(1<<8))
#define SET_D7   GPIOC->ODR|=1<<9
#define CLR_D7   GPIOC->ODR&=(~(1<<9))

#define SET_WR   GPIOA->ODR|=1<<15
#define CLR_WR   GPIOA->ODR&=(~(1<<15))
#define SET_DC   GPIOA->ODR|=1<<8
#define CLR_DC   GPIOA->ODR&=(~(1<<8))
#define SET_RS   GPIOB->ODR|=1<<8
#define CLR_RS   GPIOB->ODR&=(~(1<<8))
#define SET_CS   GPIOB->ODR|=1<<9
#define CLR_CS   GPIOB->ODR&=(~(1<<9))

#define SET_BK   GPIOA->ODR|=1<<1
#define CLR_BK   GPIOA->ODR&=(~(1<<1))

#define SET_WS2812DATA_ME   GPIOC->ODR|=1<<2
#define CLR_WS2812DATA_ME   GPIOC->ODR&=(~(1<<2))

#define SET_WS2812DATA   GPIOC->ODR|=1<<1
#define CLR_WS2812DATA   GPIOC->ODR&=(~(1<<1))
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOC
#define KEY3A_Pin GPIO_PIN_15
#define KEY3A_GPIO_Port GPIOC
#define LED1_DO_Pin GPIO_PIN_1
#define LED1_DO_GPIO_Port GPIOC
#define LED_DO_Pin GPIO_PIN_2
#define LED_DO_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOC
#define CC2520_NRST_GPIO_Pin GPIO_PIN_4
#define CC2520_NRST_GPIO_GPIO_Port GPIOA
#define CC2520_CLK_GPIO_Pin GPIO_PIN_5
#define CC2520_CLK_GPIO_GPIO_Port GPIOA
#define CC2520_MOSI_GPIO_Pin GPIO_PIN_6
#define CC2520_MOSI_GPIO_GPIO_Port GPIOA
#define CC2520_MISO_GPIO_Pin GPIO_PIN_7
#define CC2520_MISO_GPIO_GPIO_Port GPIOA
#define CC2520_POWEN_GPIO_Pin GPIO_PIN_4
#define CC2520_POWEN_GPIO_GPIO_Port GPIOC
#define KEY5_Pin GPIO_PIN_5
#define KEY5_GPIO_Port GPIOC
#define CC2520_GP1_GPIO_Pin GPIO_PIN_0
#define CC2520_GP1_GPIO_GPIO_Port GPIOB
#define RS485_CE3_Pin GPIO_PIN_1
#define RS485_CE3_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_12
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_13
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_15
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOC
#define D6_Pin GPIO_PIN_8
#define D6_GPIO_Port GPIOC
#define D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOA
#define CC2520_CS_GPIO_Pin GPIO_PIN_10
#define CC2520_CS_GPIO_GPIO_Port GPIOC
#define CC2520_GP0_GPIO_Pin GPIO_PIN_12
#define CC2520_GP0_GPIO_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
extern uint8_t tim2_counter;
extern uint16_t time_1ms;
extern void RGB_open(void );
extern void RGB_PWM(void );
extern void delay_us1(uint32_t us);
extern void send_single_data(void);
extern void reset_ws2812(void);
extern void RGB_open1(void );
extern void RGB_PWM1(void );
extern void send_single_data1(void);
extern void  send_string_data1();
extern void  send_string_data();
extern void  send_string_data_LK_H();	//水平
extern void  send_string_data_LK_V();	//垂直
extern void reset_ws2812_1(void);
extern void RGB_close1(void); //RGB闪烁
extern void RGB_close(); //RGB闪烁
extern uint8_t LED_DAT;//可位操作的数据发送暂存变量声�?????????????

extern uint8_t RR,GG,BB; //RGB灰度值全�?????????????变量声明
extern uint8_t RR1,GG1,BB1;
extern uint8_t RR2,GG2,BB2;
extern uint8_t Rx_count_UART1,Rx_count_UART2,Rx_count_UART3,Rx_count_UART4;
extern uint8_t Rxbuff1[32],Rxbuff2[32],Rxbuff3[200],Rxbuff4[13];
extern uint8_t play_time,checksum;
extern void lcd_init(void);
extern void TFT_Disp128160(const uchar *dat);
extern void lcd_address(uint XS,uint YS,uint x_total,uint y_total);		//设置窗口坐标
extern void LCD_SetPos(uint x,uint y);		//设置像素点坐�????????
extern void display_color(uint color);		//显示全屏单一颜色

extern void line_display_color(uchar y_start, uchar y_end, uint color);  //显示整行单一颜色
extern void mono_data_out(uchar mono_data,uint font_color,uint back_color); //将单色的8位的数据（代�????????8个像素点）转换成彩色的数据传输给液晶�????????
extern void mono_data_out_5x8(uchar mono_data,uint font_color,uint back_color);	//将单色的8位的数据的高5位（代表5个像素点）转换成彩色的数据传输给液晶�????????

extern void  send_image(const uchar *data);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
