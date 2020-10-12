/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"
#include "CC2520.h"

#define KEY1_1 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)==1)    //Êý¾ÝÊäÈë
#define KEY1_0 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)==0)    //Êý¾ÝÊäÈë

#define KEY2_1 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)==1)    //Êý¾ÝÊäÈë
#define KEY2_0 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)==0)    //Êý¾ÝÊäÈë

#define SET_KEY3_1   GPIOC->ODR|=1<<15
#define SET_KEY3_0   GPIOC->ODR&=(~(1<<15))

#define SET_KEY4_1   GPIOC->ODR|=1<<3
#define SET_KEY4_0   GPIOC->ODR&=(~(1<<3))
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Private variables ---------------------------------------------------------*/

uint8_t aTxStartMessage[10] ={0X55,0X55,0X55,0X55,0X55,0XAA,0XAA,0XAA,0XAA,0XAA};
uint8_t Send_buf[8] = {0} ;
uint8_t Rx_count_UART1,Rx_count_UART2,Rx_count_UART3,Rx_count_UART4;
uint8_t Rxbuff1[32],Rxbuff2[32],Rxbuff3[200];
uint8_t Rxbuff4[13]={0X55,0XAA,0X55,0X55,0X55,0XAA,0XAA,0XAA,0XAA,0XAA,0XAA,0XAA,0XA5};
/* Buffer used for reception */
uint8_t aRxBuffer[32],bRxBuffer[200],cRxBuffer[200],dRxBuffer[13];
uint8_t Rx_Num_UART1,Rx_Num_UART2,Rx_Num_UART3,Rx_Num_UART4;
uint8_t TX_LEN,Rx_LEN;
uint8_t R_BIT,G_BIT,B_BIT,i,j=0,delta_RG,delta_GB,delta_BX;
uint8_t R_DATA[3],G_DATA[3],B_DATA[3];
void IOT_PROCESS(void);

uint8_t TX_BUFF_WENXIN[7]={0XA5,0X5A,0X03,0X02,0X01,0X01,0X06};

volatile	_byte8	g_u8Flag1,g_u8Flag2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern const unsigned char gImage_Peaks_logo[40968];
extern const unsigned char gImage_3[216];
extern const unsigned char gImage_8[216];
extern const unsigned char gImage_9[216];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t tim2_counter;
uint16_t time_1ms;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef uint8_t u8;
typedef uint32_t u32;
u8 fac_us;


void delay_init(u8 SYSCLK)
{
    #if SYSTEM_SUPPORT_OS
        u32 reload;
    #endif

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    //SysTick  HCLK
    fac_us=SYSCLK;              // OS,fac_us

    #if SYSTEM_SUPPORT_OS       // OS.
        reload=SYSCLK;
        reload*=1000000/delay_ostickspersec;    //delay_ostickspersec
        //reload 24Bit 16777216, 180M 0.745s
        fac_ms=1000/delay_ostickspersec;        // OS.
        SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;// SYSTICK
        SysTick->LOAD=reload;                   // 1/OS_TICKS_PER_SEC
        SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; // SYSTICK
        #else
    #endif
}


void delay_us1(u32 nus)
{
    u32 ticks;
    u32 told,tnow,tcnt=0;
    u32 reload=SysTick->LOAD;                   //LOAD
    ticks=nus*fac_us;
    told=SysTick->VAL;
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;       // SYSTICK.
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break;
        }
    }
}
uint8	g_u8Time20ms_1,g_u8Time20ms_2,g_u8Time20ms_3,g_u8Time20ms_4,g_u8Time20ms_5,g_u8Time20ms_6,g_u8Time20ms_7;
uint16_t time3_20ms=0;
uint16_t cc2520_rx_cycle;
//uchar fre_set,fre_set_temp, fre_set_change ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint16_t i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USB_PCD_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  RR=0; GG=0; BB=0;
  FLAG_RECEIVE_OK1=1;
  fre_set=0;
  HAL_UART_Receive_IT(&huart3,(uint8_t *)cRxBuffer,13); //
 // delay_init(72);

  reset_ws2812();
  reset_ws2812_1();
  RGB_close1();
  RGB_close();
  /*
  HAL_GPIO_WritePin(RS485_CE3_GPIO_Port, RS485_CE3_Pin, GPIO_PIN_SET);
  HAL_Delay(2);//延时大概 1mS

  HAL_UART_Transmit_IT(&huart3, (uint8_t *)TX_BUFF_WENXIN, sizeof(TX_BUFF_WENXIN)); //发�?�此帧数�???????????????????????????????????????
	while(HAL_UART_Transmit_IT(&huart3, (uint8_t *)TX_BUFF_WENXIN, sizeof(TX_BUFF_WENXIN))!=HAL_OK)
	{
		;
	}
	*/
  lcd_init();
  TFT_Disp128160(gImage_Peaks_logo);
	  HAL_GPIO_WritePin(RS485_CE3_GPIO_Port, RS485_CE3_Pin, GPIO_PIN_RESET);
	  /*
	  HAL_TIM_Base_Stop_IT(&htim2);//关闭定时
	  HAL_TIM_Base_Stop_IT(&htim3);//关闭定时
	  HAL_TIM_Base_Stop_IT(&htim4);//关闭定时
	  HAL_TIM_Base_Stop_IT(&htim5);//关闭定时
	  */
	  HAL_Delay(1);//延时大概 1mS
	  CC2520_GPIO_Config();
	  //--------------------------SPI初始化函�??????---------------------------
		CC2520_SPI_Init();
		CC2520_Init(0);
		CC2520_Command(CMD_SXOSCOFF);
		//CC2520_IntoActMode_Init();
	 // CC2520_Init(0);
	  flag_CC2520_Search=0;

/*
	  RR=0;
	  GG=255;
	  BB=0;
	  send_string_data1();
	  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
		if(CC2520_RxPacket())		//判断RF2520接收
		{
			//time_cc2520_no_rx=0;
			CC2520_ReadRXFIFO();	//数据接收
			cc2520_data_load();
		}

		if(flag_CC2520_Search)
		{
			flag_CC2520_Search=0;
			CC2520_Search();
		}
		*/
		if((cc2520_rx_cycle>5))		//连续周期1s,13.7s=685
		{
			cc2520_rx_cycle=0;
			//CC2520_Command(CMD_SXOSCON);	//zigbee晶振�??�??
			//HAL_Delay(1);
			temp_send();
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

			//HAL_Delay(1000);
			//POWER_OFF_CC2520();
		}
		/*
		if(time_cc2520_no_rx>1000)	//连续20s未接收到，初始化CC2520
		{
			time_cc2520_no_rx=0;
			CC2520_SPI_Init();
			HAL_Delay(10);
			CC2520_Init(fre_set);
			CC2520_IntoActMode_Init();
			CC2520_SetRxMode();		//设置为接收模�??????
		}
		*/
      if(FLAG_RECEIVE_OK)
      {
    	  FLAG_RECEIVE_OK=0;
	      HAL_GPIO_WritePin(GPIOB, RS485_CE3_Pin, GPIO_PIN_SET);
	      HAL_Delay(2);//延时大概 1mS
    	  IOT_PROCESS();	//IOT处理进程
        //HAL_UART_Transmit(&huart2,TX_BUFF_WENXIN,TX_LEN,0xffff);    //发�?�接收到的数�?????????????????????????????????????
    	  /*
    	if(FLAG_RECEIVE_OK1)
    	{
    		FLAG_RECEIVE_OK1=0;

    		HAL_UART_Transmit_IT(&huart3, (uint8_t *)TX_BUFF_WENXIN, sizeof(TX_BUFF_WENXIN)); //发�?�此帧数�???????????????????????????????????????
			while(HAL_UART_Transmit_IT(&huart3, (uint8_t *)TX_BUFF_WENXIN, sizeof(TX_BUFF_WENXIN))!=HAL_OK)
			{
				;
			}
			*/
    	//}

		  HAL_GPIO_WritePin(GPIOB, RS485_CE3_Pin, GPIO_PIN_RESET);
		  HAL_Delay(1);//延时大概 1mS

      }
     // RGB_open();
     // RGB_PWM();
     // RGB_open1();
     // RGB_PWM1();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == htim2.Instance)
	{
		FLAG_RECEIVE_OK = 1;	//断帧机制   20ms没接收到数据，判断为1 �????????????

		Rx_LEN=Rx_count_UART3;
		Rx_count_UART3=0;
		HAL_TIM_Base_Stop_IT(&htim2);//关闭定时
	}
	if (htim->Instance == htim3.Instance)
	{
		cc2520_rx_cycle++;
		time_cc2520_no_rx++;
		time_delay++;
		time3_20ms++;
		if(FLAG_1)
		{
			  RR=25; GG=0; BB=0;
			  send_string_data();
		}
		else if(FLAG_2)
		{
			  RR=0; GG=25; BB=0;
			  send_string_data();
		}
		else if(FLAG_3)
		{
			  RR=0; GG=0; BB=25;
			  send_string_data();
		}
		else if(FLAG_4)
		{
			  RR=25; GG=25; BB=25;
			  send_string_data();
		}
		else if(FLAG_5)
		{
			  RR=0; GG=0; BB=0;
			  send_string_data();
		}
		if(time3_20ms>25)
		{
			time3_20ms=0;
			if(flag_cc2520_ok)
			{
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			}
		}

		SET_KEY3_0;
		SET_KEY4_1;
		//asm("nop");asm("nop");
		  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==0)	//SW1
		  {
			  g_u8Time20ms_1++;
			  if(g_u8Time20ms_1>1)
			  {
				  g_u8Time20ms_1=3;
				  key1_in_Flag=1;
			  }
		  }
		  else
		  {
			  g_u8Time20ms_1=0;
			  if(key1_in_Flag)
			  {
				  key1_in_Flag=0;
				  RR=25; GG=0; BB=0;
				  send_string_data();
				  //send_image(gImage_8);
			  }
		  }
		  if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)==0)	//SW3
		  {
			  g_u8Time20ms_2++;
			  if(g_u8Time20ms_2>1)
			  {
				  g_u8Time20ms_2=3;
				  key2_in_Flag=1;
			  }
		  }
		  else
		  {
			  g_u8Time20ms_2=0;
			  if(key2_in_Flag)
			  {
				  key2_in_Flag=0;
				  //send_image(gImage_9);
				  RR=0; GG=0; BB=25;
				  send_string_data();
			  }
		  }

		SET_KEY3_1;
		SET_KEY4_0;
		//asm("nop");asm("nop");
		  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==0)	//SW2
		  {
			  g_u8Time20ms_3++;
			  if(g_u8Time20ms_3>1)
			  {
				  g_u8Time20ms_3=3;
				  key3_in_Flag=1;
			  }
		  }
		  else
		  {
			  g_u8Time20ms_3=0;
			  if(key3_in_Flag)
			  {
				  key3_in_Flag=0;
				 // send_image(gImage_8);
				  RR=0; GG=25; BB=0;
				  send_string_data();
			  }
		  }
		  if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)==0)	//SW4
		  {
			  g_u8Time20ms_4++;
			  if(g_u8Time20ms_4>1)
			  {
				  g_u8Time20ms_4=3;
				  key4_in_Flag=1;
			  }
		  }
		  else
		  {
			  g_u8Time20ms_4=0;
			  if(key4_in_Flag)
			  {
				  key4_in_Flag=0;
				  //send_image(gImage_9);
				  RR=25; GG=25; BB=25;
				  send_string_data();
			  }
		  }
		  if(HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin)==0)
		  {
			  g_u8Time20ms_5++;
			  if(g_u8Time20ms_5>1)
			  {
				  g_u8Time20ms_5=3;
				  key5_in_Flag=1;
			  }
		  }
		  else
		  {
			  g_u8Time20ms_5=0;
			  if(key5_in_Flag)
			  {
				  key5_in_Flag=0;
				  RR=0; GG=0; BB=0;
				  send_string_data();
				  //send_image(gImage_3);
				  /*
				  RR=255;GG=255;BB=255;
				  toggle_Flag=~toggle_Flag;
					if(toggle_Flag)
					{
						send_string_data_LK_V();
					}
					else
					{
						send_string_data_LK_H();
					}
					*/
			  }
		  }
	}
	if (htim->Instance == htim4.Instance)
	{
		////HAL_GPIO_TogglePin(LED_DO_GPIO_Port, LED_DO_Pin);
		////HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    	if(FLAG_RECEIVE_OK1)
    	{
    		FLAG_RECEIVE_OK1=0;
    		send_string_data1();
    		//send_string_data_LK_H();

    		/*
    		if(toggle_Flag)
    		{
    			send_string_data_LK_V();
    		}
    		else
    		{
    			send_string_data_LK_H();
    		}
    		toggle_Flag=~toggle_Flag;
    		*/
    		//send_string_data1();
    		//send_string_data1();
    		//send_string_data1();
    	}
	}
}

/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */
    //HAL_UART_Transmit(&huart1, (uint8_t *)aRxBuffer, 10,0xFFFF);
  if(huart==&huart1)
  {

  }

  if(huart==&huart3)
  {
		__HAL_TIM_SET_COUNTER(&htim2,0);
		Rxbuff3[Rx_count_UART3++] = cRxBuffer[0];
		if((Rxbuff3[0]=='R'))
		{
			__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim2);
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
		else
		{
			__HAL_TIM_SET_COUNTER(&htim2,0);
			__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim2);
			Rx_count_UART3=0;
		}
		/*
		if(Rx_count_UART3>=7)
		{
		  Rx_count_UART3=0;
		}
		*/
		HAL_UART_Receive_IT(&huart3,(uint8_t *)cRxBuffer,1);


  }




}
/******************************************/
void IOT_PROCESS(void)
{
	//if((Rxbuff3[0]=='R'))
    if((Rxbuff3[0]=='R')&&(Rxbuff3[Rx_LEN-1]=='*'))
    {
       FLAG_RECEIVE_OK1=1;
       for(i=0;i<Rx_LEN-1;i++)
       {
    	   if(Rxbuff3[i]=='R')
    	   {
    		   R_BIT=i;
    	   }
    	   else if(Rxbuff3[i]=='G')
    	   {
    		   G_BIT=i;
    	   }
    	   else if(Rxbuff3[i]=='B')
    	   {
    		   B_BIT=i;
    	   }
       }
       delta_RG=G_BIT-R_BIT-1;
       delta_GB=B_BIT-G_BIT-1;
       delta_BX=Rx_LEN-B_BIT-1-1;
       switch (delta_RG)
       {
           case 0x01:
           {
        	   RR=Rxbuff3[G_BIT-1]-0X30;
           }break;
           case 0x02:
           {
        	   RR=(Rxbuff3[G_BIT-1]-0X30)+(Rxbuff3[G_BIT-2]-0X30)*10;
           }break;
           case 0x03:
           {
        	   RR=(Rxbuff3[G_BIT-1]-0X30)+(Rxbuff3[G_BIT-2]-0X30)*10+(Rxbuff3[G_BIT-3]-0X30)*100;
           }break;
       }
       switch (delta_GB)
       {
           case 0x01:
           {
        	   GG=Rxbuff3[B_BIT-1]-0X30;
           }break;
           case 0x02:
           {
        	   GG=(Rxbuff3[B_BIT-1]-0X30)+(Rxbuff3[B_BIT-2]-0X30)*10;
           }break;
           case 0x03:
           {
        	   GG=(Rxbuff3[B_BIT-1]-0X30)+(Rxbuff3[B_BIT-2]-0X30)*10+(Rxbuff3[B_BIT-3]-0X30)*100;
           }break;
       }
       switch (delta_BX)
       {
           case 0x01:
           {
        	   BB=Rxbuff3[Rx_LEN-1-1]-0X30;
           }break;
           case 0x02:
           {
        	   BB=(Rxbuff3[Rx_LEN-1-1]-0X30)+(Rxbuff3[Rx_LEN-1-2]-0X30)*10;
           }break;
           case 0x03:
           {
        	   BB=(Rxbuff3[Rx_LEN-1-1]-0X30)+(Rxbuff3[Rx_LEN-1-2]-0X30)*10+(Rxbuff3[Rx_LEN-1-3]-0X30)*100;
           }break;
       }

       TX_BUFF_WENXIN[0]=RR;
       TX_BUFF_WENXIN[1]=GG;
       TX_BUFF_WENXIN[2]=BB;

    }
	else
	{
		Rx_count_UART3=0;
		FLAG_RECEIVE_OK1=0;
	}
}
/*
 * IOT.C
 *
 *  Created on: 2019�????????????????????????????????????12�????????????????????????????????????2�????????????????????????????????????
 *      Author: lwj
 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
