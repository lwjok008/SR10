/*
 * WS2812.c
 *
 *  Created on: Mar 24, 2020
 *      Author: lwj
 */

#include "main.h"
//#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"

void WS2812_BYTE(uint8_t x);
void WS2812_BYTE_1(uint8_t x);
void RGB_open(void);
void RGB_PWM(void);
void RGB_open1(void);
void RGB_PWM1(void);
//============================================
//ws2812
//============================================
uint8_t LED_DAT;//可位操作的数据发送暂存变量声明

uint8_t RR,GG,BB; //RGB灰度值全局变量声明
uint8_t RR1,GG1,BB1;
uint8_t RR2,GG2,BB2;
//============================================
//code0
//============================================
void delay_250us(void)
{
	delay_us1(1);
}
//============================================
//code1
//============================================
void delay_750us()
{
	delay_us1(3);
}
//=============================================
//reset_ws2812
//=============================================
void reset_ws2812()
{
	CLR_WS2812DATA_ME;
	delay_us1(300);
	//SET_WS2812DATA_ME;
}
void reset_ws2812_1()
{
	CLR_WS2812DATA;
	delay_us1(300);
	//SET_WS2812DATA_ME;
}
/*********************************************/
/*
void send_72_WS2812_BYTE(void)
{
    uint16_t count,a;
    	CLR_WS2812DATA_ME;
        delay(100);
		for(a=0;a<72;a++)
		{
			WS2812_BYTE(TX1[a]);
		}
        SET_WS2812DATA_ME;
}
*/
/***********************************************************************/
void WS2812_BYTE_1(uint8_t x)
{
	uint8_t y;
	for(y=0;y<8;y++)
	{
		if(x&0x80)
		{
            SET_WS2812DATA;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            CLR_WS2812DATA;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
		}
		else
		{
            SET_WS2812DATA;
            asm("nop");asm("nop");asm("nop");
            CLR_WS2812DATA;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
		}
		x<<=1;
	}
    //delay50us();
}
/***********************************************************************/
void WS2812_BYTE(uint8_t x)
{
	uint8_t y;
	for(y=0;y<8;y++)
	{
		if(x&0x80)
		{
            SET_WS2812DATA_ME;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            CLR_WS2812DATA_ME;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
		}
		else
		{
            SET_WS2812DATA_ME;
            asm("nop");asm("nop");asm("nop");
            CLR_WS2812DATA_ME;
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");

            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
            asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
		}
		x<<=1;
	}
    //delay50us();
}
//============================================
//发送RGB数据
//============================================
void send_single_data()  //数据格式G7-G0-R7-R0-B7-B0
{
   LED_DAT=GG;
   WS2812_BYTE(LED_DAT);
   LED_DAT=RR;
   WS2812_BYTE(LED_DAT);
   LED_DAT=BB;
   WS2812_BYTE(LED_DAT);
}
void  send_string_data()
{
    uint8_t i;
    for(i=0;i<size;i++)
    {
        send_single_data();
    }
    reset_ws2812();
}
//============================================
//发送RGB数据
//============================================
void send_single_data1()  //数据格式G7-G0-R7-R0-B7-B0
{
   LED_DAT=GG;
   WS2812_BYTE_1(LED_DAT);
   LED_DAT=RR;
   WS2812_BYTE_1(LED_DAT);
   LED_DAT=BB;
   WS2812_BYTE_1(LED_DAT);
}

void send_single_data_RGB(uchar RA,uchar GA,uchar BA)  //数据格式G7-G0-R7-R0-B7-B0
{
   //LED_DAT=GG;
   WS2812_BYTE_1(GA);
  // LED_DAT=RR;
   WS2812_BYTE_1(RA);
   //LED_DAT=BB;
   WS2812_BYTE_1(BA);
}
void  send_string_data1()
{
    uint8_t i;
    for(i=0;i<72;i++)
    {
        send_single_data1();
    }
    reset_ws2812_1();
}
void  send_image(const uchar *data)
{
    uint8_t i;
    for(i=0;i<72;i++)
    {
    	   //LED_DAT=GG;
    	   WS2812_BYTE_1(*data++);
    	  // LED_DAT=RR;
    	   WS2812_BYTE_1(*data++);
    	   //LED_DAT=BB;
    	   WS2812_BYTE_1(*data++);
    }
    reset_ws2812_1();
}
void  send_string_data_LK_H()	//水平
{
    uint8_t i;
    RR1=RR;GG1=GG;BB1=BB;
    for(i=0;i<72;i++)
    {
    	if((((i+1)%12)==0)||((i%12)==0)||(i<2)||(((i+2)%12)==0)||(((i-1)%12)==0))
    	{
    		RR=RR1;GG=0;BB=0;
    	}
    	else if((((i-2)%6)==0)||(((i-3)%6)==0)||(i==2)||(i==3))
    	{
    		RR=0;GG=GG1;BB=0;
    	}
    	else
    	{
    		RR=0;GG=0;BB=BB1;
    	}
        send_single_data1();
    }

}

void  send_string_data_LK_V()	//垂直
{
    uint8_t i;
    RR1=RR;GG1=GG;BB1=BB;
    RR=RR1;GG=0;BB=0;
    for(i=0;i<24;i++)
    {
        send_single_data1();
    }
    RR=0; GG=GG1; BB=0;
    for(i=0;i<24;i++)
    {
        send_single_data1();
    }
    RR=0;GG=0;BB=BB1;
    for(i=0;i<24;i++)
    {
        send_single_data1();
    }
    reset_ws2812_1();
}
void RGB_open() //RGB闪烁
{
    uint16_t t;
    RR=255; GG=0; BB=0;
    send_string_data();
    for(t=speed1;t>0;t--)HAL_Delay(100);

    RR=0; GG=255; BB=0;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=0; GG=0; BB=255;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=255; BB=0;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=0; BB=255;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=0; GG=255; BB=255;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=255; BB=255;
    send_string_data();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

}

void RGB_open1() //RGB闪烁
{
    uint16_t t;
    RR=255; GG=0; BB=0;
    send_string_data1();
    for(t=speed1;t>0;t--)HAL_Delay(100);

    RR=0; GG=255; BB=0;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=0; GG=0; BB=255;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=255; BB=0;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=0; BB=255;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=0; GG=255; BB=255;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

    RR=255; GG=255; BB=255;
    send_string_data1();
    for(t=speed1;t>0;t--)  HAL_Delay(100);

}
void RGB_close() //RGB闪烁
{
    uint16_t t;
    RR=0; GG=0; BB=0;
    send_string_data();
    for(t=speed1;t>0;t--)HAL_Delay(100);

}
void RGB_close1() //RGB闪烁
{
    uint16_t t;
    RR=0; GG=0; BB=0;
    send_string_data1();
    for(t=speed1;t>0;t--)HAL_Delay(100);

}
//================RGB呼吸灯=============================
void RGB_PWM()
{
    uint16_t i;
    RR=0;GG=0;BB=0;
    for(i=0;i<256;i++)//红色渐亮
    {
        send_string_data();//发送RGB灰度数据
        reset_ws2812();
        HAL_Delay(1);
        RR++;
    }
    RR=255; GG=0; BB=0;
    for(i=0; i<256; i++) //红色渐灭
    {
        send_string_data(); //发送RGB灰度数据
        reset_ws2812();
        HAL_Delay(1);
        RR--;
    }
/****************************************************/
    RR=0; GG=0; BB=0;
    for(i=0; i<256; i++) //绿色渐亮
    {
       send_string_data(); //发送RGB灰度数据
       reset_ws2812();
       HAL_Delay(1);
       GG++;
    }
   RR=0; GG=255; BB=0;
   for(i=0; i<256; i++) //绿色渐灭
   {
      send_string_data(); //发送RGB灰度数据
      reset_ws2812();
      HAL_Delay(1);
      GG--;
    }
/****************************************************/
   RR=0; GG=0; BB=0;
   for(i=0; i<256; i++) //蓝色渐亮
   {
      send_string_data(); //发送RGB灰度数据
      reset_ws2812();
      HAL_Delay(1);
      BB++;
    }
    RR=0; GG=0; BB=255;
    for(i=0; i<256; i++) //蓝色渐灭
   {
      send_string_data(); //发送RGB灰度数据
      reset_ws2812();
      HAL_Delay(1);
      BB--;
   }
/****************************************************/
   RR=0; GG=0; BB=0;
   for(i=0; i<256; i++) //白色渐亮
   {
      send_string_data(); //发送RGB灰度数据
      reset_ws2812();
      HAL_Delay(1);
      RR++;
      GG++;
      BB++;
   }
   RR=255; GG=255; BB=255;
   for(i=0; i<256; i++) //白色渐灭
   {
      send_string_data(); //发送RGB灰度数据
      reset_ws2812();
      HAL_Delay(1);
      RR--;
      GG--;
      BB--;
   }
  RR=0; GG=0; BB=0;
 }

//================RGB呼吸灯=============================
void RGB_PWM1()
{
    uint16_t i;
    RR=0;GG=0;BB=0;
    for(i=0;i<256;i++)//红色渐亮
    {
        send_string_data1();//发送RGB灰度数据
        reset_ws2812_1();
        HAL_Delay(1);
        RR++;
    }
    RR=255; GG=0; BB=0;
    for(i=0; i<256; i++) //红色渐灭
    {
        send_string_data1(); //发送RGB灰度数据
        reset_ws2812_1();
        HAL_Delay(1);
        RR--;
    }
/****************************************************/
    RR=0; GG=0; BB=0;
    for(i=0; i<256; i++) //绿色渐亮
    {
       send_string_data1(); //发送RGB灰度数据
       reset_ws2812_1();
       HAL_Delay(1);
       GG++;
    }
   RR=0; GG=255; BB=0;
   for(i=0; i<256; i++) //绿色渐灭
   {
      send_string_data1(); //发送RGB灰度数据
      reset_ws2812_1();
      HAL_Delay(1);
      GG--;
    }
/****************************************************/
   RR=0; GG=0; BB=0;
   for(i=0; i<256; i++) //蓝色渐亮
   {
      send_string_data1(); //发送RGB灰度数据
      reset_ws2812_1();
      HAL_Delay(1);
      BB++;
    }
    RR=0; GG=0; BB=255;
    for(i=0; i<256; i++) //蓝色渐灭
   {
      send_string_data1(); //发送RGB灰度数据
      reset_ws2812_1();
      HAL_Delay(1);
      BB--;
   }
/****************************************************/
   RR=0; GG=0; BB=0;
   for(i=0; i<256; i++) //白色渐亮
   {
      send_string_data1(); //发送RGB灰度数据
      reset_ws2812_1();
      HAL_Delay(1);
      RR++;
      GG++;
      BB++;
   }
   RR=255; GG=255; BB=255;
   for(i=0; i<256; i++) //白色渐灭
   {
      send_string_data1(); //发送RGB灰度数据
      reset_ws2812_1();
      HAL_Delay(1);
      RR--;
      GG--;
      BB--;
   }
  RR=0; GG=0; BB=0;
 }
/*
//===================================================
//main
//===================================================
void main()
{
     delay_ms(50);
     while(1)
     {
          RGB_open();
          RGB_PWM();
     }
}
*/
