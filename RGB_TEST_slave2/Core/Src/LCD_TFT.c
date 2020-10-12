/*
 * LCD_TFT.c
 *
 *  Created on: 2020年3月26日
 *      Author: zhirong.su
 */
#include "main.h"
#include "gpio.h"
#include"LCD.h"
#define uchar unsigned char
#define uint  unsigned int
#define red    0xf800		//定义红色
#define blue   0x001f		//定义蓝色
#define green  0x07e0 	//定义绿色
#define white  0xffff 		//定义白色
#define black  0x0000 	//定义黑色
#define orange 0xfc08 	//定义橙色
#define yellow 0xffe0		//定义黄色
#define pink   0xf3f3 	//定义粉红色
#define purple 0xa1d6	 	//定义紫色
#define brown  0x8200	 //定义棕色
#define gray   0x8410	 	//定义灰色

void LCD_DA(u8 da)
{
	if((da & (1<<0))==(1<<0))
		 SET_D0;
	else
		 CLR_D0;
	if((da & (1<<1))==(1<<1))
		 SET_D1;
	else
		 CLR_D1;
	if((da & (1<<2))==(1<<2))
		 SET_D2;
	else
		 CLR_D2;
	if((da & (1<<3))==(1<<3))
		 SET_D3;
	else
		 CLR_D3;
	if((da & (1<<4))==(1<<4))
		 SET_D4;
	else
		 CLR_D4;
	if((da & (1<<5))==(1<<5))
		 SET_D5;
	else
		 CLR_D5;
	if((da & (1<<6))==(1<<6))
		 SET_D6;
	else
		 CLR_D6;
	if((da & (1<<7))==(1<<7))
		 SET_D7;
	else
		 CLR_D7;
}

void delay1(void)
{
	asm("nop");

}

//////////////////////////////////////////////////
void vSdCmd(uint8_t Command)       //send command
{

	  CLR_DC;

	  LCD_DA(Command);

	  CLR_CS;

	  CLR_WR;

	   delay1();

	    SET_WR;

		  SET_CS;
}
//////////////////////////////////////////////////
void vSdData(uint8_t Ddata)       //send command
{


	  SET_DC;

	  LCD_DA(Ddata);

	  CLR_CS;

	  CLR_WR;

		delay1();

		SET_WR;

	  SET_CS;
}
//////////////////////////////////////////////////////

//传16位数据，16位数据一起赋值
void data_out_16(uint data_16bit)
{
	uchar hdata,ldata;
	hdata=(data_16bit>>8)&0X00FF;
	ldata=data_16bit&0X00FF;
	vSdData(hdata);
	vSdData(ldata);
}



void lcd_Dot(uint16_t x, uint16_t y,uint16_t Fg_Color)

{
	  vSdCmd(0x2a);
      vSdData(x>>8); //
      vSdData(x);      //
      vSdData(x>>8); //
      vSdData(x);      //

      vSdCmd(0x2b);
      vSdData(y>>8); //
      vSdData(y);      //
      vSdData(y>>8); //
      vSdData(y);      //

      vSdCmd(0x2c);//
      vSdData(Fg_Color>>8);         //
      vSdData(Fg_Color);
}


void lcd_init(void)
{

	//PORT_LCD_RST=0;
	//lcd_gpio_config();

	CLR_RS;			//低电平：复位
	HAL_Delay(20);
	//PORT_LCD_RST=1;
	SET_RS;			//高电平：复位结束
	HAL_Delay(80);

	vSdCmd(0x11);	//开始初始化：
	vSdCmd(0x26);vSdData(0x04);	//设置GAMMA参数
	vSdCmd(0x36);vSdData(0xa8);	//行扫描顺序，列扫描顺序，横放/竖放    //设置：RAM写入方向从左向右、从上到下、数据顺序BGR
	vSdCmd(0xb1);vSdData(0x0e);vSdData(0x14);		//设置fr=61.7hz
	vSdCmd(0x3a);vSdData(0x05);		//数据格式16bit
	vSdCmd(0x29);	//开显示

	SET_BK;
	display_color(green);
	HAL_Delay(1000);
	//TFT_Disp128160(gImage_Peaks_logo);
}
//定义窗口坐标：开始坐标（XS,YS)以及窗口大小（x_total,y_total)
void lcd_address(uint XS,uint YS,uint x_total,uint y_total)
{
	int XE,YE;
	XE=XS+x_total-1;
	YE=YS+y_total-1;
	vSdCmd(0x2a);		// 设置X开始及结束的地址
	data_out_16(XS);	// X开始地址(16位）
	data_out_16(XE);	// X结束地址(16位）

	vSdCmd(0x2b);		// 设置Y开始及结束的地址
	data_out_16(YS);	// Y开始地址(16位）
	data_out_16(YE);	// Y结束地址(16位）

	vSdCmd(0x2c);	    // 写数据开始
}
/*--------------------------------------------------------------------------------------
-函数名称:void TFT_Disp128160(uchar *dat)
----------------------------------------------------------------------------------------
-函数功能:显示128*160大小图片
-入口参数:水平扫高位在前160*128bmp
-出口参数:无
---------------------------------------------------------------------------------------*/
void TFT_Disp128160(const uchar *dat)
{
    int i1,j;
//	横屏显示
	lcd_address(0,0,160,128);
	for(i1=0;i1<128;i1++)
	{
		for(j=0;j<160;j++)
		{
			vSdData(*dat++);
			vSdData(*dat++);
		}
	}
//	//竖屏显示
//	lcd_address(0,0,128,160);
//	for(i1=0;i1<160;i1++)
//	{
//		for(j=0;j<128;j++)
//		{
//		 vSdData(*dat++);
//		 vSdData(*dat++);
//		}
//	}
}

//将单色的8位的数据（代表8个像素点）转换成彩色的数据传输给液晶屏
void mono_data_out(uchar mono_data,uint font_color,uint back_color)
{
	uint k;
	for(k=0;k<8;k++)
	{
	 	if(mono_data&0x80)
		{
			data_out_16(font_color);	//当数据是1时，显示字体颜色
		}
		else
		{
			data_out_16(back_color);	//当数据是0时，显示底色
		}

		mono_data<<=1;
	}
}
void disp_ASCII_8x16(int x,int y, uchar number,int font_color,int back_color)
{
	int i1,j,k;
	k=0;
	lcd_address(x,y,8,16);
	for(i1=0;i1<16;i1++)		//高度16
	{
		for(j=0;j<1;j++)	//宽度 8
		{
			mono_data_out(table_asc[number][k],font_color,back_color);
			k++;
		}
	}
}
void disp_24x24(int x,int y,char *dp,int font_color,int back_color)
{
	int i1,j;
	lcd_address(x,y,24,24);
	for(i1=0;i1<24;i1++)		//高度21
	{
		for(j=0;j<3;j++)	//宽度 2*8=16
		{
			mono_data_out(*dp,font_color,back_color);
			dp++;
		}
	}
}
void disp_16x16(uint x,uint y,uchar *dp,uint font_color,uint back_color)
{
	int i1,j;
	lcd_address(x,y,16,16);
	for(i1=0;i1<16;i1++)		//高度21
	{
		for(j=0;j<2;j++)	//宽度 2*8=16
		{
			mono_data_out(*dp,font_color,back_color);
			dp++;
		}
	}
}

//显示全屏单一色彩
void display_color(uint color)
{
	int i1,j;

	lcd_address(0,0,160,128);
	for(i1=0;i1<128;i1++)
	{
		for(j=0;j<160;j++)
		{
			data_out_16(color);
		}
	}

	/*竖屏显示
	lcd_address(0,0,128,160);
	for(i1=0;i1<160;i1++)
	{
		for(j=0;j<128;j++)
		{
			data_out_16(color);
		}
	}
	*/
}
//=====================================================================

