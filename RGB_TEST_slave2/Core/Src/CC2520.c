/*
 * CC2520.c
 *
 *  Created on: 2020年4月10日
 *      Author: zhirong.su
 */

#include "main.h"
#include "CC2520.h"
#include"crc16.h"
#define num_timer_over 250  //250
#define TIME_OUT_SET	900    //  15分钟离线才判断为掉线

//设置ID时，需满足版本号为V1.3版本
#define ID_VER_H 	49	//1   ID版本号13
#define ID_VER_L	51	//3   ID版本号13

uchar ADDR,CRC_LOW, CRC_HIG;
uchar     *ptr;          //缓存数组指针
uchar rc1timer, rc2timer;
uchar rx1_byte_count, rx1_byte_limit, tx1_byte_count, tx1_byte_limit;
uchar rx1_buff[256];
uchar tx1_buff[256];

uchar rx2_byte_count,rx2_byte_limit, tx2_byte_count, tx2_byte_limit;
uchar rx2_buff[256];
uchar tx2_buff[256];
uint time_display;
uint time_delay;
uint8 CC2520_PSDU[1+CC2520_PSDU_Len];
//bit flag_rx1_start, flag_rx2_start, flag_rx1_end, flag_rx2_end;
//bit flag_bus_free;
uchar fre_set,fre_set_temp, fre_set_change ;
uchar	CC2520_WorkFreqIndex;
uchar CC2520_Source_PANID[2] = {0xcc,0xcc};
uchar CC2520_Source_ShortAddr[2] = {0xcc,0xcc};
uchar CC2520_Source_IEEEAddr[8] = {0x12,0x34,0x56,0x7c,0xcc,0xcc,0xcc,0xcc};
uchar CC2520_Destination_PANID[2] = {0xcc,0xcc};
uchar CC2520_Destination_ShortAddr[2] = {0xcc,0xcc};
uchar CC2520_Destination_IEEEAddr[8] = {0x12,0x34,0x56,0x7c,0xcc,0xcc,0xcc,0xcc};

uint  cc2520_rx_cycle;
uchar cc2520_rx_count, cc2520_senser_count;
uchar cc2520_rx[12][16];
uint time_cc2520_no_rx;
uchar ID_ARM_UP_A[6]={0x14,0x19,0x05,0x24,0x00,0x03};
uchar	RF_BadCycCnt;

void host_ack(uchar *dp, uint time_set);

//const uint8	FreqTable[16]={0x0B, 14, 17,20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56};	//2.4G+5M~50m
//const uint8	FreqTable[16]={0x0B, 18, 43, 68, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56};
//const uchar	FreqTable[32]={0x0B, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53,  56,
//							 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104};

const uint8	FreqTable[64]=
						  {11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56,
						   59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, //27ge
						   7,  10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52,
						   55, 58, 61, 64, 67, 70, 73, 76, 79, 82, 85, 88,//28ge
						   9,  12, 15, 18, 21, 24, 27, 30, 33}; //9ge

/*const uchar FreqTable[32]=
						 {86, 76,14, 17,20 , 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53,
						  56, 59, 62, 65, 68, 71, 74, 77, 81, 83, 7, 9, 11, 16, 21, 31};

*/
const uchar	CC2520_PackHead[24]=
{
	0x00,	//data_len
	(PAN_ID_COMPRESSION<<6)|(ACKNOWLEDGMENT_REQUEST<<5)|(FRAME_PENDING<<4)|(SECURITY_ENABLE<<3)|(FRAME_TYPE_DATA<<0),
	(SOURCE_ADDRESSING_MODE<<6)|(FRAME_VERSION<<4)|(DEST_ADDRESSING_MODE<<2),
	SEQUENCE_NUMBER,
	0xcc, 0xcc,	//CC2520_Destination_PANID
	0x12,0x34,0x56,0x7c,0xcc,0xcc,0xcc,0xcc,	//CC2520_Destination_IEEEAddr
	0xcc, 0xcc,	//CC2520_Source_PANID
	0x12,0x34,0x56,0x7c,0xcc,0xcc,0xcc,0xcc,	//CC2520_Source_IEEEAddr
};

uint32	rxd_ok=0;
uint32	rxd_err=0;


void crc_check(uchar *ptr,uchar len)	//	CRC16  多项式： X^16+X^15+X^2+1
{
	uint crc=0xffff;
	uchar i_crc,j_crc;
	for(i_crc=0;i_crc<len;i_crc++)
	{
		crc^=ptr[i_crc];
		for(j_crc=0;j_crc<8;j_crc++)
		{
			if(crc&0x0001)
				crc=(crc>>1)^0xA001;	//0xA001
			else
				crc=crc>>1;
		}
	}
	CRC_LOW=crc&0x00ff;
	CRC_HIG=crc>>8;
}
///////////////////////////////////////////////////////////
void CC2520_GPIO_Config(void)
{



	/* 停止信号 FLASH: CS引脚高电平*/
	CSN_ON();
}
/////////////////////////////////////////////////////////////
void SET_CC2520_GP0_Input(void)
{

}
/////////////////////////////////////////////////////////////
void SET_CC2520_GP1_Input(void)
{

}

void CC2520_IntoActMode_Init(void)	//在进入act模式时要做初始化
{
	/*CC2520_WriteReg(0x30,0x32);
	//CC2520_WriteReg(TXPOWER,0xF7);
	CC2520_WriteReg(0x36,0xF8);
	CC2520_WriteRAM(0x00, 0x46, 0x85);
	CC2520_WriteRAM(0x00, 0x47, 0x14);*/
	CC2520_WriteReg(0x30,0xF7);	//TXPOWER
	CC2520_WriteReg(0x31,0x94);	//TXCTRL
	CC2520_WriteReg(0x36,0xF8);
	CC2520_WriteRAM(0x00, 0x46, 0x85);
	CC2520_WriteRAM(0x00, 0x47, 0x14);

	CC2520_WriteRAM(0x00, 0x4A, 0x3F);//调整接收灵敏度
	CC2520_WriteRAM(0x00, 0x4C, 0x7B);//FSCTRL	调整合成器电流
	CC2520_WriteRAM(0x00, 0x4F, 0x2B);
	CC2520_WriteRAM(0x00, 0x53, 0x11);
	CC2520_WriteRAM(0x00, 0x56, 0x10);
	CC2520_WriteRAM(0x00, 0x57, 0x0E);
	CC2520_WriteRAM(0x00, 0x58, 0x03);

	CC2520_WriteRAM(0x00, 0x4A, 0x3F);
	CC2520_WriteRAM(0x00, 0x4C, 0x5A);
	CC2520_WriteRAM(0x00, 0x4F, 0x2B);
	CC2520_WriteRAM(0x00, 0x53, 0x11);
	CC2520_WriteRAM(0x00, 0x56, 0x10);
	CC2520_WriteRAM(0x00, 0x57, 0x0E);
	CC2520_WriteRAM(0x00, 0x58, 0x03);
}
//---------------------------CC2520射频芯片初始化函数--------------------------------------
void CC2520_Init(uchar freq)
{

	POWER_ON_CC2520();		//zigbee电源控制
	HAL_Delay(30);
	POWER_OFF_CC2520();	//zigbee电源开启

	RESET_OFF();
	HAL_Delay(10);
	RESET_ON();		//zigbee复位
	HAL_Delay(10);

	CC2520_Command(CMD_SXOSCON);	//zigbee晶振开启
	//HAL_Delay(10);

	CC2520_WriteRAM(0x03,RAM_PANID,  CC2520_Source_PANID[0]);
	CC2520_WriteRAM(0x03,RAM_PANID+1, CC2520_Source_PANID[1]);	//设置PANID
	CC2520_ReadRAM(3,RAM_PANID);

	CC2520_WriteRAM(0x03, RAM_IEEEADR,CC2520_Source_IEEEAddr[0]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+1,CC2520_Source_IEEEAddr[1]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+2, CC2520_Source_IEEEAddr[2]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+3, CC2520_Source_IEEEAddr[3]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+4, CC2520_Source_IEEEAddr[4]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+5, CC2520_Source_IEEEAddr[5]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+6, CC2520_Source_IEEEAddr[6]);
	CC2520_WriteRAM(0x03, RAM_IEEEADR+7, CC2520_Source_IEEEAddr[7]);	//设置IEEEADR
	CC2520_ReadRAM(3,RAM_IEEEADR);


	CC2520_WriteReg(FRMFILT0,0x0D);
	CC2520_WriteReg(GPIOCTRL0,0x2A);
	CC2520_WriteReg(GPIOCTRL1,0x27);
	CC2520_WriteReg(GPIOCTRL,0x3F);	//使能上拉20K
	//CC2520_WriteReg(FREQCTRL,0x0B);	//设置频段，2407M，
	//if (freq>63)	freq=0;

	//CC2520_WriteReg(FREQCTRL, FreqTable[freq]);

	CC2520_WriteReg(FREQCTRL, freq);		//设置频率

	CC2520_WorkFreqIndex = freq;

	CC2520_WriteReg(FSMCTRL,0);	//no timeout  有限元状态机

	CC2520_Command(CMD_SFLUSHRX);	//清除RX缓存
	CC2520_Command(CMD_SFLUSHTX);	//清除TX缓存

	CC2520_IntoActMode_Init();

	//HAL_Delay(10);
	//CC2520_SetRxMode();


}
//---------------------------CC2520寄存器读取函数--------------------------------------
uint8 CC2520_ReadReg(uint8  addr)
{
	uint16 value;
	CSN_OFF();
	SPI_Write(addr|REG_READ);
	value = SPI_Read();
	CSN_ON();
	return value;
}
//---------------------------CC2520寄存器写入函数--------------------------------------
void  CC2520_WriteReg(uint8  addr, uint8 value)
{
	CSN_OFF();
	SPI_Write(addr|REG_WRITE);
	SPI_Write(value);
	CSN_ON();
}
//---------------------------CC2520 RAM 读取函数--------------------------------------
uint8 CC2520_ReadRAM(uint8 addrH,uint8 addrL)
{
	uint8 value;
	CSN_OFF();
	SPI_Write(addrH|MEM_READ);
	SPI_Write(addrL);
	value = SPI_Read();
	CSN_ON();
	return value;
}
//---------------------------CC2520 RAM 写入函数--------------------------------------
void CC2520_WriteRAM(uint8 addrH,uint8 addrL, uint8 value)
{
	CSN_OFF();
	SPI_Write(addrH|MEM_WRITE);
	SPI_Write(addrL);
	SPI_Write(value);
	CSN_ON();
}


//---------------------------CC2520 TXFIFIO 写入函数--------------------------------------
void  CC2520_WriteTXFIFO(uchar txd_len)//3.5ms@len=16
{
	uint8 i;

	CC2520_Command(CMD_SFLUSHTX);//80us

	CSN_OFF();

	SPI_Write(TXFIFO_WRITE); //80us


	SPI_Write(23+txd_len+2);//80us


	for(i=1;i<24;i++)
	{
		SPI_Write(CC2520_PackHead[i]);//1.9ms@len=16
	}

	for(i=0;i<txd_len;i++)
	{
		SPI_Write(CC2520_PSDU[i]);//1.3ms@len=16
	}

	SPI_Write(0x55);//80us
	SPI_Write(0xaa);//80us
	CSN_ON();

	delay_us1(40);;//32us

}

//---------------------------CC2520命令函数--------------------------------------
void  CC2520_Command(uint8 cmd)
{
	CSN_OFF();   //NRF_CS=0	片选有效
	SPI_Write(cmd);
	CSN_ON();    //NRF_CS=1
}

//---------------------------CC2520 设置接收函数--------------------------------------
void CC2520_SetRxMode(void)
{
	//CC2520_Command(CMD_SRFOFF);
	CC2520_Command(CMD_SRXON);
}
//---------------------------CC2520 判断接收函数--------------------------------------
uint8 CC2520_RxPacket(void)
{
	uchar	ok=0;

    if((HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin)==0)&&(HAL_GPIO_ReadPin(CC2520_GP1_GPIO_GPIO_Port,CC2520_GP1_GPIO_Pin)!=0))
		ok++;
    if((HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin)==0)&&(HAL_GPIO_ReadPin(CC2520_GP1_GPIO_GPIO_Port,CC2520_GP1_GPIO_Pin)!=0))
		ok++;
    if((HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin)==0)&&(HAL_GPIO_ReadPin(CC2520_GP1_GPIO_GPIO_Port,CC2520_GP1_GPIO_Pin)!=0))
   	 	ok++;
    if (ok>1)
	{
		//HAL_Delay(1);
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

		return TRUE;
	}
	return FALSE;
}


//---------------------------CC2520 发送函数--------------------------------------
void CC2520_TxPacket(void)
{
	uint32	i=0;
	uchar	hig_cnt;
	//CC2520_Command(CMD_SRFOFF);
	CC2520_Command(CMD_STXON);
	HAL_Delay(1);
	//for (i=0; i<2500;i++);
	for (i=0; i<200;i++)
	{
		hig_cnt = 0;
		if (HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin))
			hig_cnt++;
		if (HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin))
			hig_cnt++;
		if (HAL_GPIO_ReadPin(CC2520_GP0_GPIO_GPIO_Port,CC2520_GP0_GPIO_Pin))
			hig_cnt++;
		if (hig_cnt>1)	break;
	}
	if (i==200)	RF_BadCycCnt++;
	//else
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

//--------------------------SPI初始化函数---------------------------
void CC2520_SPI_Init(void)
{
	SCLK_OFF();
	CSN_ON();
}
//--------------------------SPI单字节读取函数---------------------------
uint8 SPI_Read(void)//30us
{
	uint8 i,rxdata, hig_cnt;
	rxdata = 0x00;

	//SET_CC2520_GP0_Input();
	//SET_CC2520_GP1_Input();

	for (i = 0;i < 8;i++)
	{
		rxdata = rxdata<<1;
		SCLK_ON();
		delay_us1(12);
		hig_cnt = 0;

		if (MISO_IN())	hig_cnt++;
		if (MISO_IN())	hig_cnt++;
		if (MISO_IN())	hig_cnt++;
		if (hig_cnt>1)
        {
			rxdata |= 0x01;
		}
		else
		{
			rxdata &= ~0x01;
		}
		SCLK_OFF();
		delay_us1(12);
	 }

	 return rxdata;
}
//----------------------------SPI单字节写入函数---------------------------
void SPI_Write(uint8 txdata)  //SPI=100kHz
{
	uint8 i;

	for (i = 0;i < 8;i++)
	{
		if (txdata&0x80)
		{
			MOSI_ON();
		}
		else
		{
			MOSI_OFF();
		}
		SCLK_ON();
		delay_init(72);
		delay_init(72);
		delay_init(72);
		delay_init(72);
		//delay_init(10);
		//delay_us1(1);//延时4.7us
		txdata = txdata<<1;
		SCLK_OFF();
		delay_init(72);
		delay_init(72);
		delay_init(72);
		delay_init(72);
		//delay_us1(1);//延时4.7us
	}
}
//----------------------------SPI单字节写入读取函数---------------------------
uint8  SPI_WriteRead(uint8 txdata)
{
	uint8 i,rxdata;
	rxdata = 0x00;

	for (i = 0;i < 8;i++)
	{
		rxdata = rxdata<<1;

		if (txdata&0x80)
		{
			MOSI_ON();
		}
		else
		{
			MOSI_OFF();
		}
		SCLK_ON();

		if (MISO_IN())
		{
			rxdata |= 0x01;
		}
		else
		{
			rxdata &= ~0x01;
		}
		SCLK_OFF();
		txdata = txdata<<1;
	 }
	 return rxdata;
}


void CC2520_Search(void)
{
	uchar time_search_set;
	uchar i,j;
	for(i=0;i<12;i++)
	{
		for(j=0;j<16;j++)
			cc2520_rx[i][j]=0;
	}


	time_search_set=60;
	while(time_display<time_search_set)
	{

		/*
		if((ARM_id_change | BUSBAR_id_change)==0)		//所有senser已经搜索完成
		{
			break;
		}

		if(fre_set<32 && !flag_64channel)
		{
			if(fre_set_change>31)
			{
				fre_set_change=0;
			}
		}
		else
		{
			time_search_set=120;
			if(fre_set_change>63)
			{
				fre_set_change=0;
			}
		}
		*/
		CC2520_Init(fre_set_change);
		CC2520_IntoActMode_Init();


	}



	CC2520_Init(fre_set);
	CC2520_IntoActMode_Init();
	CC2520_SetRxMode();		//设置为接收模式
	time_display=0;


}
//---------------------------CC2520 RXFIFIO 读取函数--------------------------------------

void CC2520_ReadRXFIFO(void)
{
	uint8 i;
	CSN_OFF();
	SPI_Write(RXFIFO_READ);
	CC2520_PSDU[0] = SPI_Read(); //CC2520_PSDU[0]表示收到的数据包长度，实测值为41
	if (CC2520_PSDU[0]>127)
		CC2520_PSDU[0]=127;
	for(i=0;i<CC2520_PSDU[0];i++)
	{
		CC2520_PSDU[1+i] = SPI_Read();
	}
	CSN_ON();
	CC2520_Command(CMD_SFLUSHRX);
}

void cc2520_data_load()		//数据判断装载  是本机配对senser接收   非本机的丢弃
{
	uchar i;

	if(CC2520_PSDU[29]==ID_ARM_UP_A[0])
	{
		delay_us1(4);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		for(i=24;i<CC2520_PSDU[0]-1 ;i++)
		{
			cc2520_rx[0][i-24]=CC2520_PSDU[i];
		}
		host_ack(ID_ARM_UP_A, 12100);
		/*
		for(i=0;i<16 ;i++)
		{
			tx1_buff[i]=CC2520_PSDU[24+i];
		}
		DE_RE=1;
		tx1_byte_count=0;
		tx1_byte_limit=16;
		PIE1=0X30;
		TXEN1=1;
		*/
	}

	CC2520_PSDU[29]=0; CC2520_PSDU[28]=0; CC2520_PSDU[27]=0;
	CC2520_PSDU[26]=0; CC2520_PSDU[25]=0; CC2520_PSDU[24]=0;
}

void cc2520_rx_app()
{
	uchar x,y;
	uchar cc2520_data[16];

	//for(x=0;x<cc2520_senser_count;x++)
	x=0;
	{
		if(cc2520_rx[x][5]==ID_ARM_UP_A[0])
		{
			for(y=0;y<16 ;y++)
			{
				cc2520_data[y]=cc2520_rx[x][y];
				cc2520_rx[x][y]=0;
			}
			if(cc2520_data[6]==0x01)		//接收温度数据
			{
				ptr=cc2520_data;
				crc_check(ptr,14);
				if(cc2520_data[14]==CRC_LOW  && cc2520_data[15]==CRC_HIG)
				{
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
					flag_cc2520_ok=1;
					time_cc2520_no_rx=0;
				}
			}
		}
	}
	cc2520_data[6]=0;
}

void cc2520_rx_timeout_test()
{
	;
}


void host_ack(uchar *dp, uint time_set)
{
	CC2520_PSDU[0]=*(dp+5);
	CC2520_PSDU[1]=*(dp+4);
	CC2520_PSDU[2]=*(dp+3);
	CC2520_PSDU[3]=*(dp+2);
	CC2520_PSDU[4]=*(dp+1);
	CC2520_PSDU[5]=*(dp+0);

	CC2520_PSDU[6]=0x01;
	CC2520_PSDU[7]=0X04;
	CC2520_PSDU[8]=time_set>>8;
	CC2520_PSDU[9]=time_set;
	CC2520_PSDU[10]=0xff;
	CC2520_PSDU[11]=0xff;

	ptr=CC2520_PSDU;
	crc_check(ptr,12);
	CC2520_PSDU[12]=CRC_LOW;
	CC2520_PSDU[13]=CRC_HIG;
	CC2520_WriteTXFIFO(14);
	CC2520_TxPacket();
	CC2520_SetRxMode();		//设置为接收模式

}
/*
void temp_send()			//温度发送
{
		CC2520_PSDU[0]=0x14;
		CC2520_PSDU[1]=0x19;
		CC2520_PSDU[2]=0x05;
		CC2520_PSDU[3]=0x24;
		CC2520_PSDU[4]=0x00;
		CC2520_PSDU[5]=0x03;
		CC2520_PSDU[6]=0x01;
		CC2520_PSDU[7]=0x06;
		CC2520_PSDU[8]=0x05;
		CC2520_PSDU[9]=0x50;
		CC2520_PSDU[10]=0x00;
		CC2520_PSDU[11]=0x00;
		CC2520_PSDU[12]=0x17;
		CC2520_PSDU[13]=0x70;
		ptr=CC2520_PSDU;
		crc_check(ptr,14);
		CC2520_PSDU[14]=CRC_LOW;
		CC2520_PSDU[15]=CRC_HIG;

		POWER_ON_CC2520();
		HAL_Delay(10);

		CC2520_WriteTXFIFO(16);
		CC2520_TxPacket();
		HAL_Delay(20);
		POWER_OFF_CC2520();
		//MOSI_OFF();
		//MOSI_OFF();
}
*/
void temp_send()			//温度发送
{
	uchar i;
	//uchar fre_temp;
	/*
	for(i=0;i<6;i++)
	{
		CC2520_PSDU[i]=ADD_BUFF[i];
	}
	CC2520_PSDU[6]=0x01;
	CC2520_PSDU[7]=6;
	CC2520_PSDU[8]=temp_h;
	CC2520_PSDU[9]=temp_l;
	CC2520_PSDU[10]=0;
	CC2520_PSDU[11]=(time_send_delay-1000)/400;
	CC2520_PSDU[12]=0x00;
	CC2520_PSDU[13]=time_send_period_set/1000;
	*/
	CC2520_PSDU[0]=0x15;
	CC2520_PSDU[1]=0x19;
	CC2520_PSDU[2]=0x05;
	CC2520_PSDU[3]=0x24;
	CC2520_PSDU[4]=0x00;
	CC2520_PSDU[5]=0x03;
	CC2520_PSDU[6]=0x02;
	CC2520_PSDU[7]=0x06;
	CC2520_PSDU[8]=0x05;
	CC2520_PSDU[9]=0x50;
	CC2520_PSDU[10]=0x38;
	CC2520_PSDU[11]=0x02;
	CC2520_PSDU[12]=0x2A;
	CC2520_PSDU[13]=0x40;
	ptr=CC2520_PSDU;
	crc_check(ptr,14);
	CC2520_PSDU[14]=CRC_LOW;
	CC2520_PSDU[15]=CRC_HIG;
	CC2520_Command(CMD_SXOSCON);	//开启装载到发送结束约12ms
	delay_init(72);
	delay_init(72);
	delay_init(72);
	CSN_OFF();
	HAL_Delay(1);
	//while(!MISO_IN());
	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	CSN_ON();
	CC2520_WriteTXFIFO(16);
	CC2520_TxPacket();	//开启发送到完成时间约7ms
	delay_init(72);
	CC2520_SetRxMode();		//设置为接收模式开启
	time_delay=0;
	i=0;
	while(time_delay<50)		//最长允许开启100ms
	{

		if(CC2520_RxPacket())
		{
			CC2520_ReadRXFIFO();
			i=1;				//退出接收等待后，如i=1，则判断为有接收
			break;
		}
	}

	CC2520_Command(CMD_SXOSCOFF);

	if(i==1)			//如i=1，则判断为有接收
	{

		for(i=0;i<16 ;i++)
		{
			CC2520_PSDU[i]=CC2520_PSDU[i+24];
		}
		if(CC2520_PSDU[0]==1)
	  	{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			FLAG_1=1;
			FLAG_2=0;
			FLAG_3=0;
			FLAG_4=0;
			FLAG_5=0;
		  	//time_reset=0;
	  		ptr=CC2520_PSDU;
			crc_check(ptr,14);

			if(CC2520_PSDU[14]==CRC_LOW  && CC2520_PSDU[15]==CRC_HIG)		//校验ok
			{
				if(CC2520_PSDU[6]==0x03)
				{
					HAL_Delay(80);					//延时待定
					CC2520_Command(CMD_SXOSCON);	//开启装载到发送结束约12ms
					CSN_OFF();
					HAL_Delay(10);
					//while(!MISO_IN());
					CSN_ON();
					CC2520_WriteTXFIFO(16);
					CC2520_TxPacket();	//原包回传
					CC2520_Command(CMD_SXOSCOFF);

				}

			}
	  	}
		else if(CC2520_PSDU[0]==2)
	  	{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			FLAG_1=0;
			FLAG_2=1;
			FLAG_3=0;
			FLAG_4=0;
			FLAG_5=0;
		  	//time_reset=0;
	  		ptr=CC2520_PSDU;
			crc_check(ptr,14);

			if(CC2520_PSDU[14]==CRC_LOW  && CC2520_PSDU[15]==CRC_HIG)		//校验ok
			{
				if(CC2520_PSDU[6]==0x03)
				{
					HAL_Delay(80);					//延时待定
					CC2520_Command(CMD_SXOSCON);	//开启装载到发送结束约12ms
					CSN_OFF();
					HAL_Delay(10);
					//while(!MISO_IN());
					CSN_ON();
					CC2520_WriteTXFIFO(16);
					CC2520_TxPacket();	//原包回传
					CC2520_Command(CMD_SXOSCOFF);

				}

			}
	  	}
		else if(CC2520_PSDU[0]==3)
	  	{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			FLAG_1=0;
			FLAG_2=0;
			FLAG_3=1;
			FLAG_4=0;
			FLAG_5=0;
		  	//time_reset=0;
	  		ptr=CC2520_PSDU;
			crc_check(ptr,14);

			if(CC2520_PSDU[14]==CRC_LOW  && CC2520_PSDU[15]==CRC_HIG)		//校验ok
			{
				if(CC2520_PSDU[6]==0x03)
				{
					HAL_Delay(80);					//延时待定
					CC2520_Command(CMD_SXOSCON);	//开启装载到发送结束约12ms
					CSN_OFF();
					HAL_Delay(10);
					//while(!MISO_IN());
					CSN_ON();
					CC2520_WriteTXFIFO(16);
					CC2520_TxPacket();	//原包回传
					CC2520_Command(CMD_SXOSCOFF);

				}

			}
	  	}
		else if(CC2520_PSDU[0]==4)
	  	{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			FLAG_1=0;
			FLAG_2=0;
			FLAG_3=0;
			FLAG_4=1;
			FLAG_5=0;
		  	//time_reset=0;
	  		ptr=CC2520_PSDU;
			crc_check(ptr,14);

			if(CC2520_PSDU[14]==CRC_LOW  && CC2520_PSDU[15]==CRC_HIG)		//校验ok
			{
				if(CC2520_PSDU[6]==0x03)
				{
					HAL_Delay(80);					//延时待定
					CC2520_Command(CMD_SXOSCON);	//开启装载到发送结束约12ms
					CSN_OFF();
					HAL_Delay(10);
					//while(!MISO_IN());
					CSN_ON();
					CC2520_WriteTXFIFO(16);
					CC2520_TxPacket();	//原包回传
					CC2520_Command(CMD_SXOSCOFF);

				}

			}
	  	}
		else if(CC2520_PSDU[0]==5)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			FLAG_1=0;
			FLAG_2=0;
			FLAG_3=0;
			FLAG_4=0;
			FLAG_5=1;
		}
	}

}




