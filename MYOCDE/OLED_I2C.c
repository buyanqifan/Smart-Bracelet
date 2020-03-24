/************************************************************************************
*  Copyright (c), 2014, HelTec Automatic Technology co.,LTD.
*            All rights reserved.
*
* Http:    www.heltec.cn
* Email:   cn.heltec@gmail.com
* WebShop: heltec.taobao.com
*
* File name: OLED_I2C.c
* Project  : HelTec.uvprij
* Processor: STM32F103C8T6
* Compiler : MDK fo ARM
* 
* Author : С��
* Version: 1.00
* Date   : 2014.4.8
* Email  : hello14blog@gmail.com
* Modification: none
* 
* Description:128*64�����OLED��ʾ�������ļ����������ڻ����Զ���(heltec.taobao.com)��SD1306����OLED_Iicͨ�ŷ�ʽ��ʾ��
*
* Others: none;
*
* Function List:
*	1. void I2C_Configuration(void) -- ����CPU��Ӳ��I2C
* 2. void I2C_WriteByte(uint8_t addr,uint8_t data) -- ��Ĵ�����ַдһ��byte������
* 3. void WriteCmd(unsigned char I2C_Command) -- д����
* 4. void WriteDat(unsigned char I2C_Data) -- д����
* 5. void OLED_Init(void) -- OLED����ʼ��
* 6. void OLED_SetPos(unsigned char x, unsigned char y) -- ������ʼ������
* 7. void OLED_Fill(unsigned char fill_Data) -- ȫ�����
* 8. void OLED_CLS(void) -- ����
* 9. void OLED_ON(void) -- ����
* 10. void OLED_OFF(void) -- ˯��
* 11. void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- ��ʾ�ַ���(�����С��6*8��8*16����)
* 12. void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N) -- ��ʾ����(������Ҫ��ȡģ��Ȼ��ŵ�codetab.h��)
* 13. void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]) -- BMPͼƬ
*
* History: none;
*
*************************************************************************************/

#include "OLED_I2C.h"
#include "delay.h"
#include "codetab.h"

extern u8 OLED_GRAM[128][8];


/*
D1 --- GND ---	PD15
D3 --- VCC ---  PD1
D5 --- SCK ---  PE8
D7 --- SDA ---  PE10

*/

void I2C_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//��GPIO D��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//��GPIO E��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_1|GPIO_Pin_15;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_8|GPIO_Pin_10;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	//���ŵ�Դ����
	OLED_VCC = 1; 
	OLED_GND = 0;
	
	//����
	OLED_SCK		=1;
	OLED_SDA_OUT	=1;
	
	
}

//����ģʽ��
void OLED_Iic_Mode(GPIOMode_TypeDef Mode)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_10;		//����9
	GPIO_InitStruct.GPIO_Mode	= Mode;	//���ģʽ
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//�������
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;	//50MHZ
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void OLED_Iic_Start(void)
{
	//SDAΪ���
	OLED_Iic_Mode(GPIO_Mode_OUT);
	//���߿���
	OLED_SCK 	= 1;
	OLED_SDA_OUT = 1;

	delay_us(5);
	OLED_SDA_OUT = 0;
	
	
	//�����߶��ӵͣ�ǯס����
	delay_us(5);
	OLED_SCK 	= 0;

}

void OLED_Iic_Stop(void)
{
	//SDAΪ���
	OLED_Iic_Mode(GPIO_Mode_OUT);
	//���߿���
	OLED_SCK 	= 0;
	OLED_SDA_OUT = 0;

	delay_us(5);
	OLED_SCK 	= 1;
	
	
	//�����߶����ߣ�ֹͣ�ź�
	delay_us(5);
	OLED_SDA_OUT = 1;

}


//ͨ��ackȷ�����������ƽ
void OLED_Iic_Send_Ack(u8 ack)
{
	//SDAΪ���
	OLED_Iic_Mode(GPIO_Mode_OUT);

	OLED_SCK = 0;
	//׼������
	if(ack == 1)
	{
		OLED_SDA_OUT = 1;
	}
	else
	{
		OLED_SDA_OUT = 0;
	}
	
	//�γ���������
	delay_us(5);
	OLED_SCK = 1;
	delay_us(5);
	OLED_SCK = 0;

}

//�ȷ���λ
void OLED_Iic_Send_Byte(u8 data)
{
	u8 i;
	//SDAΪ���
	OLED_Iic_Mode(GPIO_Mode_OUT);	
	
	OLED_SCK = 0;
	
	for(i=0; i<8; i++)
	{
		//׼������
		if( data & 1<<(7-i) )    // 
		{
			OLED_SDA_OUT = 1;
		}
		else
		{
			OLED_SDA_OUT = 0;
		}	
	
		//�γ���������
		delay_us(5);
		OLED_SCK = 1;
		delay_us(5);
		OLED_SCK = 0;		
		
	}
	
}

//����һλ����  ����0��ʾ������ЧӦ���ź�  ����1��ʾ������ЧӦ���ź�
u8 OLED_Iic_Recv_Ack(void)
{
	u8 ack;
	//SDAΪ���
	OLED_Iic_Mode(GPIO_Mode_IN);	
	
	OLED_SCK = 0;
	delay_us(5);
	OLED_SCK = 1;
	
	//�ж����ŵ�ƽֵ
	if(OLED_SDA_IN == 1)
	{
		ack = 1;
	}
	else
	{
		ack = 0;
	}
	
	delay_us(5);
	OLED_SCK = 0;

	return ack;
}





void I2C_WriteByte(uint8_t addr,uint8_t data)
{
	u8 ack;
	//�����ź�
	OLED_Iic_Start();

	

	//�����豸��ַ����ִ��д����
	OLED_Iic_Send_Byte(OLED_ADDRESS);
	ack = OLED_Iic_Recv_Ack();
	if(ack == 1)
	{
		printf("ack failure\n");
		return ;
	}
	

	//�����豸��ַ����ִ��д����
	OLED_Iic_Send_Byte(addr);
	ack = OLED_Iic_Recv_Ack();
	if(ack == 1)
	{
		printf("ack failure\n");
		return ;
	}	

	//�����豸��ַ����ִ��д����
	OLED_Iic_Send_Byte(data);
	ack = OLED_Iic_Recv_Ack();
	if(ack == 1)
	{
		printf("ack failure\n");
		return ;
	}	
	

	OLED_Iic_Stop();

}

void WriteCmd(unsigned char I2C_Command)//д����
{
	I2C_WriteByte(0x00, I2C_Command);
}

void WriteDat(unsigned char I2C_Data)//д����
{
	I2C_WriteByte(0x40, I2C_Data);
}

void OLED_Init(void)
{
	delay_ms(100); //�������ʱ����Ҫ
	
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //���ȵ��� 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}

void OLED_SetPos(unsigned char x, unsigned char y) //������ʼ������
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}

void OLED_Fill(unsigned char fill_Data)//ȫ�����
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}

void OLED_CLS(void)//����
{
	OLED_Fill(0x00);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : ��OLED�������л���
//--------------------------------------------------------------
void OLED_ON(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X14);  //������ɱ�
	WriteCmd(0XAF);  //OLED����
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : ��OLED���� -- ����ģʽ��,OLED���Ĳ���10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X10);  //�رյ�ɱ�
	WriteCmd(0XAE);  //OLED����
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); ch[] -- Ҫ��ʾ���ַ���; TextSize -- �ַ���С(1:6*8 ; 2:8*16)
// Description    : ��ʾcodetab.h�е�ASCII�ַ�,��6*8��8*16��ѡ��
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); N:������codetab.h�е�����
// Description    : ��ʾcodetab.h�еĺ���,16*16����
//--------------------------------------------------------------
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=32*N;
	OLED_SetPos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
	OLED_SetPos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- ��ʼ������(x0:0~127, y0:0~7); x1,y1 -- ���Խ���(������)������(x1:1~128,y1:1~8)
// Description    : ��ʾBMPλͼ
//--------------------------------------------------------------
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
}

//OLED��ʾ����
void OLED_ShowNum(unsigned char x, unsigned char y, int num, u8 N, unsigned char TextSize)
{
	
	u8 j=0;
	u8 n[6]={0};
	x=x*8;
	n[0]=(num/10000)%10;
	n[1]=(num/1000)%10;
	n[2]=(num/100)%10;
	n[3]=(num/10)%10;
	n[4]=num%10;
	n[5]='\0';
	for(j=0;j<5;j++) n[j]=n[j]+16+32;
	OLED_ShowStr(x, y, &n[5-N], TextSize);
}


//���õ��ˢ����ʾ
void OLED_Refresh_Gram(void)
{
	u8 i, n;
	for(i=0;i<8;i++)
	{
		WriteCmd(0xb0+i);//����ҳ��ַ��0~7��???
		WriteCmd(0x02);//������ʾλ�á��е͵�ַ��ƫ����2��?
		WriteCmd(0x10);//������ʾλ�á��иߵ�ַ??????
		for(n=0;n<128;n++)WriteDat(OLED_GRAM[n][i]);
	}
}

//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

