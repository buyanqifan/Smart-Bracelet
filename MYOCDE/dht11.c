#include "dht11.h"


void Dht11_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	//打开GPIO G组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//输出
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOG, &GPIO_InitStruct);
		

}

void Dht11_Mode(GPIOMode_TypeDef Mode)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= Mode;				//模式选择
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOG, &GPIO_InitStruct);

}

//返回0表示启动正常，其它为异常
u8 Dht11_Start(void)
{
	u16 t = 0;
	
	
	Dht11_Mode(GPIO_Mode_OUT);
	
	//启动信号
	PGout(9) = 1;							//GPIO_SetBits(GPIOG, GPIO_Pin_9);
	delay_us(5);
	PGout(9) = 0;
	delay_ms(20);
	PGout(9) = 1;
	delay_us(30);
	
	
	//响应信号
	Dht11_Mode(GPIO_Mode_IN);
	
	//检测低电平到来
	while( PGin(9) != 0)
	{
		delay_us(1);
		t++;
		if(t > 100)
			return 1;
	}

	//过渡低电平
	t = 0;
	while( PGin(9) == 0)
	{
		delay_us(1);
		t++;
		if(t > 100)
			return 2;
	}	
	
	//过滤高电平
	t = 0;
	while( PGin(9) == 1)
	{
		delay_us(1);
		t++;
		if(t > 100)
			return 3;
	}		
	
	return 0;
	
}

//返回0为异常
u8 Dht11_Read_Byte(void)
{
	u8 i, t = 0, data = 0 ;
	
	
	for(i=0; i<8; i++)
	{
		//过渡低电平
		t = 0;
		while( PGin(9) == 0)
		{
			delay_us(1);
			t++;
			if(t > 100)
			return 0;
		}	

		//延时45us，再判断电平
		delay_us(45);
		
		if(PGin(9) == 1)		//表示收到1
		{
			data |= 1<<(7-i);

			//过滤高电平
			t = 0;
			while( PGin(9) == 1)
			{
				delay_us(1);
				t++;
				if(t > 100)
					return 0;
			}	
		}
		
	
	}
	

	return data;
}

u8 Dht11_Data(u8 *data)
{
	u8 i;
	
	for(i=0; i<5; i++)
	{
		data[i] = Dht11_Read_Byte();
	}
	
	if(data[4] == data[0]+data[1]+data[2]+data[3])
	{
		return 0;
	}

	return 1;

}




