#include "dht11.h"


void Dht11_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	//��GPIO G��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOG, &GPIO_InitStruct);
		

}

void Dht11_Mode(GPIOMode_TypeDef Mode)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//����9
	GPIO_InitStruct.GPIO_Mode	= Mode;				//ģʽѡ��
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOG, &GPIO_InitStruct);

}

//����0��ʾ��������������Ϊ�쳣
u8 Dht11_Start(void)
{
	u16 t = 0;
	
	
	Dht11_Mode(GPIO_Mode_OUT);
	
	//�����ź�
	PGout(9) = 1;							//GPIO_SetBits(GPIOG, GPIO_Pin_9);
	delay_us(5);
	PGout(9) = 0;
	delay_ms(20);
	PGout(9) = 1;
	delay_us(30);
	
	
	//��Ӧ�ź�
	Dht11_Mode(GPIO_Mode_IN);
	
	//���͵�ƽ����
	while( PGin(9) != 0)
	{
		delay_us(1);
		t++;
		if(t > 100)
			return 1;
	}

	//���ɵ͵�ƽ
	t = 0;
	while( PGin(9) == 0)
	{
		delay_us(1);
		t++;
		if(t > 100)
			return 2;
	}	
	
	//���˸ߵ�ƽ
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

//����0Ϊ�쳣
u8 Dht11_Read_Byte(void)
{
	u8 i, t = 0, data = 0 ;
	
	
	for(i=0; i<8; i++)
	{
		//���ɵ͵�ƽ
		t = 0;
		while( PGin(9) == 0)
		{
			delay_us(1);
			t++;
			if(t > 100)
			return 0;
		}	

		//��ʱ45us�����жϵ�ƽ
		delay_us(45);
		
		if(PGin(9) == 1)		//��ʾ�յ�1
		{
			data |= 1<<(7-i);

			//���˸ߵ�ƽ
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




