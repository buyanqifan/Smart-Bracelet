#include "infrared.h"

void Infrared_Init(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct;    
	EXTI_InitTypeDef    EXTI_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStruct;
	
	//使能SYSCFG时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//使能GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	=	GPIO_Pin_8; 	//引脚
	GPIO_InitStruct.GPIO_Mode	=   GPIO_Mode_IN; 	//输入模式
	GPIO_InitStruct.GPIO_PuPd	=	GPIO_PuPd_UP;	//上拉
	GPIO_InitStruct.GPIO_Speed	=   GPIO_Speed_50MHz; //速度50MHZ

	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//设置IO口与中断线的映射关系,必须分开写      
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8);

	EXTI_InitStruct.EXTI_Mode		=	EXTI_Mode_Interrupt; //中断
	EXTI_InitStruct.EXTI_Line		=	EXTI_Line8;  //中断线0
	EXTI_InitStruct.EXTI_Trigger	=   EXTI_Trigger_Falling; //下降沿
	EXTI_InitStruct.EXTI_LineCmd	=   ENABLE; //中断线使能
	//初始化线上中断，设置触发条件等。  
	EXTI_Init(&EXTI_InitStruct);	


	NVIC_InitStruct.NVIC_IRQChannel			= EXTI9_5_IRQn; //中断号
	NVIC_InitStruct.NVIC_IRQChannelCmd		= ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority =0x02;	//响应优先级
	NVIC_Init(&NVIC_InitStruct);

}
u32 ir_pluse_high_time(void)
{
	u32 t=0;
	
	while(PAin(8))
	{
		t++;
		delay_us(20); //20微秒
		
		if(t > 250)   //大于5ms数据异常
			break;
	}
	return t;
}


void EXTI9_5_IRQHandler(void)
{
	u32 t=0;
	u32 ir_bit=0;
	u8  ir_valed=0;
	u32 ir_data = 0;
	u8  ir_cunt=0;
	//判断是否中断线8
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		while(1)
		{
			if(PAin(8))  //等待到高电平，过滤低电平
			{
				t = ir_pluse_high_time();
				if(t>=250)
					break;
				
				if(t>200 && t<250)  //4ms ~5ms
				{
					ir_valed = 1;   //同步码头有效
					continue;
				}
				//若高电平持续时间为200~1000us内则为数据位为0：  560us在200~1000us
				else if(t>10 && t<50)
				{
					ir_bit = 0;
				}
				else if(t>60 && t<90)//若高电平持续时间为1200~1800us内则为数据位为1：  1680us在1200~1800us
				{
					ir_bit = 1;
				}
				
				if(ir_valed)
				{
					//将位数据移到到ir_data
					ir_data |=ir_bit<<(31-ir_cunt);
					
					ir_cunt++;
					
					if(ir_cunt >= 32)
					{
						printf("ir_data = %#X\n",ir_data);
						break;
					}
					
				
				}
				
				
			
			}
		
		}

	}
	
	//清除中断标志位
	EXTI_ClearITPendingBit(EXTI_Line8);
	

}