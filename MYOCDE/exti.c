#include "exti.h"

/*

选择按键PA0输入做为中断源

*/
void Exti_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	EXTI_InitTypeDef  EXTI_InitStruct;
	NVIC_InitTypeDef  NVIC_InitStruct;
	//打开GPIO A组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	//打开GPIO A组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	
	//使能SYSCFG时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//输出
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//输出
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	//设置IO口与中断线的映射关系。
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource4);
	
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line0; 			//中断线0
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//中断
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//下降沿
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//中断线使能
	//初始化线上中断，设置触发条件等。
    EXTI_Init(&EXTI_InitStruct);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line2; 			//中断线2
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//中断
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//下降沿
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//中断线使能
	//初始化线上中断，设置触发条件等。
    EXTI_Init(&EXTI_InitStruct);

	EXTI_InitStruct.EXTI_Line	= EXTI_Line3; 			//中断线3
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//中断
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//下降沿
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//中断线使能
	//初始化线上中断，设置触发条件等。
    EXTI_Init(&EXTI_InitStruct);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line4; 			//中断线4
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//中断
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//下降沿
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//中断线使能
	//初始化线上中断，设置触发条件等。
    EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI0_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//配置中断分组（NVIC），并使能中断。
    NVIC_Init(&NVIC_InitStruct);
	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI2_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//配置中断分组（NVIC），并使能中断。
    NVIC_Init(&NVIC_InitStruct);	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI3_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//配置中断分组（NVIC），并使能中断。
    NVIC_Init(&NVIC_InitStruct);		
	
	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI4_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 1;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//配置中断分组（NVIC），并使能中断。
    NVIC_Init(&NVIC_InitStruct);		
	
	
	
}

//这个就是CPU响应的中断程序
void EXTI0_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line0) == SET)     //判断中线0是否置1
	{
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);			//灯状态变更
		
		//中断处理程序
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

//这个就是CPU响应的中断程序
void EXTI2_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line2) == SET)     //判断中线0是否置1
	{
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_10);			//灯状态变更
		
		//中断处理程序
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

//这个就是CPU响应的中断程序
void EXTI3_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line3) == SET)     //判断中线0是否置1
	{
		
		GPIO_ToggleBits(GPIOE, GPIO_Pin_13);			//灯状态变更
		
		//中断处理程序
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}


//这个就是CPU响应的中断程序
void EXTI4_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line4) == SET)     //判断中线0是否置1
	{
		
		GPIO_ToggleBits(GPIOE, GPIO_Pin_14);			//灯状态变更
		
		//中断处理程序
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
