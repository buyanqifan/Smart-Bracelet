#include "sr04.h"

/*
PA2 ---- TRIG
PA3 ---- ECHO
*/

void Sr04_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	
	//打开GPIO F组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2;  		//引脚2
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//输出
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_3;  		//引脚3
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//输出
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉	 
	GPIO_Init(GPIOA, &GPIO_InitStruct);		
	



	//1、能定时器时钟。
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 84-1;		//1MHZ
	TIM_TimeBaseInitStruct.TIM_Period		= 50000;	//1us计一个数
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	
	
	//5、不使能定时器。
    TIM_Cmd(TIM4, DISABLE);
}

//测量距离
u16 Get_Sr04_Distance(void)
{
	u16 value;
	
	//启动信号
	TRIG = 0;
	delay_us(5);
	TRIG = 1;
	delay_us(15);
	TRIG = 0;
	
	TIM4->CNT = 0;		//定时器的CNT为0
	while(ECHO == 0);    //等待高电平到来
	TIM_Cmd(TIM4, ENABLE);
	
	while(ECHO == 1);    //等待高电平到来
	value = TIM4->CNT;
	TIM_Cmd(TIM4, DISABLE);
	
	
	value = value/58;
	
	return value;

}


