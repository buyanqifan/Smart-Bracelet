#include "pwm.h"

void Pwm_Init(void)
{
	GPIO_InitTypeDef  			GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef 			TIM_OCInitStruct;
	//	使能定时器14时钟：
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	//	使能GPIOF时钟：
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;		//得胜
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	//GPIOF9复用映射到定时器14
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); 	//控制LED0
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 84-1;			//84MHZ/84 = 1MHZ  Prescaler范围1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 500-1;		//在1MHZ时钟频率下，用时500us 自动重装载寄存器值
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStruct);	
	
	
	TIM_OCInitStruct.TIM_OCMode			= TIM_OCMode_PWM1;		//PWWM1模式
	TIM_OCInitStruct.TIM_OCPolarity		= TIM_OCPolarity_Low;	//输出极性为高，输出有效电平为高电平
	TIM_OCInitStruct.TIM_OutputState	= TIM_OutputState_Enable;
	//	初始化输出比较参数:	OC1表示通道1
	TIM_OC1Init(TIM14,&TIM_OCInitStruct);
	
	
	//使能预装载寄存器： 
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable); 
	
	//使能自动重装载的预装载寄存器允许位	
	TIM_ARRPreloadConfig(TIM14,ENABLE);
	
	//使能定时器。
	TIM_Cmd(TIM14, ENABLE);
}