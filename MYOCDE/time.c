#include "time.h"

u32 led_flag = 0;


void Time1_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1、能定时器时钟。
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 16800-1;		//168MHZ/16800 = 10000HZ  Prescaler范围1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 2000;			//在10000HZ时钟频率下，用时0.2s 自动重装载寄存器值
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM1_UP_TIM10_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//3、启定时器中断，配置NVIC。
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4、设置 TIM4_DIER  允许更新中断
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	//5、使能定时器。
    TIM_Cmd(TIM1, ENABLE);
}











void Time2_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1、能定时器时钟。
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler范围1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 5000-1;	//在10000HZ时钟频率下，用时0.5s 自动重装载寄存器值
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM2_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//3、启定时器中断，配置NVIC。
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4、设置 TIM4_DIER  允许更新中断
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//5、使能定时器。
    TIM_Cmd(TIM2, ENABLE);
}
//////////////////////////
void Time3_Init(void)
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1、能定时器时钟。
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler范围1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 10000-1;	//在10000HZ时钟频率下，用时1s 自动重装载寄存器值
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= TIM3_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//3、启定时器中断，配置NVIC。
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4、设置 TIM4_DIER  允许更新中断
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//5、使能定时器。
    TIM_Cmd(TIM3, ENABLE);
}
//////////////////////////
void Time4_Init(void)
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1、能定时器时钟。
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler范围1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 10-1;		//在10000HZ时钟频率下，用时1ms 自动重装载寄存器值
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //分频因子
	//2、初始化定时器，配置ARR,PSC。
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM4_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//3、启定时器中断，配置NVIC。
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4、设置 TIM4_DIER  允许更新中断
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	//5、使能定时器。
    TIM_Cmd(TIM4, ENABLE);
}


void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET) //溢出中断
	{
		/*处理的程序段*/
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);
		
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update); //清除中断标志位
}





//6、编写中断服务函数。
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{
		/*处理的程序段*/
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_10);
		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位
}

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{

		/*处理的程序段*/
		GPIO_ToggleBits(GPIOE, GPIO_Pin_13);
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //清除中断标志位
}



//每隔1ms中断一次
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		/*处理的程序段*/
		led_flag++;
		if(led_flag == 9)
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_14); //亮
			led_flag = 0;
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_14); //灭
		}
		
		
		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位

}
	
	