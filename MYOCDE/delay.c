#include "delay.h"

u32 my_nus = 21;  		//每1us数21个数
u32 my_nms = 21000;  	//每1us数21000个数

void Delay_Init(void)
{
	//初始化Systick定时器的时钟频率168MHZ/8 = 21MHZ
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

}


//nus最大值为798915
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = my_nus*nus - 1; //设置自动重装初值
	SysTick->VAL  = 0;    			//VAL置为0
	
	SysTick->CTRL |= (1<<0);		//启动定时器
	
	do
	{
		temp = SysTick->CTRL;
	}while(  (temp & (1<<0)) && !(temp & (1<<16)) );
	
	SysTick->CTRL &= ~(1<<0);		//关闭定时器

}


//nms最大值为798.915
void delay_ms(u32 nms)
{
	u32 temp;
	SysTick->LOAD = my_nms*nms - 1; //设置自动重装初值
	SysTick->VAL  = 0;    			//VAL置为0
	
	SysTick->CTRL |= (1<<0);		//启动定时器
	
	do
	{
		temp = SysTick->CTRL;
	}while(  (temp & (1<<0)) && !(temp & (1<<16)) );
	
	SysTick->CTRL &= ~(1<<0);		//关闭定时器

}


void delay_s(u32 ns)
{
	int i;
	
	for(i=0; i<ns; i++)
	{
		delay_ms(500);
		delay_ms(500);
	}
}





