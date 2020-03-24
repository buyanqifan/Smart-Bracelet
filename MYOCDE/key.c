#include "key.h"

void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//打开GPIO F组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//打开GPIO E组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//输出
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//输出
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	

}