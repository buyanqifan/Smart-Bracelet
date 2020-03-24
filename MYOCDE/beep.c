#include "beep.h"

void Beep_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//打开GPIO F组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_8;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//输出
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	

}