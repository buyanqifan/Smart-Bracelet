#include "key.h"

void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//��GPIO F��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//��GPIO E��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//���� 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//���� 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	

}