#include "beep.h"

void Beep_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//��GPIO F��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_8;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	

}