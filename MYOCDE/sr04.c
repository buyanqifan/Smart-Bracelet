#include "sr04.h"

/*
PA2 ---- TRIG
PA3 ---- ECHO
*/

void Sr04_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	
	//��GPIO F��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2;  		//����2
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_3;  		//����3
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����	 
	GPIO_Init(GPIOA, &GPIO_InitStruct);		
	



	//1���ܶ�ʱ��ʱ�ӡ�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 84-1;		//1MHZ
	TIM_TimeBaseInitStruct.TIM_Period		= 50000;	//1us��һ����
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	
	
	//5����ʹ�ܶ�ʱ����
    TIM_Cmd(TIM4, DISABLE);
}

//��������
u16 Get_Sr04_Distance(void)
{
	u16 value;
	
	//�����ź�
	TRIG = 0;
	delay_us(5);
	TRIG = 1;
	delay_us(15);
	TRIG = 0;
	
	TIM4->CNT = 0;		//��ʱ����CNTΪ0
	while(ECHO == 0);    //�ȴ��ߵ�ƽ����
	TIM_Cmd(TIM4, ENABLE);
	
	while(ECHO == 1);    //�ȴ��ߵ�ƽ����
	value = TIM4->CNT;
	TIM_Cmd(TIM4, DISABLE);
	
	
	value = value/58;
	
	return value;

}


