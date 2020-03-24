#include "pwm.h"

void Pwm_Init(void)
{
	GPIO_InitTypeDef  			GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef 			TIM_OCInitStruct;
	//	ʹ�ܶ�ʱ��14ʱ�ӣ�
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	//	ʹ��GPIOFʱ�ӣ�
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;		//��ʤ
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	//GPIOF9����ӳ�䵽��ʱ��14
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); 	//����LED0
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 84-1;			//84MHZ/84 = 1MHZ  Prescaler��Χ1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 500-1;		//��1MHZʱ��Ƶ���£���ʱ500us �Զ���װ�ؼĴ���ֵ
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStruct);	
	
	
	TIM_OCInitStruct.TIM_OCMode			= TIM_OCMode_PWM1;		//PWWM1ģʽ
	TIM_OCInitStruct.TIM_OCPolarity		= TIM_OCPolarity_Low;	//�������Ϊ�ߣ������Ч��ƽΪ�ߵ�ƽ
	TIM_OCInitStruct.TIM_OutputState	= TIM_OutputState_Enable;
	//	��ʼ������Ƚϲ���:	OC1��ʾͨ��1
	TIM_OC1Init(TIM14,&TIM_OCInitStruct);
	
	
	//ʹ��Ԥװ�ؼĴ����� 
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable); 
	
	//ʹ���Զ���װ�ص�Ԥװ�ؼĴ�������λ	
	TIM_ARRPreloadConfig(TIM14,ENABLE);
	
	//ʹ�ܶ�ʱ����
	TIM_Cmd(TIM14, ENABLE);
}