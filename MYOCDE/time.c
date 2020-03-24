#include "time.h"

u32 led_flag = 0;


void Time1_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1���ܶ�ʱ��ʱ�ӡ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 16800-1;		//168MHZ/16800 = 10000HZ  Prescaler��Χ1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 2000;			//��10000HZʱ��Ƶ���£���ʱ0.2s �Զ���װ�ؼĴ���ֵ
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM1_UP_TIM10_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//3������ʱ���жϣ�����NVIC��
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4������ TIM4_DIER  ��������ж�
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	//5��ʹ�ܶ�ʱ����
    TIM_Cmd(TIM1, ENABLE);
}











void Time2_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1���ܶ�ʱ��ʱ�ӡ�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler��Χ1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 5000-1;	//��10000HZʱ��Ƶ���£���ʱ0.5s �Զ���װ�ؼĴ���ֵ
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM2_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//3������ʱ���жϣ�����NVIC��
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4������ TIM4_DIER  ��������ж�
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//5��ʹ�ܶ�ʱ����
    TIM_Cmd(TIM2, ENABLE);
}
//////////////////////////
void Time3_Init(void)
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1���ܶ�ʱ��ʱ�ӡ�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler��Χ1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 10000-1;	//��10000HZʱ��Ƶ���£���ʱ1s �Զ���װ�ؼĴ���ֵ
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= TIM3_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//3������ʱ���жϣ�����NVIC��
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4������ TIM4_DIER  ��������ж�
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//5��ʹ�ܶ�ʱ����
    TIM_Cmd(TIM3, ENABLE);
}
//////////////////////////
void Time4_Init(void)
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//1���ܶ�ʱ��ʱ�ӡ�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler	= 8400-1;	//84MHZ/8400 = 10000HZ  Prescaler��Χ1~65536 
	TIM_TimeBaseInitStruct.TIM_Period		= 10-1;		//��10000HZʱ��Ƶ���£���ʱ1ms �Զ���װ�ؼĴ���ֵ
	TIM_TimeBaseInitStruct.TIM_CounterMode	= TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;  	 //��Ƶ����
	//2����ʼ����ʱ��������ARR,PSC��
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	


	NVIC_InitStruct.NVIC_IRQChannel						= TIM4_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//3������ʱ���жϣ�����NVIC��
    NVIC_Init(&NVIC_InitStruct);	
	
	
	//4������ TIM4_DIER  ��������ж�
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	//5��ʹ�ܶ�ʱ����
    TIM_Cmd(TIM4, ENABLE);
}


void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET) //����ж�
	{
		/*����ĳ����*/
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);
		
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update); //����жϱ�־λ
}





//6����д�жϷ�������
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		/*����ĳ����*/
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_10);
		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //����жϱ�־λ
}

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{

		/*����ĳ����*/
		GPIO_ToggleBits(GPIOE, GPIO_Pin_13);
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //����жϱ�־λ
}



//ÿ��1ms�ж�һ��
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //����ж�
	{
		/*����ĳ����*/
		led_flag++;
		if(led_flag == 9)
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_14); //��
			led_flag = 0;
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_14); //��
		}
		
		
		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //����жϱ�־λ

}
	
	