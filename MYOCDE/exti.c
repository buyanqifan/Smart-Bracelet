#include "exti.h"

/*

ѡ�񰴼�PA0������Ϊ�ж�Դ

*/
void Exti_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	EXTI_InitTypeDef  EXTI_InitStruct;
	NVIC_InitTypeDef  NVIC_InitStruct;
	//��GPIO A��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	//��GPIO A��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	
	//ʹ��SYSCFGʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//���� 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_IN;		//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//���� 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	//����IO�����ж��ߵ�ӳ���ϵ��
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource4);
	
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line0; 			//�ж���0
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//�ж�
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//�½���
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�
    EXTI_Init(&EXTI_InitStruct);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line2; 			//�ж���2
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//�ж�
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//�½���
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�
    EXTI_Init(&EXTI_InitStruct);

	EXTI_InitStruct.EXTI_Line	= EXTI_Line3; 			//�ж���3
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//�ж�
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//�½���
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�
    EXTI_Init(&EXTI_InitStruct);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line4; 			//�ж���4
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//�ж�
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Falling;	//�½���
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�
    EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI0_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//�����жϷ��飨NVIC������ʹ���жϡ�
    NVIC_Init(&NVIC_InitStruct);
	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI2_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//�����жϷ��飨NVIC������ʹ���жϡ�
    NVIC_Init(&NVIC_InitStruct);	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI3_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//�����жϷ��飨NVIC������ʹ���жϡ�
    NVIC_Init(&NVIC_InitStruct);		
	
	
	
	NVIC_InitStruct.NVIC_IRQChannel						= EXTI4_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 1;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//�����жϷ��飨NVIC������ʹ���жϡ�
    NVIC_Init(&NVIC_InitStruct);		
	
	
	
}

//�������CPU��Ӧ���жϳ���
void EXTI0_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line0) == SET)     //�ж�����0�Ƿ���1
	{
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);			//��״̬���
		
		//�жϴ������
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

//�������CPU��Ӧ���жϳ���
void EXTI2_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line2) == SET)     //�ж�����0�Ƿ���1
	{
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_10);			//��״̬���
		
		//�жϴ������
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

//�������CPU��Ӧ���жϳ���
void EXTI3_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line3) == SET)     //�ж�����0�Ƿ���1
	{
		
		GPIO_ToggleBits(GPIOE, GPIO_Pin_13);			//��״̬���
		
		//�жϴ������
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}


//�������CPU��Ӧ���жϳ���
void EXTI4_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line4) == SET)     //�ж�����0�Ƿ���1
	{
		
		GPIO_ToggleBits(GPIOE, GPIO_Pin_14);			//��״̬���
		
		//�жϴ������
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
