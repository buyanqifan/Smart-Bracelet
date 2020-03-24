#include "infrared.h"

void Infrared_Init(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct;    
	EXTI_InitTypeDef    EXTI_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStruct;
	
	//ʹ��SYSCFGʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//ʹ��GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	=	GPIO_Pin_8; 	//����
	GPIO_InitStruct.GPIO_Mode	=   GPIO_Mode_IN; 	//����ģʽ
	GPIO_InitStruct.GPIO_PuPd	=	GPIO_PuPd_UP;	//����
	GPIO_InitStruct.GPIO_Speed	=   GPIO_Speed_50MHz; //�ٶ�50MHZ

	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//����IO�����ж��ߵ�ӳ���ϵ,����ֿ�д      
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8);

	EXTI_InitStruct.EXTI_Mode		=	EXTI_Mode_Interrupt; //�ж�
	EXTI_InitStruct.EXTI_Line		=	EXTI_Line8;  //�ж���0
	EXTI_InitStruct.EXTI_Trigger	=   EXTI_Trigger_Falling; //�½���
	EXTI_InitStruct.EXTI_LineCmd	=   ENABLE; //�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�  
	EXTI_Init(&EXTI_InitStruct);	


	NVIC_InitStruct.NVIC_IRQChannel			= EXTI9_5_IRQn; //�жϺ�
	NVIC_InitStruct.NVIC_IRQChannelCmd		= ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority =0x02;	//��Ӧ���ȼ�
	NVIC_Init(&NVIC_InitStruct);

}
u32 ir_pluse_high_time(void)
{
	u32 t=0;
	
	while(PAin(8))
	{
		t++;
		delay_us(20); //20΢��
		
		if(t > 250)   //����5ms�����쳣
			break;
	}
	return t;
}


void EXTI9_5_IRQHandler(void)
{
	u32 t=0;
	u32 ir_bit=0;
	u8  ir_valed=0;
	u32 ir_data = 0;
	u8  ir_cunt=0;
	//�ж��Ƿ��ж���8
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		while(1)
		{
			if(PAin(8))  //�ȴ����ߵ�ƽ�����˵͵�ƽ
			{
				t = ir_pluse_high_time();
				if(t>=250)
					break;
				
				if(t>200 && t<250)  //4ms ~5ms
				{
					ir_valed = 1;   //ͬ����ͷ��Ч
					continue;
				}
				//���ߵ�ƽ����ʱ��Ϊ200~1000us����Ϊ����λΪ0��  560us��200~1000us
				else if(t>10 && t<50)
				{
					ir_bit = 0;
				}
				else if(t>60 && t<90)//���ߵ�ƽ����ʱ��Ϊ1200~1800us����Ϊ����λΪ1��  1680us��1200~1800us
				{
					ir_bit = 1;
				}
				
				if(ir_valed)
				{
					//��λ�����Ƶ���ir_data
					ir_data |=ir_bit<<(31-ir_cunt);
					
					ir_cunt++;
					
					if(ir_cunt >= 32)
					{
						printf("ir_data = %#X\n",ir_data);
						break;
					}
					
				
				}
				
				
			
			}
		
		}

	}
	
	//����жϱ�־λ
	EXTI_ClearITPendingBit(EXTI_Line8);
	

}