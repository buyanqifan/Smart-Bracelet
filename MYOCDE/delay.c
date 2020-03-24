#include "delay.h"

u32 my_nus = 21;  		//ÿ1us��21����
u32 my_nms = 21000;  	//ÿ1us��21000����

void Delay_Init(void)
{
	//��ʼ��Systick��ʱ����ʱ��Ƶ��168MHZ/8 = 21MHZ
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

}


//nus���ֵΪ798915
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = my_nus*nus - 1; //�����Զ���װ��ֵ
	SysTick->VAL  = 0;    			//VAL��Ϊ0
	
	SysTick->CTRL |= (1<<0);		//������ʱ��
	
	do
	{
		temp = SysTick->CTRL;
	}while(  (temp & (1<<0)) && !(temp & (1<<16)) );
	
	SysTick->CTRL &= ~(1<<0);		//�رն�ʱ��

}


//nms���ֵΪ798.915
void delay_ms(u32 nms)
{
	u32 temp;
	SysTick->LOAD = my_nms*nms - 1; //�����Զ���װ��ֵ
	SysTick->VAL  = 0;    			//VAL��Ϊ0
	
	SysTick->CTRL |= (1<<0);		//������ʱ��
	
	do
	{
		temp = SysTick->CTRL;
	}while(  (temp & (1<<0)) && !(temp & (1<<16)) );
	
	SysTick->CTRL &= ~(1<<0);		//�رն�ʱ��

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





