#include "RTC.h"
#include "delay.h"
#include "OLED_I2C.h"
//��������

void Rtc_Init(u8 year, u8 month, u8 date, u8 hours, u8 minutes, u8 clock_init_flag)
{
	
	RTC_InitTypeDef	 RTC_InitStruct;
	RTC_TimeTypeDef  RTC_TimeStruct;
	RTC_DateTypeDef  RTC_DateStruc;
	//1��ʹ��PWRʱ�ӣ�
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	//2��ʹ�ܺ󱸼Ĵ�������:   
	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0);
	if(RTC_ReadBackupRegister(RTC_BKP_DR0) !=  clock_init_flag)
	{
	
		//3������RTCʱ��Դ��ʹ��RTCʱ�ӣ�
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RCC_RTCCLKCmd(ENABLE);
		//���ʹ��LSE��Ҫ��LSE��
		RCC_LSEConfig(RCC_LSE_ON);
		
		delay_ms(50); //��ʱ�ȴ�ʱ���ȶ�
		
		
		RTC_InitStruct.RTC_HourFormat	= RTC_HourFormat_24;	//24Сʱ��
		RTC_InitStruct.RTC_AsynchPrediv = 0x7F; 				//�첽��Ƶ
		RTC_InitStruct.RTC_SynchPrediv  = 0xFF;					//ͬ����Ƶ
		//4�� ��ʼ��RTC(ͬ��/�첽��Ƶϵ����ʱ�Ӹ�ʽ)��
		RTC_Init (&RTC_InitStruct);
		
		
		RTC_TimeStruct.RTC_H12		= RTC_H12_AM;
		RTC_TimeStruct.RTC_Hours	= hours; 
		RTC_TimeStruct.RTC_Minutes	= minutes;
		RTC_TimeStruct.RTC_Seconds	= 30;
		//5�� ����ʱ�䣺
		RTC_SetTime (RTC_Format_BIN,&RTC_TimeStruct);
		
		RTC_DateStruc.RTC_Year		= year;
		RTC_DateStruc.RTC_Month		= month;
		RTC_DateStruc.RTC_Date		= date;
		RTC_DateStruc.RTC_WeekDay	= RTC_Weekday_Thursday;
		//6���������ڣ�
		RTC_SetDate(RTC_Format_BIN,&RTC_DateStruc);
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0);
		
		//printf("�������� = 20%d�� %d�� %d�� %dʱ %d�� \n", year, month date, hours, minutes);
	}
	
}

void Rtc_Get(void)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	
	//printf("����:20%d-%d-%d  ",RTC_DateStruct.RTC_Year
	//					      ,RTC_DateStruct.RTC_Month
	//					      ,RTC_DateStruct.RTC_Date);
	OLED_ShowNum(4, 3, RTC_DateStruct.RTC_Year, 2, 1);
	OLED_ShowStr(46,3,"-",1);
	OLED_ShowNum(7, 3, RTC_DateStruct.RTC_Month, 2, 1);
	OLED_ShowStr(71,3,"-",1);
	OLED_ShowNum(10, 3, RTC_DateStruct.RTC_Date, 2, 1);
	
	//printf("ʱ��:%d:%d:%d\n",RTC_TimeStruct.RTC_Hours
	//						,RTC_TimeStruct.RTC_Minutes
	//						,RTC_TimeStruct.RTC_Seconds);
	OLED_ShowNum(4, 5, RTC_TimeStruct.RTC_Hours, 2, 2);
	OLED_ShowStr(49,5,":",2);
	OLED_ShowNum(7, 5, RTC_TimeStruct.RTC_Minutes, 2, 2);
	OLED_ShowStr(73,5,":",2);
	OLED_ShowNum(10, 5, RTC_TimeStruct.RTC_Seconds, 2, 2);
}

void Rtc_AlarmA(u8 date, u8 hours, u8 minutes)
{
	RTC_TimeTypeDef 	RTC_AlarmTime;
	RTC_AlarmTypeDef 	RTC_AlarmStruct;
	EXTI_InitTypeDef    EXTI_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStruct;

	//2���ر����ӣ�
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE); 
	
	
	RTC_AlarmTime.RTC_H12	=	RTC_H12_AM;
	RTC_AlarmTime.RTC_Hours	=	hours;
	RTC_AlarmTime.RTC_Minutes=	minutes;
	RTC_AlarmTime.RTC_Seconds=	00;	
	
	
	RTC_AlarmStruct.RTC_AlarmTime 			= RTC_AlarmTime;  //ʱ������
	RTC_AlarmStruct.RTC_AlarmMask 			= RTC_AlarmMask_None; //������λ	
	RTC_AlarmStruct.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;//����������
	RTC_AlarmStruct.RTC_AlarmDateWeekDay	= date;		//��������
	//3���������Ӳ�����
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmStruct);
	
	
	//4���������ӣ�
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);
	//5���������������жϣ�
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line17; 			//�ж���4
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//�ж�
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Rising;	//������
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//�ж���ʹ��
	//��ʼ�������жϣ����ô��������ȡ�
    EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= RTC_Alarm_IRQn;  	//ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//��Ӧ���ȼ�	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//ʹ��
	//�����жϷ��飨NVIC������ʹ���жϡ�
    NVIC_Init(&NVIC_InitStruct);
	
	printf("�������� = %d�� %dʱ %d��\n", date, hours, minutes);
}

//6����д�жϷ�������

void RTC_Alarm_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line17) == SET)     //�ж�����0�Ƿ���1
	{
		if( RTC_GetITStatus(RTC_IT_ALRA) == SET)
		{
			
			//�����¼�
			GPIO_SetBits(GPIOF, GPIO_Pin_8);
			
			RTC_ClearITPendingBit(RTC_IT_ALRA);
		}
		//�жϴ������
		EXTI_ClearITPendingBit(EXTI_Line17);
	}

}
