#include "RTC.h"
#include "delay.h"
#include "OLED_I2C.h"
//配置日历

void Rtc_Init(u8 year, u8 month, u8 date, u8 hours, u8 minutes, u8 clock_init_flag)
{
	
	RTC_InitTypeDef	 RTC_InitStruct;
	RTC_TimeTypeDef  RTC_TimeStruct;
	RTC_DateTypeDef  RTC_DateStruc;
	//1、使能PWR时钟：
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	//2、使能后备寄存器访问:   
	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0);
	if(RTC_ReadBackupRegister(RTC_BKP_DR0) !=  clock_init_flag)
	{
	
		//3、配置RTC时钟源，使能RTC时钟：
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RCC_RTCCLKCmd(ENABLE);
		//如果使用LSE，要打开LSE：
		RCC_LSEConfig(RCC_LSE_ON);
		
		delay_ms(50); //延时等待时钟稳定
		
		
		RTC_InitStruct.RTC_HourFormat	= RTC_HourFormat_24;	//24小时制
		RTC_InitStruct.RTC_AsynchPrediv = 0x7F; 				//异步分频
		RTC_InitStruct.RTC_SynchPrediv  = 0xFF;					//同步分频
		//4、 初始化RTC(同步/异步分频系数和时钟格式)：
		RTC_Init (&RTC_InitStruct);
		
		
		RTC_TimeStruct.RTC_H12		= RTC_H12_AM;
		RTC_TimeStruct.RTC_Hours	= hours; 
		RTC_TimeStruct.RTC_Minutes	= minutes;
		RTC_TimeStruct.RTC_Seconds	= 30;
		//5、 设置时间：
		RTC_SetTime (RTC_Format_BIN,&RTC_TimeStruct);
		
		RTC_DateStruc.RTC_Year		= year;
		RTC_DateStruc.RTC_Month		= month;
		RTC_DateStruc.RTC_Date		= date;
		RTC_DateStruc.RTC_WeekDay	= RTC_Weekday_Thursday;
		//6、设置日期：
		RTC_SetDate(RTC_Format_BIN,&RTC_DateStruc);
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0);
		
		//printf("设置日期 = 20%d年 %d月 %d日 %d时 %d分 \n", year, month date, hours, minutes);
	}
	
}

void Rtc_Get(void)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	
	//printf("日期:20%d-%d-%d  ",RTC_DateStruct.RTC_Year
	//					      ,RTC_DateStruct.RTC_Month
	//					      ,RTC_DateStruct.RTC_Date);
	OLED_ShowNum(4, 3, RTC_DateStruct.RTC_Year, 2, 1);
	OLED_ShowStr(46,3,"-",1);
	OLED_ShowNum(7, 3, RTC_DateStruct.RTC_Month, 2, 1);
	OLED_ShowStr(71,3,"-",1);
	OLED_ShowNum(10, 3, RTC_DateStruct.RTC_Date, 2, 1);
	
	//printf("时间:%d:%d:%d\n",RTC_TimeStruct.RTC_Hours
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

	//2、关闭闹钟：
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE); 
	
	
	RTC_AlarmTime.RTC_H12	=	RTC_H12_AM;
	RTC_AlarmTime.RTC_Hours	=	hours;
	RTC_AlarmTime.RTC_Minutes=	minutes;
	RTC_AlarmTime.RTC_Seconds=	00;	
	
	
	RTC_AlarmStruct.RTC_AlarmTime 			= RTC_AlarmTime;  //时间设置
	RTC_AlarmStruct.RTC_AlarmMask 			= RTC_AlarmMask_None; //无掩码位	
	RTC_AlarmStruct.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;//按日期来闹
	RTC_AlarmStruct.RTC_AlarmDateWeekDay	= date;		//设置日期
	//3、配置闹钟参数：
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmStruct);
	
	
	//4、开启闹钟：
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);
	//5、开启配置闹钟中断：
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);
	
	EXTI_InitStruct.EXTI_Line	= EXTI_Line17; 			//中断线4
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;	//中断
	EXTI_InitStruct.EXTI_Trigger= EXTI_Trigger_Rising;	//上升沿
	EXTI_InitStruct.EXTI_LineCmd= ENABLE;				//中断线使能
	//初始化线上中断，设置触发条件等。
    EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel						= RTC_Alarm_IRQn;  	//通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2;			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2;			//响应优先级	
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//使能
	//配置中断分组（NVIC），并使能中断。
    NVIC_Init(&NVIC_InitStruct);
	
	printf("设置闹钟 = %d日 %d时 %d分\n", date, hours, minutes);
}

//6、编写中断服务函数：

void RTC_Alarm_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line17) == SET)     //判断中线0是否置1
	{
		if( RTC_GetITStatus(RTC_IT_ALRA) == SET)
		{
			
			//闹钟事件
			GPIO_SetBits(GPIOF, GPIO_Pin_8);
			
			RTC_ClearITPendingBit(RTC_IT_ALRA);
		}
		//中断处理程序
		EXTI_ClearITPendingBit(EXTI_Line17);
	}

}
