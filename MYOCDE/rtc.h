#ifndef __RTC_H
#define __RTC_H
#include "stm32f4xx.h"

void Rtc_Init(u8 year, u8 month, u8 date, u8 hours, u8 minutes, u8 clock_init_flag);
void Rtc_Get(void);
void Rtc_AlarmA(u8 date, u8 hours, u8 minutes);
#endif
