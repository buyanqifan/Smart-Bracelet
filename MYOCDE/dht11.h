#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"

void Dht11_Init(void);
u8 Dht11_Start(void);
u8 Dht11_Read_Byte(void);
u8 Dht11_Data(u8 *data);

#endif
