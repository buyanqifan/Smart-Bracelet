#ifndef __LIGHT_SENSOR_H
#define __LIGHT_SENSOR_H

#include "stm32f4xx.h"


void Light_Sensor_Init(void);
u16 Get_Light_Sensor_Value(void);

#endif
