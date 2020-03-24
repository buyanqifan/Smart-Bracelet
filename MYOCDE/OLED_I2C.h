#ifndef __OLED_I2C_H
#define	__OLED_I2C_H

#include "stm32f4xx.h"
#include "sys.h"


/*
D1 --- GND ---	PD15
D3 --- VCC ---  PD1
D5 --- SCK ---  PE8
D7 --- SDA ---  PE10
*/

#define OLED_GND 		PDout(15)
#define OLED_VCC 		PDout(1)
#define OLED_SCK    	PEout(8)
#define OLED_SDA_OUT    PEout(10)
#define OLED_SDA_IN    	PEin(10)



#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

void I2C_Configuration(void);
void I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
void OLED_ShowNum(unsigned char x, unsigned char y, int num,u8 N,unsigned char TextSize);


#endif
