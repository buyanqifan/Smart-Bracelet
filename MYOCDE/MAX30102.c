#include "MAX30102.h"


/*
D2 --- GND ---	PD0
D4 --- SCK ---  PE7
D6 --- SDA ---  PE9
D8 --- VCC ---  PE11
*/

void MAX30102_I2C_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	//打开GPIO D组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//打开GPIO E组时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//输出
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_11;  		//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//输出
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//推挽
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //速度	 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	//引脚电源设置
	MAX30102_VCC = 1; 
	MAX30102_GND = 0;
	
	//空闲
	MAX30102_SCK		=1;
	MAX30102_SDA_OUT	=1;
	
	
}

//数据模式线
void MAX30102_Iic_Mode(GPIOMode_TypeDef Mode)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;		//引脚9
	GPIO_InitStruct.GPIO_Mode	= Mode;	//输出模式
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//输出推挽
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;	//50MHZ
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void MAX30102_Iic_Start(void)
{
	//SDA为输出
	MAX30102_Iic_Mode(GPIO_Mode_OUT);
	//总线空闲
	MAX30102_SCK 	= 1;
	MAX30102_SDA_OUT = 1;

	delay_us(5);
	MAX30102_SDA_OUT = 0;
	
	
	//两条线都接低，钳住总线
	delay_us(5);
	MAX30102_SCK 	= 0;

}

void MAX30102_Iic_Stop(void)
{
	//SDA为输出
	MAX30102_Iic_Mode(GPIO_Mode_OUT);
	//总线空闲
	MAX30102_SCK 	= 0;
	MAX30102_SDA_OUT = 0;

	delay_us(5);
	MAX30102_SCK 	= 1;
	
	
	//两条线都拉高，停止信号
	delay_us(5);
	MAX30102_SDA_OUT = 1;

}


//通过ack确定引脚输出电平
void MAX30102_Iic_Send_Ack(u8 ack)
{
	//SDA为输出
	MAX30102_Iic_Mode(GPIO_Mode_OUT);

	MAX30102_SCK = 0;
	//准备数据
	if(ack == 1)
	{
		MAX30102_SDA_OUT = 1;
	}
	else
	{
		MAX30102_SDA_OUT = 0;
	}
	
	//形成脉冲周期
	delay_us(5);
	MAX30102_SCK = 1;
	delay_us(5);
	MAX30102_SCK = 0;

}

//先发高位
void MAX30102_Iic_Send_Byte(u8 data)
{
	u8 i;
	//SDA为输出
	MAX30102_Iic_Mode(GPIO_Mode_OUT);	
	
	MAX30102_SCK = 0;
	
	for(i=0; i<8; i++)
	{
		//准备数据
		if( data & 1<<(7-i) )    // 
		{
			MAX30102_SDA_OUT = 1;
		}
		else
		{
			MAX30102_SDA_OUT = 0;
		}	
	
		//形成脉冲周期
		delay_us(5);
		MAX30102_SCK = 1;
		delay_us(5);
		MAX30102_SCK = 0;		
		
	}
	
}

//接收一位数据  返回0表示接收有效应答信号  返回1表示接收无效应答信号
u8 MAX30102_Iic_Recv_Ack(void)
{
	u8 ack;
	//SDA为输出
	MAX30102_Iic_Mode(GPIO_Mode_IN);	
	
	MAX30102_SCK = 0;
	delay_us(5);
	MAX30102_SCK = 1;
	
	//判断引脚电平值
	if(MAX30102_SDA_IN == 1)
	{
		ack = 1;
	}
	else
	{
		ack = 0;
	}
	
	delay_us(5);
	MAX30102_SCK = 0;

	return ack;
}

u8 MAX30102_Iic_Recv_Byte(void)
{
	u8 i, data = 0;  //0100 1010
	
	//模式输出
	MAX30102_Iic_Mode(GPIO_Mode_IN);		
	
	MAX30102_SCK = 0;
	
	for(i=0; i<8; i++)
	{
		delay_us(5);
		MAX30102_SCK = 1;
		
		if(MAX30102_SDA_IN == 1)
		{
			data |= 1<<(7-i);
		}

		delay_us(5);
		MAX30102_SCK = 0;
	}
	
	return data;

}

bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{

	u8 ack;
	//开启信号
	MAX30102_Iic_Start();
	
	//发送设备地址，并执行写操作
	MAX30102_Iic_Send_Byte(I2C_WRITE_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}
	
	MAX30102_Iic_Send_Byte(uch_addr);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	

	//发送数据
	MAX30102_Iic_Send_Byte(uch_data);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	

	MAX30102_Iic_Stop();
    return true;

}

bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
	u8 ack;

	//开启信号
	MAX30102_Iic_Start();
	
	//发送设备地址，并执行写操作
	MAX30102_Iic_Send_Byte(I2C_WRITE_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}
	
	MAX30102_Iic_Send_Byte(uch_addr);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
	
	//开启信号
	MAX30102_Iic_Start();	
	
	//发送设备地址，并执行读操作
	MAX30102_Iic_Send_Byte(I2C_READ_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
	
		
	//接受数据
	*puch_data = MAX30102_Iic_Recv_Byte();

	MAX30102_Iic_Send_Ack(1); //发送无效应答
	MAX30102_Iic_Stop();
    return true;
}

bool maxim_max30102_init()
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return false;
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00))
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(!maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;
  
  if(!maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return false;
  if(!maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return false;
  if(!maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return false;
  return true;  
}

bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
	u8 ack;
    uint32_t un_temp;
    unsigned char uch_temp;
	char ach_i2c_data[6];
    *pun_red_led=0;
    *pun_ir_led=0;
	

    //read and clear status register
    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
	ach_i2c_data[0]=REG_FIFO_DATA;
	

	//开启信号
	MAX30102_Iic_Start();
	
	//发送设备地址，并执行写操作
	MAX30102_Iic_Send_Byte(I2C_WRITE_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}
	
	MAX30102_Iic_Send_Byte(REG_FIFO_DATA);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
	
	//开启信号
	MAX30102_Iic_Start();	
	
	//发送设备地址，并执行读操作
	MAX30102_Iic_Send_Byte(I2C_READ_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
		
	//接受数据
	ach_i2c_data[0] = MAX30102_Iic_Recv_Byte();
	MAX30102_Iic_Send_Ack(0);
	ach_i2c_data[1] = MAX30102_Iic_Recv_Byte();
	MAX30102_Iic_Send_Ack(0);
	ach_i2c_data[2] = MAX30102_Iic_Recv_Byte();
	MAX30102_Iic_Send_Ack(0);
	ach_i2c_data[3] = MAX30102_Iic_Recv_Byte();
	MAX30102_Iic_Send_Ack(0);
	ach_i2c_data[4] = MAX30102_Iic_Recv_Byte();
	MAX30102_Iic_Send_Ack(0);
	ach_i2c_data[5] = MAX30102_Iic_Recv_Byte();

	MAX30102_Iic_Send_Ack(1); //发送无效应答
	MAX30102_Iic_Stop();

	
    
	un_temp=(unsigned char) ach_i2c_data[0];
	un_temp<<=16;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[1];
	un_temp<<=8;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[2];
	*pun_red_led+=un_temp;
	
	un_temp=(unsigned char) ach_i2c_data[3];
	un_temp<<=16;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[4];
	un_temp<<=8;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[5];
	*pun_ir_led+=un_temp;
	*pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
	*pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]

    return true;
	
}

bool maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}
