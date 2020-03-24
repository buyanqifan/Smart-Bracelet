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
	//��GPIO D��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//��GPIO E��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_0;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_11;  		//����9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;	//���
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//����
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz; //�ٶ�	 
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	//���ŵ�Դ����
	MAX30102_VCC = 1; 
	MAX30102_GND = 0;
	
	//����
	MAX30102_SCK		=1;
	MAX30102_SDA_OUT	=1;
	
	
}

//����ģʽ��
void MAX30102_Iic_Mode(GPIOMode_TypeDef Mode)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9;		//����9
	GPIO_InitStruct.GPIO_Mode	= Mode;	//���ģʽ
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;	//�������
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;	//50MHZ
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;		//����
	GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void MAX30102_Iic_Start(void)
{
	//SDAΪ���
	MAX30102_Iic_Mode(GPIO_Mode_OUT);
	//���߿���
	MAX30102_SCK 	= 1;
	MAX30102_SDA_OUT = 1;

	delay_us(5);
	MAX30102_SDA_OUT = 0;
	
	
	//�����߶��ӵͣ�ǯס����
	delay_us(5);
	MAX30102_SCK 	= 0;

}

void MAX30102_Iic_Stop(void)
{
	//SDAΪ���
	MAX30102_Iic_Mode(GPIO_Mode_OUT);
	//���߿���
	MAX30102_SCK 	= 0;
	MAX30102_SDA_OUT = 0;

	delay_us(5);
	MAX30102_SCK 	= 1;
	
	
	//�����߶����ߣ�ֹͣ�ź�
	delay_us(5);
	MAX30102_SDA_OUT = 1;

}


//ͨ��ackȷ�����������ƽ
void MAX30102_Iic_Send_Ack(u8 ack)
{
	//SDAΪ���
	MAX30102_Iic_Mode(GPIO_Mode_OUT);

	MAX30102_SCK = 0;
	//׼������
	if(ack == 1)
	{
		MAX30102_SDA_OUT = 1;
	}
	else
	{
		MAX30102_SDA_OUT = 0;
	}
	
	//�γ���������
	delay_us(5);
	MAX30102_SCK = 1;
	delay_us(5);
	MAX30102_SCK = 0;

}

//�ȷ���λ
void MAX30102_Iic_Send_Byte(u8 data)
{
	u8 i;
	//SDAΪ���
	MAX30102_Iic_Mode(GPIO_Mode_OUT);	
	
	MAX30102_SCK = 0;
	
	for(i=0; i<8; i++)
	{
		//׼������
		if( data & 1<<(7-i) )    // 
		{
			MAX30102_SDA_OUT = 1;
		}
		else
		{
			MAX30102_SDA_OUT = 0;
		}	
	
		//�γ���������
		delay_us(5);
		MAX30102_SCK = 1;
		delay_us(5);
		MAX30102_SCK = 0;		
		
	}
	
}

//����һλ����  ����0��ʾ������ЧӦ���ź�  ����1��ʾ������ЧӦ���ź�
u8 MAX30102_Iic_Recv_Ack(void)
{
	u8 ack;
	//SDAΪ���
	MAX30102_Iic_Mode(GPIO_Mode_IN);	
	
	MAX30102_SCK = 0;
	delay_us(5);
	MAX30102_SCK = 1;
	
	//�ж����ŵ�ƽֵ
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
	
	//ģʽ���
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
	//�����ź�
	MAX30102_Iic_Start();
	
	//�����豸��ַ����ִ��д����
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

	//��������
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

	//�����ź�
	MAX30102_Iic_Start();
	
	//�����豸��ַ����ִ��д����
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
	
	//�����ź�
	MAX30102_Iic_Start();	
	
	//�����豸��ַ����ִ�ж�����
	MAX30102_Iic_Send_Byte(I2C_READ_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
	
		
	//��������
	*puch_data = MAX30102_Iic_Recv_Byte();

	MAX30102_Iic_Send_Ack(1); //������ЧӦ��
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
	

	//�����ź�
	MAX30102_Iic_Start();
	
	//�����豸��ַ����ִ��д����
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
	
	//�����ź�
	MAX30102_Iic_Start();	
	
	//�����豸��ַ����ִ�ж�����
	MAX30102_Iic_Send_Byte(I2C_READ_ADDR);
	ack = MAX30102_Iic_Recv_Ack();
	if(ack == 1)
	{
		MAX30102_Iic_Stop();
		//printf("ack failure\n");
		return false;
	}	
		
	//��������
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

	MAX30102_Iic_Send_Ack(1); //������ЧӦ��
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
