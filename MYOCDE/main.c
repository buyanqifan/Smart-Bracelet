#include "stm32f4xx.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "exti.h"
#include "delay.h"
#include "time.h"
#include "pwm.h"
#include "usart.h"
#include "string.h"
#include "dht11.h"
#include <stdio.h>
#include "sr04.h"
#include "infrared.h"
#include "iwdg.h"
#include "rtc.h"
#include "adc.h"
#include "light_sensor.h"
#include "iic.h"
#include "OLED_I2C.h"
#include "delay.h"
#include "MAX30102.h"
#include "algorithm.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

#define MAX_BRIGHTNESS 255
#define INT  0 //读取按键0
//max变量
uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value  血氧浓度
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid   血氧浓度是否有效
int32_t n_heart_rate;   //heart rate value 心率
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid  心率是否有效
uint8_t uch_dummy;
int32_t n_heart_rate_last;
int32_t n_heart_rate_d;
u8 n_heart_rate_flag = 0;
//接收函数变量
u8 rx_flag = 0;
u8 rx_i	= 0;
u8 rx_count	= 0;
u8 rx_data[64] = {0};
u8 buffer[64] = {0};
//闹钟变量
u8 alarm_hours[3]   = {0};
u8 alarm_minutes[3] = {0};
u8 alarm_date[3]    = {0};
u8 clock_year[3]    = {0};
u8 clock_month[3]   = {0};
u8 clock_date[3]    = {0};
u8 clock_hours[3]   = {0};
u8 clock_minutes[3] = {0};

u8 clock_init_flag = 0;		//改为1则初始化时可修改时间

//OLED与图片
unsigned int OLED_x = 0;
unsigned int OLED_y = 0;
unsigned int OLED_i = 0;
extern const unsigned char MAIN[];
extern const unsigned char MAXBAP[];
u8 OLED_GRAM[128][8] = {0};
uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int i;
int32_t n_brightness;
float f_temp;
u8 OLED_flag = 0;		//血氧flag
u8 OLED_pumflag = 0;	//跑步flag

//步数变量
uint8_t res;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度		
unsigned long StepCount = 0;             //初始步数
unsigned long StepCounttmp = 0;             //存储步数数值
int distance = 0;    //测定距离

u8 display_flag = 0;



//串口1蓝牙接收数据
void USART1_IRQHandler(void);

void CLOCK(void);

void MAX30102(void);

void PUM6050(void);

int main(void)
{


	//中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Pwm_Init();
	Led_Init();
	Key_Init();
	Delay_Init();
	Usart1_Init();
	Beep_Init();
	
	//OLED初始化
	I2C_Configuration();
	OLED_Init();
	OLED_Fill(0x00);		//全屏熄灭
	OLED_DrawBMP(0,0,128,8,(unsigned char *)MAIN);		//显示主界面
	Rtc_Init(19, 10, 10, 22, 44, clock_init_flag);//设置当前时间
	
	//pum6050初始化 计步器
	MPU_Init();
	while(mpu_dmp_init())
	{
		printf("MPU6050 ERROR \r\n");
		delay_ms(500);
	}

	//max血氧初始化
	MAX30102_I2C_Configuration();
	printf("test\n");
	maxim_max30102_reset();
	printf("test\n");
	maxim_max30102_read_reg(0,&uch_dummy);
	printf("test\n");
	maxim_max30102_init();
	printf("test\n");
	n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
	
	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	
	 for(i=0;i<n_ir_buffer_length;i++)
    {
        //while(INT==1);   //wait until the interrupt pin asserts
        
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
        printf("red=");
        printf("%i", aun_red_buffer[i]);
        printf(", ir=");
        printf("%i\n\r", aun_ir_buffer[i]);
    }
    un_prev_data=aun_red_buffer[i];
	
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
   
	printf("test\n");
	
	//初始化完成，进入主界面
	//OLED_DrawBMP(0,0,128,8,(unsigned char *)MAXBAP);		//显示心率界面
	OLED_DrawBMP(0,0,128,8,(unsigned char *)MAIN);		//显示主界面
	OLED_ShowStr(0,7,"HR",1);						//左下角显示HR
	OLED_ShowStr(127-6*4,7,"STEP",1);				//左下角显示STEP
	while(1)
	{

        OLED_DrawBMP(0,0,128,8,(unsigned char *)MAIN);		//显示主界面
		OLED_ShowStr(0,7,"HR",1);						//左下角显示HR
		OLED_ShowStr(127-6*4,7,"STEP",1);				//左下角显示STEP
		CLOCK();
		if(display_flag == 0)
			MAX30102();
		else
			PUM6050();
		
		display_flag = 0;
	}
	

	return 0;
}


//串口1蓝牙接收数据
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET) //溢出中断
	{
		
		//数据存放在rx_data当中
		buffer[rx_count++] = USART_ReceiveData(USART1);
		
		if(buffer[rx_count-1] == ':')  //表示接收到结束符
		{
			for(rx_i=0; rx_i<rx_count-1; rx_i++)
			{
				rx_data[rx_i] = buffer[rx_i];      //将收到的有效数据存储在rx_data当中
			
			}
		
			rx_count = 0;			//置0,下次buffer从buffer[0]开始接收数据
			memset(buffer, 0, sizeof(buffer));
			rx_flag =1; //表示接收到数据
		}
	
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE); //清除中断标志位

}


//时钟显示及调节函数
void CLOCK(void)
{
	OLED_DrawBMP(0,0,128,8,(unsigned char *)MAIN);		//显示主界面
	OLED_ShowStr(0,7,"HR",1);						//左下角显示HR
	OLED_ShowStr(127-6*4,7,"STEP",1);				//左下角显示STEP
	while(1)
	{
		
		if(rx_flag == 1)  //表示串口有数据
		{

			clock_init_flag = 1;		//允许修改时间
			//printf("rx_data = %s\n",rx_data);
			if(strncmp(rx_data,"alarm",5) == 0)  //表示调闹钟例如：alarm19-21-21:
			{
				alarm_date[0]    = rx_data[5];
				alarm_date[1]    = rx_data[6];
				alarm_hours[0]   = rx_data[8];
				alarm_hours[1]   = rx_data[9];
				alarm_minutes[0] = rx_data[11];
				alarm_minutes[1] = rx_data[12];
				
				//字符转化为整型
				if(atoi(alarm_date)>0 && atoi(alarm_hours)>0 && atoi(alarm_minutes)>0)
				{
					Rtc_AlarmA(atoi(alarm_date), atoi(alarm_hours), atoi(alarm_minutes));
					//printf("alarm_date    = %d\n",atoi(alarm_date));
					//printf("alarm_hours   = %d\n",atoi(alarm_hours));
					//printf("alarm_minutes = %d\n",atoi(alarm_minutes));
				}
				
			}
			
			if(strncmp(rx_data,"clock",5) == 0)  //表示调例如：clock19-09-20-19-22:
			{
				clock_year[0]    = rx_data[5];
				clock_year[1]    = rx_data[6];
				clock_month[0]   = rx_data[8];
				clock_month[1]   = rx_data[9];
				clock_date[0]    = rx_data[11];
				clock_date[1]    = rx_data[12];
				clock_hours[0]   = rx_data[14];
				clock_hours[1]   = rx_data[15];
				clock_minutes[0] = rx_data[17];
				clock_minutes[1] = rx_data[18];
				
				if(atoi(clock_year)>0 && atoi(clock_month)>0 && atoi(clock_date)>0 && atoi(clock_hours)>0 && atoi(clock_minutes)>0)
				{
					Rtc_Init(atoi(clock_year), atoi(clock_month), atoi(clock_date), atoi(clock_hours), atoi(clock_minutes), ++clock_init_flag);
				}
				clock_init_flag = 0;			//禁止修改时间

			}
			
			memset(rx_data, 0, sizeof(rx_data));
			rx_flag = 0;
			
		}
		
		Rtc_Get();//打印当前时间

		if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断左键是否按下
		{
			delay_ms(15);		//延时消抖
			if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断按键是否按下
			{
				
				while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET);   //等待按键松开
				
				//返回主函数进入心率界面
				display_flag = 0;
				return ;
			}			
		
		}
		
		if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断右键是否按下
			{
				delay_ms(15);		//延时消抖
				if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断按键是否按下
				{
					
					while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET);   //等待按键松开
					//返回主函数进入计步界面
					display_flag = 1;
					return ;
				}			
			
			}
		
	}
}



void MAX30102(void)
{
	OLED_DrawBMP(0,0,128,8,(unsigned char *)MAXBAP);		//显示心率界面
	OLED_ShowStr(0,7,"HRdig",1);						//左下角显示HRdig
	OLED_ShowStr(127-6*6,7,"REBACK",1);				//左下角显示REBACK
	while(1)
	{
		i=0;
        un_min=0x3FFFF;
        un_max=0;
		
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
        
        //take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(INT == 1);
            maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
        
            if(aun_red_buffer[i]>un_prev_data)//just to determine the brightness of LED according to the deviation of adjacent two AD data
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }

			TIM_SetCompare1(TIM14, (1-(float)n_brightness/256)*499);
			GPIO_ResetBits(GPIOE, GPIO_Pin_14);	
            //pwmled.write(1-(float)n_brightness/256);//pwm control led brightness
						if(n_brightness<120)
							GPIO_SetBits(GPIOF, GPIO_Pin_10);
						else
							GPIO_ResetBits(GPIOF, GPIO_Pin_10);

            //send samples and calculation result to terminal program through UART
//            printf("red=");
//            printf("%i", aun_red_buffer[i]);
//            printf(", ir=");
//            printf("%i", aun_ir_buffer[i]);
//            printf(", HR=%i, ", n_heart_rate); 
//            printf("HRvalid=%i, ", ch_hr_valid);
//            printf("SpO2=%i, ", n_sp02);
//            printf("SPO2Valid=%i\n\r", ch_spo2_valid);

			//显示心率界面			
			if(OLED_flag == 0)
			{
				//OLED_DrawBMP(0,0,128,8,(unsigned char *)MAXBAP);		//显示心率界面
				OLED_ShowStr(55,2,"HR:",2);
				OLED_ShowNum(10, 2, n_heart_rate,3, 2);			
				OLED_ShowStr(55,5,"SPO2:",2);
				OLED_ShowNum(12, 5, n_sp02,3, 2);
			}

			//按下左键进入心率图界面
			if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断按键是否按下
			{
				delay_ms(15);		//延时消抖
				if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断按键是否按下
				{
					
					while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET);   //等待按键松开
					
					//显示心率图
					OLED_flag = 1;
					OLED_CLS();
					OLED_ShowStr(127-6*6,7,"REBACK",1);				//右下角显示REBACK
				}			
			
			}			
			
			//按下右键返回主界面
			if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断右键是否按下
			{
				delay_ms(15);		//延时消抖
				if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断按键是否按下
				{
					
					while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET);   //等待按键松开
					//返回主界面
					OLED_flag = 0;
					return ;
				}			
			
			}
        }
		
		//心率图界面
		if(OLED_flag == 1)
		{
			
			if(n_heart_rate < 384)  //把心率大于384的无效数据舍弃
			{
				//一个数据由四个点组成
				//第二个点开始中间插入数据点连成曲线
				if(n_heart_rate_flag == 1)
				{
					//显示插入的数据
					if(n_heart_rate > n_heart_rate_last)
					{
						n_heart_rate_d = n_heart_rate - n_heart_rate_last;
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last+n_heart_rate_d/4)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last+n_heart_rate_d/4)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4*2)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last+n_heart_rate_d/4*2)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4*2)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last+n_heart_rate_d/4*2)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4*3)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last+n_heart_rate_d/4*3)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last+n_heart_rate_d/4*3)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last+n_heart_rate_d/4*3)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
					}
					else
					{
						n_heart_rate_d = n_heart_rate_last - n_heart_rate;
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last - n_heart_rate_d/4)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last - n_heart_rate_d/4)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4*2)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last - n_heart_rate_d/4*2)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4*2)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last - n_heart_rate_d/4*2)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4*3)/6, 1);
						OLED_DrawPoint(OLED_x - 1, (64-(n_heart_rate_last - n_heart_rate_d/4*3)/6)-1, 1);
						OLED_DrawPoint(OLED_x++, 64-(n_heart_rate_last - n_heart_rate_d/4*3)/6, 1);
						OLED_DrawPoint(OLED_x -1, (64-(n_heart_rate_last - n_heart_rate_d/4*3)/6)-1, 1);
						OLED_Refresh_Gram();    //刷新屏幕显示数据
						
					}
				}
				
				//显示真正获取的数据
				OLED_DrawPoint(OLED_x++, 64-n_heart_rate/6, 1);
				OLED_DrawPoint(OLED_x - 1, (64-n_heart_rate/6)-1, 1);
				OLED_DrawPoint(OLED_x++, 64-n_heart_rate/6, 1);
				OLED_DrawPoint(OLED_x -1, (64-n_heart_rate/6)-1, 1);
				

				n_heart_rate_flag = 1;
				OLED_Refresh_Gram();    //刷新屏幕显示数据
				n_heart_rate_last = n_heart_rate;	//存储上一次心率值
				if(OLED_x > 127)
				{
					OLED_x = 0;
					n_heart_rate_flag = 0;
					memset(OLED_GRAM,'\0',sizeof(OLED_GRAM));	//描点的数据清零
				}
			}
		}

	
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

		
	}		
}

void PUM6050(void)
{

	OLED_DrawBMP(0,0,128,8,(unsigned char *)MAXBAP);		//显示跑步界面
	OLED_ShowStr(0,7,"StepCount",1);						//左下角显示StepCount
	OLED_ShowStr(127-6*6,7,"REBACK",1);				//左下角显示REBACK

	
	while(1)
	{
		//显示步数界面
		if(OLED_pumflag == 0)
		{
			res=mpu_dmp_get_data(&pitch,&roll,&yaw);
			if(res==0)
			{ 
				dmp_get_pedometer_step_count(&StepCount);   //得到步数    返回0表示函数调用成功
				distance = StepCount*0.7;	//得到距离	
				OLED_ShowStr(55,2,"Step:",2);
				OLED_ShowNum(12, 2, StepCount,3, 2);	//显示步数
				OLED_ShowStr(55,5,"DIS:",2);	
				OLED_ShowNum(11, 5, distance,3, 2);		//显示距离
				
				if(rx_flag == 1)  //表示串口有数据
				{
				
					if(strncmp(rx_data,"getstep",7) == 0)  //蓝牙发送getstep:接收步数数据
						printf("StepCount:%d\n", StepCount);
					memset(rx_data, 0, sizeof(rx_data));
					rx_flag = 0;
				}
				
				StepCounttmp = StepCount;				//存储步数数据
			}
		}
		
		//显示计步模式界面
		if(OLED_pumflag == 1)
		{
			dmp_get_pedometer_step_count(&StepCount);   //得到步数    返回0表示函数调用成功
			distance = StepCount*0.7;	//得到距离	
			for(OLED_i = 0; OLED_i<4; OLED_i++)		//显示中文"计步模式"
			{
				OLED_ShowCN(32+OLED_i*16,3,OLED_i);
			}
			OLED_ShowNum(12, 3, StepCount,4, 2);	//显示步数
			for(OLED_i = 4; OLED_i<8; OLED_i++)		//显示中文"行走距离"
			{
				OLED_ShowCN(32+OLED_i*16-4*16,5,OLED_i);
			}
			OLED_ShowNum(12, 5, distance,4, 2);	//显示距离
			
			if(rx_flag == 1)  //表示串口有数据
			{
			
				if(strncmp(rx_data,"getstep",7) == 0)  //蓝牙发送getstep:接收计步模式步数数据
					printf("StepCount:%d\n", StepCount);
				memset(rx_data, 0, sizeof(rx_data));
				rx_flag = 0;
			}

		}
		
		//按下左键进入计步模式
		if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断按键是否按下
			{
				delay_ms(15);		//延时消抖
				if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET )  //判断按键是否按下
				{
					
					while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET);   //等待按键松开
					
					OLED_pumflag = 1;//显示计步模式界面
					OLED_DrawBMP(0,0,128,8,(unsigned char *)MAIN);		//显示主界面
					OLED_ShowStr(0,7,"ReStepCount",1);					//左下角显示ReStepCount
					OLED_ShowStr(127-6*6,7,"REBACK",1);					//左下角显示REBACK
					StepCounttmp = StepCount;							//记下步数
					while(dmp_set_pedometer_step_count(0) != 0);		//步数清零
					
					
				}			
			
			}			
		

		
		//按下右键进入主界面
		if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断右键是否按下
		{
			delay_ms(15);		//延时消抖
			if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET )  //判断按键是否按下
			{
				
				while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET);   //等待按键松开
				//返回主界面
				OLED_pumflag = 0;
				while(dmp_set_pedometer_step_count(StepCounttmp) != 0);			//将原步数赋回
				return ;
			}			
		
		}
	}

}

