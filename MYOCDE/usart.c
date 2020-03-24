#include "usart.h"
#include "stdio.h"


#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
} 

//重定义fputc函数   printf 是一个宏
int fputc(int ch, FILE *f)
{ 	
	USART_SendData(USART1,ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);      
	return ch;
}

//蓝牙
void Usart1_Init(void)
{
	GPIO_InitTypeDef  			GPIO_InitStruct;
	USART_InitTypeDef 			USART_InitStruct;
	NVIC_InitTypeDef  			NVIC_InitStruct;
	//使能 USART1 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	
	//使用的是串口 1，串口 1 对应着芯片引脚 PA9,PA10 需要使能PA组的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 

	//设置引脚复用器映射：调用 GPIO_PinAFConfig 函数。
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART3);
	
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_9|GPIO_Pin_10;	//引脚9
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;				//复用
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;			//输出推挽
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;			//50MHZ
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_UP;				//上拉
	//初始化IO口为复用功能输出
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//GPIO 初始化设置：要设置模式为复用功能。
	

	
	USART_InitStruct.USART_BaudRate 			= 9600;				//一般设置为 9600;
	USART_InitStruct.USART_WordLength 			= USART_WordLength_8b;	//字长为 8 位数据格式
	USART_InitStruct.USART_StopBits 			= USART_StopBits_1;		//一个停止位
	USART_InitStruct.USART_Parity 				= USART_Parity_No;		//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;//无硬件控制流
	USART_InitStruct.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;//收发模式
	//串口参数初始化：设置波特率，字长，奇偶校验等参数。
	USART_Init(USART1, &USART_InitStruct); 

	NVIC_InitStruct.NVIC_IRQChannel						= USART1_IRQn;  //串口1通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 2; 			//抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority			= 2; 			//响应优先级
	NVIC_InitStruct.NVIC_IRQChannelCmd					= ENABLE;		//通道使能
	//开启中断并且初始化 NVIC，使能中断（如果需要开启串口中断才需要这个步骤）。
    NVIC_Init(&NVIC_InitStruct);		
	
	//告之CPU中断的方式
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//使能串口。
	USART_Cmd(USART1, ENABLE); 

}