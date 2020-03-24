#include "light_sensor.h"


void Light_Sensor_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	ADC_CommonInitTypeDef 	ADC_CommonInitStruct;
	ADC_InitTypeDef 		ADC_InitStruct;
	
	//��GPIO F��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	//��ADC3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);  
	
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_7;  		//����7
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AN;			//���
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;		//����	 
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	ADC_CommonInitStruct.ADC_Mode				= ADC_Mode_Independent; 		//����ģʽ
	ADC_CommonInitStruct.ADC_Prescaler			= ADC_Prescaler_Div4; 			// 84/4 = 21MHZ  Ƶ�ʲ��ܴ��� 36MHZ
	ADC_CommonInitStruct.ADC_DMAAccessMode		= ADC_DMAAccessMode_Disabled; 	//����DMA
	ADC_CommonInitStruct.ADC_TwoSamplingDelay	= ADC_TwoSamplingDelay_5Cycles;	//���������׶�֮����ӳ�5��ʱ��

	//2����ʼ��ADC_CCR�Ĵ�����
     ADC_CommonInit(&ADC_CommonInitStruct);
	 
	 //3����ʼ��ADC1����������ADC1�Ĺ���ģʽ�Լ��������е������Ϣ��
	ADC_InitStruct.ADC_Resolution 			= ADC_Resolution_12b;//12λģʽ ���� 4095
	ADC_InitStruct.ADC_ScanConvMode 		= DISABLE;//��ɨ��ģʽ	
	ADC_InitStruct.ADC_ContinuousConvMode 	= DISABLE;//�ر�����ת��
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStruct.ADC_DataAlign 			= ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStruct.ADC_NbrOfConversion 		= 1;//1��ת���ڹ��������� 
	ADC_Init(ADC3, &ADC_InitStruct);		//ADC��ʼ��
	
	//4��ʹ��ADC��
    ADC_Cmd(ADC3, ENABLE);
	//5�����ù���ͨ��������
     ADC_RegularChannelConfig(ADC3,ADC_Channel_5,1,ADC_SampleTime_15Cycles);	
	
}

u16 Get_Light_Sensor_Value(void)
{
	u16 value;
	
	//�������ת��
	ADC_SoftwareStartConv(ADC3);

	//7���ȴ�ת����ɣ���ȡADCֵ��
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET );
	
	//��ȡADC��ֵ 
	value = ADC_GetConversionValue(ADC3);
	
	return value;

}
