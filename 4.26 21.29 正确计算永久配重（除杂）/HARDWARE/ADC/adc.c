#include "adc.h"
#include "delay.h"

void Adc_Init(void)
{ 
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//使能PA和adc1通道时钟
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//模拟输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC分频因子为6，72/6=12，ADC最大时间不能超过14M
	ADC_DeInit(ADC1);//复位ADC1
	
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel=2;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);//ADC1初始化参数
	
	ADC_Cmd(ADC1,ENABLE);//使能指定的ADC1
	ADC_ResetCalibration(ADC1);//使能复位校准
	while(ADC_GetResetCalibrationStatus(ADC1));//等待复位校准结束
	ADC_StartCalibration(ADC1);//开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));//等待校准结束
}

u16 Get_Adc(u8 ch)//设置指定ADC的规则通道，一个序列，采样时间
{
  ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_28Cycles5);//ADC1，ADC通道，采样时间为239.5周期，（1.5+239.5）/12M约20us的转换时间
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待转换结束
	return ADC_GetConversionValue(ADC1);//返回最近一次ADC1规则的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{  u32 Q;
   u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	 {
	   temp_val+=Get_Adc(ch);
		// delay_ms(5);
	 }
  //return temp_val/times;
	 Q=(((temp_val<<1)/times)>>1)+(((temp_val<<1)/times)&1);
	 return Q;
}


//void Get_Adc_Vual(void)
//{
//	   float adcx1,adcx2,temp1,temp2;
//	
//    adcx1=(float)Get_Adc_Average(ADC_Channel_1,10);//ADC的值
//		temp1=adcx1*(3.3/4096);
//		adcx1=temp1;//ADC电压值
//		
//		adcx2=(float)Get_Adc_Average(ADC_Channel_2,10);
//		temp2=adcx2*(3.3/4096);
//		adcx2=temp2;
//		printf(" %f %f\r\n",adcx1,adcx2);
//		delay_ms(250);	
//}



















