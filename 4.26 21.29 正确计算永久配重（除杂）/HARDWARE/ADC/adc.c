#include "adc.h"
#include "delay.h"

void Adc_Init(void)
{ 
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//ʹ��PA��adc1ͨ��ʱ��
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//ģ������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����ADC��Ƶ����Ϊ6��72/6=12��ADC���ʱ�䲻�ܳ���14M
	ADC_DeInit(ADC1);//��λADC1
	
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel=2;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);//ADC1��ʼ������
	
	ADC_Cmd(ADC1,ENABLE);//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);//ʹ�ܸ�λУ׼
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�У׼����
}

u16 Get_Adc(u8 ch)//����ָ��ADC�Ĺ���ͨ����һ�����У�����ʱ��
{
  ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_28Cycles5);//ADC1��ADCͨ��������ʱ��Ϊ239.5���ڣ���1.5+239.5��/12MԼ20us��ת��ʱ��
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ʹ��ָ����ADC1�����ת����������
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);//�������һ��ADC1�����ת�����
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
//    adcx1=(float)Get_Adc_Average(ADC_Channel_1,10);//ADC��ֵ
//		temp1=adcx1*(3.3/4096);
//		adcx1=temp1;//ADC��ѹֵ
//		
//		adcx2=(float)Get_Adc_Average(ADC_Channel_2,10);
//		temp2=adcx2*(3.3/4096);
//		adcx2=temp2;
//		printf(" %f %f\r\n",adcx1,adcx2);
//		delay_ms(250);	
//}



















