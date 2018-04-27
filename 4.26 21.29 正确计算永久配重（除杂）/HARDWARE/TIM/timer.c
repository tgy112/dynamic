#include "timer.h"
#include "led.h"
#include "usart.h"
#include "sys.h"
#include "delay.h"

//��ʱ��2ͨ��1PWM���벶������
void TIM2_Cap_Init(void)  //PA0
{	 
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM2_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ��TIM2ʱ��
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;     //����0��FFFF
  TIM_TimeBaseStructure.TIM_Prescaler = 7199;     //ʱ�ӷ�Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;   //ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//������ʼ��
  
	//��ʼ��TIM2PWM����ģʽ
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01,ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ,���˲�
  TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM2,TIM_TS_TI1FP1);//ѡ��IC1Ϊ����Դ
	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);//TIM2��ģʽ�������źŵ����������³�ʼ���������ʹ������ĸ����¼�
	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);//������ʱ���ı�������
	
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);//����CC1IE�����ж�	
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2
 
}

//��ʱ��2�жϷ������	 
u16 IC1Value;
float frequency,RPM;
u16 flag_RPM;
void TIM2_IRQHandler(void)
{ 
    TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);//���TIM���жϴ�����λ
	
	  IC1Value=TIM_GetCapture1(TIM2);//��ȡIC1����Ĵ�����ֵ����ΪPWM�����ڵļ���ֵ
	  
	 if(IC1Value!=0)
	   {
	     frequency = 10000.0 / ((float)IC1Value);//����ǿ��ת��
			 RPM=60*frequency;
	   }
    else
		{
		   frequency=0;
			  RPM=0;
		}
		flag_RPM=0;	
}

//��ʾ����ת��,ʹ��Flag��־���������ж�ʱ��RPM=0
void Get_RPM(void)
{
		  delay_ms(100);
			flag_RPM=flag_RPM+1;
			if(flag_RPM>1)
			{
			  RPM=0;
			}
	  //  printf("%f\r\n",RPM); 
			if(RPM>2000)
		  	GPIO_ResetBits(GPIOA,GPIO_Pin_8);	//��ǵ���ﵽƽ��ת��
			else
			  GPIO_SetBits(GPIOA,GPIO_Pin_8);	
}


















