#include "timer.h"
#include "led.h"
#include "usart.h"
#include "sys.h"
#include "delay.h"

//定时器2通道1PWM输入捕获配置
void TIM2_Cap_Init(void)  //PA0
{	 
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM2_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;     //周期0～FFFF
  TIM_TimeBaseStructure.TIM_Prescaler = 7199;     //时钟分频
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;   //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//模式
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//基本初始化
  
	//初始化TIM2PWM输入模式
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01,选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 ,不滤波
  TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM2,TIM_TS_TI1FP1);//选择IC1为触发源
	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);//TIM2从模式：触发信号的上升沿重新初始化计数器和触发器的更新事件
	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);//启动定时器的被动触发
	
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);//允许CC1IE捕获中断	
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2
 
}

//定时器2中断服务程序	 
u16 IC1Value;
float frequency,RPM;
u16 flag_RPM;
void TIM2_IRQHandler(void)
{ 
    TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);//清除TIM的中断待处理位
	
	  IC1Value=TIM_GetCapture1(TIM2);//读取IC1捕获寄存器的值，即为PWM波周期的计数值
	  
	 if(IC1Value!=0)
	   {
	     frequency = 10000.0 / ((float)IC1Value);//数据强制转换
			 RPM=60*frequency;
	   }
    else
		{
		   frequency=0;
			  RPM=0;
		}
		flag_RPM=0;	
}

//显示测量转速,使用Flag标志是在跳出中断时让RPM=0
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
		  	GPIO_ResetBits(GPIOA,GPIO_Pin_8);	//标记电机达到平衡转速
			else
			  GPIO_SetBits(GPIOA,GPIO_Pin_8);	
}


















