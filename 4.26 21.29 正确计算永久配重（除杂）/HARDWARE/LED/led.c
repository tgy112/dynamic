#include "led.h"
	    
//LED IO初始化
//PB13是周期信号输入
//PA8是32倍频信号输入

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);//使能端口时钟
	
	//PA0定时器2通道1PWM输入捕获，测转速
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;  //PA0 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //PA0 输入 
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);						 //PA0 下拉
	
	//PA2
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //LED0-->PA.2 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.2
 GPIO_SetBits(GPIOD,GPIO_Pin_2);						 //输出高
	
	//PA8是32倍频信号通道，PB13是周期信号通道
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //设置成上拉输入
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8 

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				  
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //设置成上拉输入
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.13 
}
 
