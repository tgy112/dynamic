#include "led.h"
	    
//LED IO��ʼ��
//PB13�������ź�����
//PA8��32��Ƶ�ź�����

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);//ʹ�ܶ˿�ʱ��
	
	//PA0��ʱ��2ͨ��1PWM���벶�񣬲�ת��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;  //PA0 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //PA0 ���� 
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);						 //PA0 ����
	
	//PA2
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //LED0-->PA.2 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.2
 GPIO_SetBits(GPIOD,GPIO_Pin_2);						 //�����
	
	//PA8��32��Ƶ�ź�ͨ����PB13�������ź�ͨ��
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //���ó���������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8 

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				  
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //���ó���������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.13 
}
 
