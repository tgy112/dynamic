#include "exti.h"
#include "led.h" 
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "math.h"

float Vibration_A[LENGTH_COUNTER] = {0};
float Phase_a[LENGTH_COUNTER] = {0};
float Vibration_B[LENGTH_COUNTER] = {0};
float Phase_b[LENGTH_COUNTER] = {0};


u16 num_shuzu = 0; 
u16 flag_T_Signal=0;   //���ת�ٴ���2000��־λ
u16 flag_Start=1; 
u16 flag_Calculate = 0;
u16 flag_print_OK = 0;

extern float RPM;
float X[8][32],Y[8][32],X_TEMP[8][32],Y_TEMP[8][32];
float M[33],N[33];
float angle=11.25*3.141592653/180;


void EXT(u8 en,u8 w)		//en:1������;0�ر�;
{
    EXTI->PR=1<<w;  //���11��ΪLINE11�ϵ��жϱ�־λ
    if(en)EXTI->IMR|=1<<w;//������line11�ϵ��ж�
    else EXTI->IMR&=~(1<<w);//����line11�ϵ��ж�   
}

//�ⲿ�жϳ�ʼ������
void EXTIX_Init(void)
{   
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);//��ͬ���ж���Ҫ�ֿ����ã�PB13�����ź�
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line13;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStruct);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);//PA8��32��Ƶ�ź�
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line8;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStruct);
}

//�����ź��жϺ�����PB13������ȡֵ��־�ź�
void EXTI15_10_IRQHandler(void)
{ 
	if(RPM>2000)
	{
		flag_T_Signal=1;
		EXT(1,8);//�򿪲����ź��ж�
		EXT(0,13);
	}
	else
		flag_T_Signal=0;//�ٶ�С��2000
	
	EXTI_ClearITPendingBit(EXTI_Line13);
}

//32��Ƶ�ź��ж�PA8�����鱣��
void EXTI9_5_IRQHandler(void)
{  
	u16 i = 0,j = 0;
	 if (flag_T_Signal==1 && flag_Start == 1) //��һ���ٶȴﵽ2000���յ�һ����ʼ�ź� ��ʼ�ɼ�
	 { 
		 i = num_shuzu / 32;//ȡ��
		 j = num_shuzu % 32;//ȡ��
		 
		 X[i][j]=(Get_Adc_Average(ADC_Channel_1,2))*(3.3/4096);
		 Y[i][j]=(Get_Adc_Average(ADC_Channel_2,2))*(3.3/4096);
		 
		 num_shuzu = num_shuzu + 1;
		 
		 if(num_shuzu > 256)
		   { 
				// flag_T_Signal=0;
//				 flag_Start = 0;//ֹͣ����
				 flag_Calculate = 1;//��Ǽ�����ɵ�������ĺ�����֪ͨ���ͣ��
				 flag_print_OK = 1; //�ɼ��������ˣ���ʼ��ӡ
		  	 num_shuzu = 0;
				 flag_T_Signal = 0;
				 EXT(0,8);
				 EXT(1,13);
				 			 
				for(i=0;i<=31;i++)//����ת�棬�������ڴ������ݹ����жϵ�����������
				{
						for(j=0;j<=7;j++)
						{ 
							X_TEMP[j][i] = X[j][i]; 
						}
				}
				for(i=0;i<=31;i++)//����ת�棬�������ڴ������ݹ����жϵ�����������
				{
						for(j=0;j<=7;j++)
						{ 
							Y_TEMP[j][i] = Y[j][i]; 
						}
				}
				
		   }
   }
	EXTI_ClearITPendingBit(EXTI_Line8);
}

//�õ���У��ƽ����������λ
u8 max_i=0;
float x_sum=0,mix_m,max_m;
float A,a,X_a,Y_a,a_temp,a_test;
void Get_Vibration_phase_left(void)
  {
		u8 i,j;
		for(j=0;j<=31;j++)//ÿ��AD��ѹֵ���ֵ
		{
			x_sum = 0;
				for(i=0;i<=7;i++)
		  	{ 
					x_sum += X_TEMP[i][j]; 
		  	}
				M[j] = (x_sum / 8);
				x_sum = 0;
				//printf("%f \r\n",M[j]);
		}
		
		//������˲�(ë��ʦ�ķ���)
		X_a=M[0]*cos(0)+M[1]*(cos(angle))+M[2]*(cos(2*angle))+M[3]*(cos(3*angle))+M[4]*(cos(4*angle))+M[5]*(cos(5*angle))+M[6]*(cos(6*angle))+M[7]*(cos(7*angle))
		+M[8]*(cos(8*angle))+M[9]*(cos(9*angle))+M[10]*(cos(10*angle))+M[11]*(cos(11*angle))+M[12]*(cos(12*angle))+M[13]*(cos(13*angle))+M[14]*(cos(14*angle))+M[15]*(cos(15*angle))
		+M[16]*(cos(16*angle))+M[17]*(cos(17*angle))+M[18]*(cos(18*angle))+M[19]*(cos(19*angle))+M[20]*(cos(20*angle))+M[21]*(cos(21*angle))+M[22]*(cos(22*angle))+M[23]*(cos(23*angle))
		+M[24]*(cos(24*angle))+M[25]*(cos(25*angle))+M[26]*(cos(26*angle))+M[27]*(cos(27*angle))+M[28]*(cos(28*angle))+M[29]*(cos(29*angle))+M[30]*(cos(30*angle))+M[31]*(cos(31*angle));
		
		Y_a=M[0]*sin(0)+M[1]*(sin(angle))+M[2]*(sin(2*angle))+M[3]*(sin(3*angle))+M[4]*(sin(4*angle))+M[5]*(sin(5*angle))+M[6]*(sin(6*angle))+M[7]*(sin(7*angle))
		+M[8]*(sin(8*angle))+M[9]*(sin(9*angle))+M[10]*(sin(10*angle))+M[11]*(sin(11*angle))+M[12]*(sin(12*angle))+M[13]*(sin(13*angle))+M[14]*(sin(14*angle))+M[15]*(sin(15*angle))
		+M[16]*(sin(16*angle))+M[17]*(sin(17*angle))+M[18]*(sin(18*angle))+M[19]*(sin(19*angle))+M[20]*(sin(20*angle))+M[21]*(sin(21*angle))+M[22]*(sin(22*angle))+M[23]*(sin(23*angle))
		+M[24]*(sin(24*angle))+M[25]*(sin(25*angle))+M[26]*(sin(26*angle))+M[27]*(sin(27*angle))+M[28]*(sin(28*angle))+M[29]*(sin(29*angle))+M[30]*(sin(30*angle))+M[31]*(sin(31*angle));
		
				
		max_m = M[0];
		for(i=0;i<=31;i++)//������η�ֵ�������
		{
				if(max_m<M[i]){max_m=M[i];max_i = i;}
				else;
		}
		
		A=sqrt(X_a*X_a+Y_a*Y_a)*78.92;//��У��ƽ�����񶯷�ֵ
//		Get_three_times(Vibration_A,LENGTH_COUNTER,A);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����صķ�ֵA0��A1��A2����Vibration_A[0]��Vibration_A[1]��Vibration_A[2]
		a_temp=atan(Y_a/X_a)/3.14159*180;
		
		if(X_a>0)
		{
			if(Y_a<0)a = a_temp + 360;
			else		 a = a_temp;
		}
		else a = a_temp + 180;
		
		a-=80;//������ת����������ë��ʦ�豸�Աȣ��������ʵ����Ҫ�޸�
		if(a<0)a_test=a+360;
		else a_test = a;
			
//		printf("%f   %f   \r\n",A,a);
//		Get_three_times(Phase_a,LENGTH_COUNTER,a);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����ص���λa0��a1��a2����Phase_a[0]��Phase_a[1]��Phase_a[2]
		//printf("%f   %f   %f   %f   %f   %f\r\n",Vibration_A[0],Vibration_A[1],Vibration_A[2],Phase_a[0],Phase_a[1],Phase_a[2]);
  }

//�õ���У��ƽ����������λ
float y_sum=0,max_n;
float B,b,X_b,Y_b,b_temp,b_test;
void Get_Vibration_phase_right(void)
  {
		u8 i,j;
		for (j=0;j<=31;j++)
		  {
        for(i=0;i<=7;i++)
		  	{  
					 y_sum=y_sum+(Y[i][j]);
		  	}
				N[j] = (y_sum / 8 );
				y_sum = 0;
			}	
   
		X_b=N[0]*cos(0)+N[1]*(cos(angle))+N[2]*(cos(2*angle))+N[3]*(cos(3*angle))+N[4]*(cos(4*angle))+N[5]*(cos(5*angle))+N[6]*(cos(6*angle))+N[7]*(cos(7*angle))
		+N[8]*(cos(8*angle))+N[9]*(cos(9*angle))+N[10]*(cos(10*angle))+N[11]*(cos(11*angle))+N[12]*(cos(12*angle))+N[13]*(cos(13*angle))+N[14]*(cos(14*angle))+N[15]*(cos(15*angle))
		+N[16]*(cos(16*angle))+N[17]*(cos(17*angle))+N[18]*(cos(18*angle))+N[19]*(cos(19*angle))+N[20]*(cos(20*angle))+N[21]*(cos(21*angle))+N[22]*(cos(22*angle))+N[23]*(cos(23*angle))
		+N[24]*(cos(24*angle))+N[25]*(cos(25*angle))+N[26]*(cos(26*angle))+N[27]*(cos(27*angle))+N[28]*(cos(28*angle))+N[29]*(cos(29*angle))+N[30]*(cos(30*angle))+N[30]*(cos(31*angle));
		
		Y_b=N[0]*sin(0)+N[1]*(sin(angle))+N[2]*(sin(2*angle))+N[3]*(sin(3*angle))+N[4]*(sin(4*angle))+N[5]*(sin(5*angle))+N[6]*(sin(6*angle))+N[7]*(sin(7*angle))
		+N[8]*(sin(8*angle))+N[9]*(sin(9*angle))+N[10]*(sin(10*angle))+N[11]*(sin(11*angle))+N[12]*(sin(12*angle))+N[13]*(sin(13*angle))+N[14]*(sin(14*angle))+N[15]*(sin(15*angle))
		+N[16]*(sin(16*angle))+N[17]*(sin(17*angle))+N[18]*(sin(18*angle))+N[19]*(sin(19*angle))+N[20]*(sin(20*angle))+N[21]*(sin(21*angle))+N[22]*(sin(22*angle))+N[23]*(sin(23*angle))
		+N[24]*(sin(24*angle))+N[25]*(sin(25*angle))+N[26]*(sin(26*angle))+N[27]*(sin(27*angle))+N[28]*(sin(28*angle))+N[29]*(sin(29*angle))+N[30]*(sin(30*angle))+N[31]*(sin(31*angle));
					
			max_n = N[0];
		for(i=0;i<=31;i++)//������η�ֵ�������
		{
				if(max_n<N[i]){max_n=N[i];max_i = i;}
				else;
		}
		b_temp=atan(Y_b/X_b)/3.14159*180;
		
		if((max_i<12 && (b_temp<=90 && b_temp>=0)) || (max_i>=24 && (b_temp>=-90 && b_temp<=0)))//��-90��90�ȵ�����ת����Ϊ360����
			b=b_temp-180+270;
		else 			
			b=b_temp+270;
		
		b+=190;//������ת����������ë��ʦ�豸�Աȣ��������ʵ����Ҫ�޸�
		if(b>360)b_test=b-360;
		else b_test = b;
		
//		B=sqrt(X_b*X_b+Y_b*Y_b);//��У��ƽ�����񶯷�ֵ
//		Get_three_times(Vibration_B,LENGTH_COUNTER,B);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����صķ�ֵB0��B1��B2����Vibration_B[0]��Vibration_B[1]��Vibration_B[2]
//		
//		b=atan(Y_b/X_b);//��У��ƽ�����λ
//		Get_three_times(Phase_b,LENGTH_COUNTER,b);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����ص���λb0��b1��b2����Phase_b[0]��Phase_b[1]��Phase_b[2]
		//printf("%f %f %f %f %f %f\r\n",Vibration_B[0],Vibration_B[1],Vibration_B[2],Phase_b[0],Phase_b[1],Phase_b[2]);
  }	
	 
	//�������ε�����Ϣ
	void Get_three_times(float Vibration_phase[],int n,float num)
	{
	  int m = 0;
		for(m = n-2;m >= 0;m--)
			{
				Vibration_phase[m+1] = Vibration_phase[m];
			}
	  	Vibration_phase[0] = num;
	}

