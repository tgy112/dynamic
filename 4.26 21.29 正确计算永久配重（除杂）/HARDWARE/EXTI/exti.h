#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

void EXTIX_Init(void);//IO��ʼ��
void Get_Vibration_phase_left(void);
void Get_Vibration_phase_right(void);
void Get_three_times(float Vibration_phase[],int n,float num);
void EXT(u8 en,u8 w);
#define LENGTH_COUNTER 3 //�궨����ͷ�ļ���ÿ�������Ϳ��Ե�����
#endif

