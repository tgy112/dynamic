#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//CAN���� ����  
//ZLC
//������̳:www.BIT.com
//�޸�����:2014/11/05
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) XXXX�Ƽ����޹�˾ 2014-2024
//All rights reserved						  

//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 Can_Send_Msg(u8* msg,u8 len);						//��������

u8 Can_Receive_Msg(u8 *buf);							//��������
#endif


