#include "can.h"

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ��(APB1)�ڳ�ʼ����ʱ������Ϊ64/2=32M,�������CAN_Normal_Init(1,4,3,4,1);
//������Ϊ:32M/((1+4+3)*4)=1MKbps
//����ֵ:0,��ʼ��OK;
//����,��ʼ��ʧ��;

u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
  {
	  GPIO_InitTypeDef GPIO_InitStructure; 
  	CAN_InitTypeDef CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
    #if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
    #endif

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE); //�˿ڸ���
	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//��������
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	  
		//CAN��Ԫ����
		CAN_InitStructure.CAN_ABOM=DISABLE;//����Զ����߹��������CAN_MCR�Ĵ�����INRQλ������1�����0��һ��Ӳ�����
		//��128��11λ����������λ�����˳�����״̬
		CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������
		CAN_InitStructure.CAN_NART=ENABLE;//��ֹ�����Զ����ͣ�����ֻ������һ��
		CAN_InitStructure.CAN_RFLM=DISABLE;//���Ĳ�����,�µĸ��Ǿɵ�
		CAN_InitStructure.CAN_TTCM=DISABLE;//��ֹʱ�䴥��ͨ��ģʽ
		CAN_InitStructure.CAN_TXFP=DISABLE;//����FIFO���ȼ��ɱ��ı�ʶ������
		CAN_InitStructure.CAN_Mode=mode;

  	//���ò�����
  	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
		
		
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0020;//��ʶ���Ĵ���������֡31-21λ��0000 0000 0010 0000��ID=1
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFE0;//32λMASK��ֻMASKB����֡31-21λ��1111 1111 1110 0000
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//��ʶ��ģʽ
		CAN_FilterInitStructure.CAN_FilterNumber=0;//ָ������ʼ��������0
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32λ������
		CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��


    #if CAN_RX0_INT_ENABLE
	
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//��can�����жϣ�FIFO0��Ϣ�Һ��ж�����.		
    
    //can�����ж�
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
    #endif
		
   return 0;
  }   
	
	
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
	void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage; 
	RxMessage.StdId=0x00;  
	RxMessage.ExtId=0x00;  
	RxMessage.IDE=0;  
	RxMessage.DLC=0; 
  RxMessage.FMI=0;
	RxMessage.Data[0]=0x00; 
  RxMessage.Data[1]=0x00; 

  RxMessage.Data[2]=0x00;
	RxMessage.Data[3]=0x00; 
  RxMessage.Data[4]=0x00;  
	RxMessage.Data[5]=0x00;  
	RxMessage.Data[6]=0x00;  
	RxMessage.Data[7]=0x00;  
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);  
	if(RxMessage.IDE==CAN_ID_STD)  
		{      
			ret = 1;  
		} 
		else 
			{    
				ret = 0;  
			}
}
		    
//  void USB_LP_CAN1_RX0_IRQHandler(void)
//	  {
//				CanRxMsg RxMessage;
//				int i=0;
//				CAN_Receive(CAN1,0,&RxMessage);
//				for(i=0;i<8;i++)
//				printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
//    }
#endif

//can����һ������(�̶���ʽ:IDΪ0X0,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//����,ʧ��;
		
u8 Can_Send_Msg(u8* msg,u8 len)
  {	
			u8 mbox;
			u16 i=0;
			CanTxMsg TxMessage;
			TxMessage.StdId=0x01;			// ��׼��ʶ��Ϊ01
			TxMessage.ExtId=0x12;			// ������չ��ʾ����29λ��
			TxMessage.IDE=0;			    // ʹ����չ��ʶ��
			TxMessage.RTR=0;		      // ��Ϣ����Ϊ����֡��һ֡8λ
			TxMessage.DLC=len;				// ����֡��Ϣ
				
			for(i=0;i<len;i++)
				 TxMessage.Data[i] = msg[i];	// ��һ֡��Ϣ          
				 mbox = CAN_Transmit(CAN1, &TxMessage); 
				 i = 0;
			while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
				
			if(i>=0XFFF)return 1;
			return 0;		
  }
	
//can�ڽ������ݲ�ѯ;
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//����,���յ����ݳ���;
	
u8 Can_Receive_Msg(u8 *buf)
	{		   		   
	 	u32 i;
		CanRxMsg RxMessage;
	  if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
	  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	  for(i=0;i<8;i++)
		{
			buf[i]=RxMessage.Data[i];
		}
		return RxMessage.DLC;	
	}

//	while(1)
//	{
//		//can�������
//				for(i=0;i<8;i++)
//		{
//		buf[i] = i;//��䷢�ͻ�����
// 		}
//		Can_Receive_Msg(buf);
//		Can_Send_Msg(buf,8);//����8���ֽ�
//  }
//		
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			printf("\r\n�����͵���ϢΪ:\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART1->DR=USART_RX_BUF[t];
//				while((USART1->SR&0X40)==0);//�ȴ����ͽ���
//			}
//			printf("\r\n\r\n");//���뻻��
//			USART_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\n ����ʵ��\r\n");
//				printf("\r\n\r\n\r\n");
//			}
//			if(times%200==0)printf("����������,�Իس�������\r\n");  
//			if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
//			delay_ms(10);   
//		}













