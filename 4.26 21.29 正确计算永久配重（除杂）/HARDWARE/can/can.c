#include "can.h"

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟(APB1)在初始化的时候设置为64/2=32M,如果设置CAN_Normal_Init(1,4,3,4,1);
//则波特率为:32M/((1+4+3)*4)=1MKbps
//返回值:0,初始化OK;
//其他,初始化失败;

u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
  {
	  GPIO_InitTypeDef GPIO_InitStructure; 
  	CAN_InitTypeDef CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
    #if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
    #endif

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE); //端口复用
	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	  
		//CAN单元设置
		CAN_InitStructure.CAN_ABOM=DISABLE;//软件自动离线管理，软件对CAN_MCR寄存器的INRQ位进行置1随后清0后，一旦硬件检测
		//到128次11位连续的隐性位，就退出离线状态
		CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒
		CAN_InitStructure.CAN_NART=ENABLE;//禁止报文自动传送，报文只被发送一次
		CAN_InitStructure.CAN_RFLM=DISABLE;//报文不锁定,新的覆盖旧的
		CAN_InitStructure.CAN_TTCM=DISABLE;//禁止时间触发通信模式
		CAN_InitStructure.CAN_TXFP=DISABLE;//发送FIFO优先级由报文标识符决定
		CAN_InitStructure.CAN_Mode=mode;

  	//设置波特率
  	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
		
		
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0020;//标识符寄存器，数据帧31-21位：0000 0000 0010 0000，ID=1
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFE0;//32位MASK，只MASKB数据帧31-21位：1111 1111 1110 0000
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//标识符模式
		CAN_FilterInitStructure.CAN_FilterNumber=0;//指定待初始化过滤器0
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32位过滤器
		CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化


    #if CAN_RX0_INT_ENABLE
	
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//打开can接受中断，FIFO0消息挂号中断允许.		
    
    //can向量中断
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
    #endif
		
   return 0;
  }   
	
	
#if CAN_RX0_INT_ENABLE	//使能RX0中断
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

//can发送一组数据(固定格式:ID为0X0,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//其他,失败;
		
u8 Can_Send_Msg(u8* msg,u8 len)
  {	
			u8 mbox;
			u16 i=0;
			CanTxMsg TxMessage;
			TxMessage.StdId=0x01;			// 标准标识符为01
			TxMessage.ExtId=0x12;			// 设置扩展标示符（29位）
			TxMessage.IDE=0;			    // 使用扩展标识符
			TxMessage.RTR=0;		      // 消息类型为数据帧，一帧8位
			TxMessage.DLC=len;				// 发送帧信息
				
			for(i=0;i<len;i++)
				 TxMessage.Data[i] = msg[i];	// 第一帧信息          
				 mbox = CAN_Transmit(CAN1, &TxMessage); 
				 i = 0;
			while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
				
			if(i>=0XFFF)return 1;
			return 0;		
  }
	
//can口接收数据查询;
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//其他,接收的数据长度;
	
u8 Can_Receive_Msg(u8 *buf)
	{		   		   
	 	u32 i;
		CanRxMsg RxMessage;
	  if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	  for(i=0;i<8;i++)
		{
			buf[i]=RxMessage.Data[i];
		}
		return RxMessage.DLC;	
	}

//	while(1)
//	{
//		//can代码测试
//				for(i=0;i<8;i++)
//		{
//		buf[i] = i;//填充发送缓冲区
// 		}
//		Can_Receive_Msg(buf);
//		Can_Send_Msg(buf,8);//发送8个字节
//  }
//		
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//			printf("\r\n您发送的消息为:\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART1->DR=USART_RX_BUF[t];
//				while((USART1->SR&0X40)==0);//等待发送结束
//			}
//			printf("\r\n\r\n");//插入换行
//			USART_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\n 串口实验\r\n");
//				printf("\r\n\r\n\r\n");
//			}
//			if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
//			if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
//			delay_ms(10);   
//		}













