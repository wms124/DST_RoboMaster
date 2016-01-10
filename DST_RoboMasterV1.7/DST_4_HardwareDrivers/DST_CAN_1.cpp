/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/


#include "DST_CAN_1.h"
#include "stm32f4xx_can.h"


DST_CAN_1 can1;

DST_CAN_1::DST_CAN_1()
{
	
}


void DST_CAN_1::CAN_Configuration_Init(void)
{
	GPIO_InitTypeDef				GPIO_InitStructure;	
	CAN_InitTypeDef			 		CAN_InitStructure;
 	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef 				NVIC_InitStructure;

 //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_CAN_1, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_1_TX | CAN_1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIO_CAN_1, &GPIO_InitStructure);//初始化CAN_1_TX,CAN_1_RX

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOD1复用为CAN1_TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOD0复用为CAN1_RX

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
	
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//★FIFO0★消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//发送CAN1命令
uint8_t DST_CAN_1::Send_CAN1_CMD(int16_t current_201,int16_t current_202,int16_t current_203)
{
	uint8_t mbox;					//发送邮箱
	uint16_t i= 0;
	CanTxMsg tx_message;	//CAN Tx message structure definition
	
	tx_message.StdId = 0x200;					//标准ID
	tx_message.IDE = CAN_Id_Standard;	//标准帧
	tx_message.RTR = CAN_RTR_Data;		//数据帧
	tx_message.DLC = 0x08;						//要发送的数据长度
	
	/*********************数据*********************/
	tx_message.Data[0] = (unsigned char)(current_201 >> 8);
	tx_message.Data[1] = (unsigned char)current_201;
	tx_message.Data[2] = (unsigned char)(current_202 >> 8);
	tx_message.Data[3] = (unsigned char)current_202;
	tx_message.Data[4] = (unsigned char)(current_203 >> 8);
	tx_message.Data[5] = (unsigned char)current_203;
	tx_message.Data[6] = 0x00;
	tx_message.Data[7] = 0x00;
	
	mbox = CAN_Transmit(CAN1,&tx_message);
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return _DEFEATED;	//发送失败
	return _SUCCEED;	//发送成功
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		   其他,接收的数据长度;
uint8_t DST_CAN_1::Receive_CAN1_MSG(uint8_t *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;	//★关联FIFO0★
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;//没有接收到数据,直接退出 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


//CAN1接收中断调用函数
void DST_CAN_1::CAN1_RX0_IRQHandler_Fun(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		/********HG900 6025数据接收********/
		_data_pro.HG900_6025_DataPro(&can1_rx_msg);//CAN1云台数据处理
		//test fun
		ws(&can1_rx_msg);
	}
}



















