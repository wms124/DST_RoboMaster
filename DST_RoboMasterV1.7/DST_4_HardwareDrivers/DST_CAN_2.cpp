/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/


#include "DST_CAN_2.h"
#include "stm32f4xx_can.h"


DST_CAN_2 can2;


DST_CAN_2::DST_CAN_2()
{

}


void DST_CAN_2::CAN_Configuration_Init(void)
{
	GPIO_InitTypeDef				GPIO_InitStructure;	
	CAN_InitTypeDef			 		CAN_InitStructure;
 	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef 				NVIC_InitStructure;

	 //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_CAN_2, ENABLE);//使能PORTA时钟	                   											 

	//STM32F407 要使用CAN2 必须先使能CAN1时钟!!!!!
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟	

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_2_TX | CAN_2_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIO_CAN_2, &GPIO_InitStructure);//初始化PA11,PA12

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13复用为CAN2_TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12复用为CAN2_RX

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=DISABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=ENABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2 
	
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber=27;	  //过滤器0~27 如果和CAN1相同，则会导致无法进入中断
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//★FIFO0★消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


uint8_t DST_CAN_2::Send_CAN2_CMD(int16_t current_201,int16_t current_202,int16_t current_203)
{
	uint8_t mbox;	//发送邮箱
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
	
	mbox = CAN_Transmit(CAN2,&tx_message);
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return _DEFEATED;	//发送失败
  return _SUCCEED;	//发送成功
}
	
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		   其他,接收的数据长度;
uint8_t DST_CAN_2::Receive_CAN2_MSG(uint8_t *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;	//★关联FIFO0★
	if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;//没有接收到数据,直接退出 
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}



/*************************************************************************
                      RoboModule_Driver_Reset
函数描述：让挂接在CAN总线上的某个驱动器复位
传入参数：CAN_ID
*************************************************************************/
uint8_t DST_CAN_2::RoboModule_Driver_Reset(unsigned char CAN_ID)
{
	uint8_t mbox;	//发送邮箱
	uint16_t i= 0;
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    //标准帧
	tx_message.RTR = CAN_RTR_DATA;  //数据帧
	tx_message.DLC = 0x08;          //帧长度为8
	tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
	
	tx_message.Data[0] = 0x55;
	tx_message.Data[1] = 0x55;
	tx_message.Data[2] = 0x55;
	tx_message.Data[3] = 0x55;
	tx_message.Data[4] = 0x55;
	tx_message.Data[5] = 0x55;
	tx_message.Data[6] = 0x55;
	tx_message.Data[7] = 0x55;
	
//	can_tx_success_flag = 0;
	mbox = CAN_Transmit(CAN2,&tx_message);
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return _DEFEATED;	//发送失败
  return _SUCCEED;	//发送成功
//while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Mode_Chioce
函数描述：让挂接在CAN总线上的某个驱动器进入某种模式
传入参数：CAN_ID
传入参数：ENTER_X_MODE
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Mode_Chioce(unsigned char CAN_ID,unsigned char ENTER_X_MODE)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
    if((ENTER_X_MODE != ENTER_PWM_MODE)&&
       (ENTER_X_MODE != ENTER_CURRENT_MODE)&&
       (ENTER_X_MODE != ENTER_SPEED_MODE)&&
       (ENTER_X_MODE != ENTER_LOCATION_MODE))
    {
        ENTER_X_MODE = ENTER_PWM_MODE;
    }
    
    tx_message.Data[0] = ENTER_X_MODE;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_PWM_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在PWM模式下赋值
传入参数：CAN_ID
传入参数：PWM_Value
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_PWM_Mode_Set(unsigned char CAN_ID,short PWM_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((PWM_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(PWM_Value&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Current_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在Current模式下赋值
传入参数：CAN_ID
传入参数：Current_Value
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Current_Mode_Set(unsigned char CAN_ID,short Current_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID

    if(Current_Value > 2000)
    {
        Current_Value = 2000;
    }
    else if(Current_Value < -2000)
    {
        Current_Value = -2000;
    }

    tx_message.Data[0] = (unsigned char)((Current_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Current_Value&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Speed_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在Speed模式下赋值
传入参数：CAN_ID
传入参数：PWM_Value 
          Speed_Value
*************************************************************************/
uint8_t DST_CAN_2::RoboModule_Driver_Speed_Mode_Set(unsigned char CAN_ID,short PWM_Value,short Speed_Value)
{
	uint8_t mbox;	//发送邮箱
	uint16_t i= 0;
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    //标准帧
	tx_message.RTR = CAN_RTR_DATA;  //数据帧
	tx_message.DLC = 0x08;          //帧长度为8
	tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID

	if(PWM_Value > 5000)
	{
			PWM_Value = 5000;
	}
	else if(PWM_Value < -5000)
	{
			PWM_Value = -5000;
	}

	if(Speed_Value > 1000)
	{
			Speed_Value = 1000;
	}
	else if(Speed_Value < -1000)
	{
			Speed_Value = -1000;
	}

	tx_message.Data[0] = (unsigned char)((PWM_Value>>8)&0xff);
	tx_message.Data[1] = (unsigned char)(PWM_Value&0xff);
	tx_message.Data[2] = (unsigned char)((Speed_Value>>8)&0xff);
	tx_message.Data[3] = (unsigned char)(Speed_Value&0xff);
	tx_message.Data[4] = 0x55;
	tx_message.Data[5] = 0x55;
	tx_message.Data[6] = 0x55;
	tx_message.Data[7] = 0x55;

//can_tx_success_flag = 0;
	mbox = CAN_Transmit(CAN2,&tx_message);
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return _DEFEATED;	//发送失败
  return _SUCCEED;	//发送成功
//	CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Location_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在Location模式下赋值
传入参数：CAN_ID
传入参数：PWM_Value 
          Speed_Value 
          Location_Mode
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Location_Mode_Set(unsigned char CAN_ID,short PWM_Value,short Speed_Value,int Location_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID

    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }

    if(Speed_Value > 1000)
    {
        Speed_Value = 1000;
    }
    else if(Speed_Value < -1000)
    {
        Speed_Value = -1000;
    }
    
    if(Location_Value > 5000000)
    {
        Location_Value = 5000000;
    }
    else if(Location_Value < -5000000)
    {
        Location_Value = -5000000;
    }

    tx_message.Data[0] = (unsigned char)((PWM_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(PWM_Value&0xff);
    tx_message.Data[2] = (unsigned char)((Speed_Value>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Speed_Value&0xff);
    tx_message.Data[4] = (unsigned char)((Location_Value>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Location_Value>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Location_Value>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Location_Value&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}









































