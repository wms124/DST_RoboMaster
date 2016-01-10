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

	 //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_CAN_2, ENABLE);//ʹ��PORTAʱ��	                   											 

	//STM32F407 Ҫʹ��CAN2 ������ʹ��CAN1ʱ��!!!!!
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��	

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_2_TX | CAN_2_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIO_CAN_2, &GPIO_InitStructure);//��ʼ��PA11,PA12

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13����ΪCAN2_TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12����ΪCAN2_RX

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=ENABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2 
	
	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber=27;	  //������0~27 �����CAN1��ͬ����ᵼ���޷������ж�
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//��FIFO0����Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


uint8_t DST_CAN_2::Send_CAN2_CMD(int16_t current_201,int16_t current_202,int16_t current_203)
{
	uint8_t mbox;	//��������
	uint16_t i= 0;
	CanTxMsg tx_message;	//CAN Tx message structure definition
	
	tx_message.StdId = 0x200;					//��׼ID
	tx_message.IDE = CAN_Id_Standard;	//��׼֡
	tx_message.RTR = CAN_RTR_Data;		//����֡
	tx_message.DLC = 0x08;						//Ҫ���͵����ݳ���
	
	/*********************����*********************/
	tx_message.Data[0] = (unsigned char)(current_201 >> 8);
	tx_message.Data[1] = (unsigned char)current_201;
	tx_message.Data[2] = (unsigned char)(current_202 >> 8);
	tx_message.Data[3] = (unsigned char)current_202;
	tx_message.Data[4] = (unsigned char)(current_203 >> 8);
	tx_message.Data[5] = (unsigned char)current_203;
	tx_message.Data[6] = 0x00;
	tx_message.Data[7] = 0x00;
	
	mbox = CAN_Transmit(CAN2,&tx_message);
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return _DEFEATED;	//����ʧ��
  return _SUCCEED;	//���ͳɹ�
}
	
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		   ����,���յ����ݳ���;
uint8_t DST_CAN_2::Receive_CAN2_MSG(uint8_t *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;	//�����FIFO0��
	if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;//û�н��յ�����,ֱ���˳� 
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}



/*************************************************************************
                      RoboModule_Driver_Reset
�����������ùҽ���CAN�����ϵ�ĳ����������λ
���������CAN_ID
*************************************************************************/
uint8_t DST_CAN_2::RoboModule_Driver_Reset(unsigned char CAN_ID)
{
	uint8_t mbox;	//��������
	uint16_t i= 0;
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    //��׼֡
	tx_message.RTR = CAN_RTR_DATA;  //����֡
	tx_message.DLC = 0x08;          //֡����Ϊ8
	tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID
	
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
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return _DEFEATED;	//����ʧ��
  return _SUCCEED;	//���ͳɹ�
//while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Mode_Chioce
�����������ùҽ���CAN�����ϵ�ĳ������������ĳ��ģʽ
���������CAN_ID
���������ENTER_X_MODE
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Mode_Chioce(unsigned char CAN_ID,unsigned char ENTER_X_MODE)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID
    
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
�������������ҽ���CAN�����ϵ�ĳ����������PWMģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_PWM_Mode_Set(unsigned char CAN_ID,short PWM_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID
    
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
�������������ҽ���CAN�����ϵ�ĳ����������Currentģʽ�¸�ֵ
���������CAN_ID
���������Current_Value
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Current_Mode_Set(unsigned char CAN_ID,short Current_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID

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
�������������ҽ���CAN�����ϵ�ĳ����������Speedģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value 
          Speed_Value
*************************************************************************/
uint8_t DST_CAN_2::RoboModule_Driver_Speed_Mode_Set(unsigned char CAN_ID,short PWM_Value,short Speed_Value)
{
	uint8_t mbox;	//��������
	uint16_t i= 0;
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    //��׼֡
	tx_message.RTR = CAN_RTR_DATA;  //����֡
	tx_message.DLC = 0x08;          //֡����Ϊ8
	tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID

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
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return _DEFEATED;	//����ʧ��
  return _SUCCEED;	//���ͳɹ�
//	CAN_Transmit(CAN2,&tx_message);
//    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Location_Mode_Set
�������������ҽ���CAN�����ϵ�ĳ����������Locationģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value 
          Speed_Value 
          Location_Mode
*************************************************************************/
void DST_CAN_2::RoboModule_Driver_Location_Mode_Set(unsigned char CAN_ID,short PWM_Value,short Speed_Value,int Location_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    tx_message.StdId = CAN_ID;      //֡IDΪ���������CAN_ID

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









































