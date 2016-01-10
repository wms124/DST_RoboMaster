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

 //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_CAN_1, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_1_TX | CAN_1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIO_CAN_1, &GPIO_InitStructure);//��ʼ��CAN_1_TX,CAN_1_RX

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOD1����ΪCAN1_TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOD0����ΪCAN1_RX

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
	
	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//��FIFO0����Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//����CAN1����
uint8_t DST_CAN_1::Send_CAN1_CMD(int16_t current_201,int16_t current_202,int16_t current_203)
{
	uint8_t mbox;					//��������
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
	
	mbox = CAN_Transmit(CAN1,&tx_message);
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return _DEFEATED;	//����ʧ��
	return _SUCCEED;	//���ͳɹ�
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		   ����,���յ����ݳ���;
uint8_t DST_CAN_1::Receive_CAN1_MSG(uint8_t *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;	//�����FIFO0��
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;//û�н��յ�����,ֱ���˳� 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


//CAN1�����жϵ��ú���
void DST_CAN_1::CAN1_RX0_IRQHandler_Fun(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		/********HG900 6025���ݽ���********/
		_data_pro.HG900_6025_DataPro(&can1_rx_msg);//CAN1��̨���ݴ���
		//test fun
		ws(&can1_rx_msg);
	}
}



















