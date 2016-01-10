#ifndef __DST_CAN_1_H__
#define __DST_CAN_1_H__
/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_RoboMasterInclude.h"




class DST_CAN_1{
	
public:
	
	DST_CAN_1();

	//CAN���ó�ʼ����
	void CAN_Configuration_Init(void);

	//����CAN1����
	uint8_t Send_CAN1_CMD(int16_t current_201,int16_t current_202,int16_t current_203);

	//��ѯ����
	uint8_t Receive_CAN1_MSG(uint8_t *buf);

	//CAN1�����жϵ��ú���
	void CAN1_RX0_IRQHandler_Fun(void);

private:
	
	CanRxMsg can1_rx_msg;
	
};
extern DST_CAN_1 can1;




























#endif

















