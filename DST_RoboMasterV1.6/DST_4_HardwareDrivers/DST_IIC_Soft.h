#ifndef __DST_IIC_SOFT_H__
#define __DST_IIC_SOFT_H__
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
#include "DST_IIC_Soft.h"


class DST_IIC_Soft{

public:
		
	DST_IIC_Soft();
	void IIC_Soft_Init(void);
	uint8_t IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
	uint8_t IIC_Read_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *REG_data);
	uint8_t IIC_Write_MultByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);
	uint8_t IIC_Read_MultByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);

private:
	void IIC_Delay(void);
	int  IIC_Start(void);	
	void IIC_Stop(void);
	void IIC_Ack(void);
	void IIC_NoAck(void);
	int  IIC_WaitAck(void);
	void IIC_SendByte(uint8_t SendByte);
	uint8_t IIC_ReadByte(uint8_t ack);
};
extern DST_IIC_Soft iic;













#endif











