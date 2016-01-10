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



DST_IIC_Soft iic;

DST_IIC_Soft::DST_IIC_Soft()
{

}


void DST_IIC_Soft::IIC_Soft_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	//��©
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}


void DST_IIC_Soft::IIC_Delay(void)
{
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	//__nop();__nop();__nop();
}


int DST_IIC_Soft::IIC_Start(void)
{
	SDA_H;
	SCL_H;
	IIC_Delay();
	if(!SDA_R)return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	IIC_Delay();
	if(SDA_R)return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;
	IIC_Delay();
	return 1;
}

void DST_IIC_Soft::IIC_Stop(void)
{
	SCL_L;
	IIC_Delay();
	SDA_L;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SDA_H;
	IIC_Delay();
}

void DST_IIC_Soft::IIC_Ack(void)
{
	SCL_L;
	IIC_Delay();
	SDA_L;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SCL_L;
	IIC_Delay();
}

void DST_IIC_Soft::IIC_NoAck(void)
{
	SCL_L;
	IIC_Delay();
	SDA_H;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SCL_L;
	IIC_Delay();
}

//����Ϊ:=1��ASK,=0��ASK
int DST_IIC_Soft::IIC_WaitAck(void)
{
	uint8_t ErrTime = 0;
	SCL_L;
	IIC_Delay();
	SDA_H;			
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	while(SDA_R)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;
	IIC_Delay();
	return 0;
}

//���ݴӸ�λ����λ
void DST_IIC_Soft::IIC_SendByte(uint8_t SendByte)	
{
	u8 i=8;
	while(i--)
	{
		SCL_L;
		IIC_Delay();
		if(SendByte&0x80)
			SDA_H;  
		else 
			SDA_L;   
		SendByte<<=1;
		IIC_Delay();
		SCL_H;
		IIC_Delay();
	}
	SCL_L;	
}


//��1���ֽڣ�ack=1ʱ������ACK��ack=0������NACK
uint8_t DST_IIC_Soft::IIC_ReadByte(uint8_t ack)
{
	u8 i=8;
	u8 ReceiveByte=0;

	SDA_H;				
	while(i--)
	{
		ReceiveByte<<=1;      
		SCL_L;
		IIC_Delay();
		SCL_H;
		IIC_Delay();	
		if(SDA_R)
		{
			ReceiveByte|=0x01;
		}
	}
	SCL_L;
	if (ack)
		IIC_Ack();
	else
		IIC_NoAck();  
	return ReceiveByte;	
}


// IICдһ���ֽ�����
uint8_t DST_IIC_Soft::IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1);   
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(REG_Address);       
	IIC_WaitAck();	
	IIC_SendByte(REG_data);
	IIC_WaitAck();   
	IIC_Stop(); 
	return 0;
}


// IIC��һ�ֽ�����
uint8_t DST_IIC_Soft::IIC_Read_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *REG_data)
{      		
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1); 
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(REG_Address);     
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1 | 0x01);
	IIC_WaitAck();
	*REG_data= IIC_ReadByte(0);
	IIC_Stop();
	return 0;
}	

// IICд���ֽ�����
uint8_t DST_IIC_Soft::IIC_Write_MultByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, u8 *buf)
{	
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1); 
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(REG_Address); 
	IIC_WaitAck();
	while(len--) 
	{
		IIC_SendByte(*buf++); 
		IIC_WaitAck();
	}
	IIC_Stop();
	return 0;
}


// IIC�����ֽ�����
uint8_t DST_IIC_Soft::IIC_Read_MultByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf)
{	
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1); 
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(REG_Address); 
	IIC_WaitAck();
	
	IIC_Start();
	IIC_SendByte(SlaveAddress<<1 | 0x01); 
	IIC_WaitAck();
	while(len) 
	{
		if(len == 1)
		{
			*buf = IIC_ReadByte(0);
		}
		else
		{
			*buf = IIC_ReadByte(1);
		}
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}






