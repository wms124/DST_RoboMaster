/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_DataProcess.h"



DST_DataProcess _data_pro;


DST_DataProcess::DST_DataProcess()
{
	Pit_first_read = 0;
	Yaw_first_read = 0;
	
	Pit_location_temp= 0;
	Yaw_location_temp = 0;
}

/* ��HG900������6025����Ϸ��صĻ�е�Ƕ�ֵ�м����λ����Ϣ
 * ���ݻ�е�Ƿ��ص�����0~8191��������ʱ�루�������ϣ�ת��һȦ��
 * @param  rx_msg��CAN���߽��յ���������Ϣ����ʽ���£�
 * -------------------------------------------------------------------------------------------------
 * |������| DATA[0] | DATA[1] |  DATA[2]  |  DATA[3]  |  DATA[4]  |  DATA[5]  |  DATA[6] | DATA[7] |
 * |------|����-----|---------|-----------|-----------|-----------|-----------|----------|---------|
 * | ���� |��е�Ǹ�8|��е�ǵ�8|ʵ�ʵ�����8|ʵ�ʵ�����8|����������8|����������8|��������ֵ|  Null   |
 * -------------------------------------------------------------------------------------------------
 */
void DST_DataProcess::HG900_6025_DataPro(CanRxMsg* rx_msg)
{
	switch(rx_msg->StdId)
	{
		case PITCH_ID:
			//�õ���е�Ƕ�ֵ
			//����ǵ�һ�ζ�ȡPIT
			if(Pit_first_read == 1)
			{
				Pit_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
			}
			else
			{
				Pit_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
				Pit_first_data = Pit_data;//ƫ��
				Pit_olddata = Pit_data;
				Pit_first_read = 1;
			}
			//�ж���һ�������뱾���������Ƿ���ת����һȦ��ת��һȦPit_location_temp++����תһȦ��Pit_location_temp--
			if((Pit_data<2000)&&(Pit_olddata>5000))
			{
				Pit_location_temp++;
			}
			else if((Pit_data>5000)&&(Pit_olddata<2000))
			{
				Pit_location_temp--;
			}
			Pit_olddata = Pit_data;
			//λ�� = ת����Ȧ�� * ÿȦ�ĽǶ�ֵ + ���ڵĽǶ�ֵ
			Pit_location = Pit_location_temp * 8191 + Pit_data - Pit_first_data;//mechanical angle value(0~8191)  ��ȥƫ��
			break;
		case YAW_ID:
			//�õ���е�Ƕ�ֵ
			//����ǵ�һ�ζ�ȡYAW
			if(Yaw_first_read == 1)
			{
				Yaw_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
			}
			else
			{
				Yaw_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
				Yaw_first_data = Yaw_data;
				Yaw_olddata = Yaw_data;
				Yaw_first_read = 1;
			}
			//�ж���һ�������뱾���������Ƿ���ת����һȦ��ת��һȦPit_location_temp++����תһȦ��Pit_location_temp--
			if((Yaw_data<2000)&&(Yaw_olddata>5000))
			{
				Yaw_location_temp++;
			}
			else if((Yaw_data>5000)&&(Yaw_olddata<2000))
			{
				Yaw_location_temp--;
			}
			Yaw_olddata = Yaw_data;
			//λ�� = ת����Ȧ�� * ÿȦ�ĽǶ�ֵ + ���ڵĽǶ�ֵ
			Yaw_location = Yaw_location_temp * 8191 + Yaw_data - Yaw_first_data;//mechanical angle value(0~8191)  ��ȥƫ��
			break;
	}
}

/* DBUSң�����ݽ��գ�����֡��ʽ��
 * ң����Ϣ��
 * --------------------------------------------------------------------
 * |  ��  |  ͨ��0  |  ͨ��1  |  ͨ��2  |  ͨ��3  |    S1   |    S2   |
 * |------|����-----|---------|---------|---------|---------|---------|
 * | ƫ�� |    0    |   11    |    22   |    33   |    44   |    46   |    
 * |------|����-----|---------|---------|---------|---------|---------|
 * | ���� |   11    |   11    |    11   |   11    |     2   |    2    |    
 * |------|����-----|---------|---------|---------|---------|---------|
 * |      |���1684 |���1684 |���1684 |���1684 |���  3  |���  3  |
 * | ��Χ |�м�1024 |�м�1024 |�м�1024 |�м�1024 |�м�  2  |�м�  2  |
 * |      |��С 364 |��С 364 |��С 364 |��С 364 |��С  1  |��С  1  |
 * --------------------------------------------------------------------
 * �����Ϣ��
 * ----------------------------------------------------------------------
 * |  ��  |  ���x��  |  ���y��  |  ���z��  |  ������  |  ����Ҽ�  |
 * |------|����-------|-----------|-----------|------------|------------|
 * | ƫ�� |     48    |     64    |     80    |     86     |     94     |
 * |------|����-------|-----------|-----------|------------|------------|
 * | ���� |     16    |     16    |     16    |      8     |      8     |
 * |------|����-------|-----------|-----------|------------|------------|
 * |      |���  32767|���  32767|���  32767|���  1     |���  1     |
 * | ��Χ |��С -32768|��С -32768|��С -32768|��С  0     |��С  0     |
 * |      |��ֵֹ  0  |��ֵֹ  0  |��ֵֹ  0  |            |            |
 * ----------------------------------------------------------------------
 * ������Ϣ��
 * ---------����-------------------------
 * |  ��  | ����									      |
 * |------|����-------------------------|
 * | ƫ�� | 102													|
 * |------|����-------------------------|
 * | ���� | 16  												|
 * |------|����-------------------------|
 * |      |bit0 -----> W��              |
 * | ���� |bit1 -----> S��              |
 * |      |bit2 -----> A��              |
 * |      |bit3 -----> D��              |
 * |      |bit4 -----> Q��              |
 * |      |bit5 -----> E��              |
 * |      |bit6 -----> Shift��  			  |
 * |      |bit7 -----> Ctrl��     			|
 * --------------------------------------
 */
void DST_DataProcess::DBUS_DataReceive(void)
{
	//ң�����ݴ���ң��4��ͨ�����ݷ�Χ��364~1684���м�ֵΪ1024��
	//(ң��ͨ������ - 0x400) �õ� -660~660 ��Χ�仯��ң��ͨ������
	dbus.RC_Ctl.rc_660.ch0 = (((dbus.dbus_rx_buffer[0]| (dbus.dbus_rx_buffer[1] << 8)) & 0x07ff) - 0x400);	//!< Channel_0   value:-660~660  	
	dbus.RC_Ctl.rc_660.ch1 = ((((dbus.dbus_rx_buffer[1] >> 3) | (dbus.dbus_rx_buffer[2] << 5)) & 0x07ff) - 0x400); //!< Channel_1  value:-660~660  	
	dbus.RC_Ctl.rc_660.ch2 = ((((dbus.dbus_rx_buffer[2] >> 6) | (dbus.dbus_rx_buffer[3] << 2) | (dbus.dbus_rx_buffer[4] << 10)) & 0x07ff)- 0x400);//!< Channel_2  value:-660~660  	
	dbus.RC_Ctl.rc_660.ch3 = ((((dbus.dbus_rx_buffer[4] >> 1) | (dbus.dbus_rx_buffer[5] << 7)) & 0x07ff) - 0x400); //!< Channel_3  value:-660~660  	
	//(ң��ͨ������ - 0x400) * 1.5151 +1000 �õ� 0~2000 ��Χ�仯��ң��ͨ������
	dbus.RC_Ctl.rc_2000.ch0 = dbus.RC_Ctl.rc_660.ch0 * 1.5151 + 1000; //!< Channel_0   value:0~2000  	
	dbus.RC_Ctl.rc_2000.ch1 = dbus.RC_Ctl.rc_660.ch1 * 1.5151 + 1000; //!< Channel_1   value:0~2000  	
	dbus.RC_Ctl.rc_2000.ch2 = dbus.RC_Ctl.rc_660.ch2 * 1.5151 + 1000; //!< Channel_2   value:0~2000  	
	dbus.RC_Ctl.rc_2000.ch3 = dbus.RC_Ctl.rc_660.ch3 * 1.5151 + 1000; //!< Channel_3   value:0~2000  	
	dbus.RC_Ctl.rc_660.s1 = ((dbus.dbus_rx_buffer[5] >> 4)& 0x03); //!< Switch left 
	dbus.RC_Ctl.rc_660.s2 = ((dbus.dbus_rx_buffer[5] >> 6)& 0x03);  //!< Switch right
	dbus.RC_Ctl.mouse.x = dbus.dbus_rx_buffer[6] | ((int16_t)dbus.dbus_rx_buffer[7]  << 8); //!< Mouse X axis
	dbus.RC_Ctl.mouse.y = dbus.dbus_rx_buffer[8] | ((int16_t)dbus.dbus_rx_buffer[9]  << 8); //!< Mouse Y axis
	dbus.RC_Ctl.mouse.z = dbus.dbus_rx_buffer[10] | ((int16_t)dbus.dbus_rx_buffer[11] << 8); //!< Mouse Z axis
	dbus.RC_Ctl.mouse.press_l = dbus.dbus_rx_buffer[12];  //!< Mouse Left Is Press 
	dbus.RC_Ctl.mouse.press_r = dbus.dbus_rx_buffer[13];  //!< Mouse Right Is Press 
	dbus.RC_Ctl.key.v = dbus.dbus_rx_buffer[14] | ((uint16_t)dbus.dbus_rx_buffer[15]  << 8); //!< KeyBoard value
	//Convert();
//	_data_pro.DBUS_DataPro();//DBUS ���ݴ������̵��RM35����
}

/***********DBUS���ݴ�����*************/
void DST_DataProcess::DBUS_DataPro(void)
{
	Wheel_Rot = dbus.RC_Ctl.rc_660.ch0 / 20;
	Wheel_L_R = dbus.RC_Ctl.rc_660.ch3 / 20;
	Wheel_F_B = dbus.RC_Ctl.rc_660.ch2 / 20;
	if(ABS(Wheel_F_B) <= 1) //����ƽ��
	{
		Wheel_F_B = 0;
	}
	if(ABS(Wheel_L_R) <=1)
	{
		Wheel_L_R = 0;
	}
	/*--------------RM35���̵������--------------*/
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM01,5000,Wheel_F_B + Wheel_L_R - Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM02,5000,Wheel_F_B - Wheel_L_R - Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM04,5000,Wheel_F_B + Wheel_L_R + Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM03,5000,Wheel_F_B - Wheel_L_R + Wheel_Rot);
}

//�õ�Pit������λ
int16_t DST_DataProcess::Get_Pit_Location(void)
{
	return Pit_location;
}
//�õ�Yaw������λ
int16_t DST_DataProcess::Get_Yaw_Location(void)
{
	return Yaw_location;
}























