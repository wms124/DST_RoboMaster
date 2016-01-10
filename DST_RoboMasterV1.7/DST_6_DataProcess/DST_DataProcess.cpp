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

/* 从HG900驱动在6025电机上返回的机械角度值中计算出位置信息
 * 根据机械角返回的数据0~8191代表电机逆时针（轴心向上）转过一圈。
 * @param  rx_msg：CAN总线接收到的数据消息，格式如下：
 * -------------------------------------------------------------------------------------------------
 * |数据域| DATA[0] | DATA[1] |  DATA[2]  |  DATA[3]  |  DATA[4]  |  DATA[5]  |  DATA[6] | DATA[7] |
 * |------|——-----|---------|-----------|-----------|-----------|-----------|----------|---------|
 * | 内容 |机械角高8|机械角低8|实际电流高8|实际电流低8|给定电流高8|给定电流低8|霍尔开关值|  Null   |
 * -------------------------------------------------------------------------------------------------
 */
void DST_DataProcess::HG900_6025_DataPro(CanRxMsg* rx_msg)
{
	switch(rx_msg->StdId)
	{
		case PITCH_ID:
			//得到机械角度值
			//如果是第一次读取PIT
			if(Pit_first_read == 1)
			{
				Pit_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
			}
			else
			{
				Pit_data = ((rx_msg->Data[0])<<8)|(rx_msg->Data[1]);
				Pit_first_data = Pit_data;//偏移
				Pit_olddata = Pit_data;
				Pit_first_read = 1;
			}
			//判断上一次数据与本次数据中是否旋转过了一圈，转过一圈Pit_location_temp++，反转一圈则Pit_location_temp--
			if((Pit_data<2000)&&(Pit_olddata>5000))
			{
				Pit_location_temp++;
			}
			else if((Pit_data>5000)&&(Pit_olddata<2000))
			{
				Pit_location_temp--;
			}
			Pit_olddata = Pit_data;
			//位置 = 转过的圈数 * 每圈的角度值 + 现在的角度值
			Pit_location = Pit_location_temp * 8191 + Pit_data - Pit_first_data;//mechanical angle value(0~8191)  减去偏移
			break;
		case YAW_ID:
			//得到机械角度值
			//如果是第一次读取YAW
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
			//判断上一次数据与本次数据中是否旋转过了一圈，转过一圈Pit_location_temp++，反转一圈则Pit_location_temp--
			if((Yaw_data<2000)&&(Yaw_olddata>5000))
			{
				Yaw_location_temp++;
			}
			else if((Yaw_data>5000)&&(Yaw_olddata<2000))
			{
				Yaw_location_temp--;
			}
			Yaw_olddata = Yaw_data;
			//位置 = 转过的圈数 * 每圈的角度值 + 现在的角度值
			Yaw_location = Yaw_location_temp * 8191 + Yaw_data - Yaw_first_data;//mechanical angle value(0~8191)  减去偏移
			break;
	}
}

/* DBUS遥控数据接收，数据帧格式：
 * 遥控信息：
 * --------------------------------------------------------------------
 * |  域  |  通道0  |  通道1  |  通道2  |  通道3  |    S1   |    S2   |
 * |------|——-----|---------|---------|---------|---------|---------|
 * | 偏移 |    0    |   11    |    22   |    33   |    44   |    46   |    
 * |------|——-----|---------|---------|---------|---------|---------|
 * | 长度 |   11    |   11    |    11   |   11    |     2   |    2    |    
 * |------|——-----|---------|---------|---------|---------|---------|
 * |      |最大1684 |最大1684 |最大1684 |最大1684 |最大  3  |最大  3  |
 * | 范围 |中间1024 |中间1024 |中间1024 |中间1024 |中间  2  |中间  2  |
 * |      |最小 364 |最小 364 |最小 364 |最小 364 |最小  1  |最小  1  |
 * --------------------------------------------------------------------
 * 鼠标信息：
 * ----------------------------------------------------------------------
 * |  域  |  鼠标x轴  |  鼠标y轴  |  鼠标z轴  |  鼠标左键  |  鼠标右键  |
 * |------|——-------|-----------|-----------|------------|------------|
 * | 偏移 |     48    |     64    |     80    |     86     |     94     |
 * |------|——-------|-----------|-----------|------------|------------|
 * | 长度 |     16    |     16    |     16    |      8     |      8     |
 * |------|——-------|-----------|-----------|------------|------------|
 * |      |最大  32767|最大  32767|最大  32767|最大  1     |最大  1     |
 * | 范围 |最小 -32768|最小 -32768|最小 -32768|最小  0     |最小  0     |
 * |      |静止值  0  |静止值  0  |静止值  0  |            |            |
 * ----------------------------------------------------------------------
 * 键盘信息：
 * ---------——-------------------------
 * |  域  | 按键									      |
 * |------|——-------------------------|
 * | 偏移 | 102													|
 * |------|——-------------------------|
 * | 长度 | 16  												|
 * |------|——-------------------------|
 * |      |bit0 -----> W键              |
 * | 功能 |bit1 -----> S键              |
 * |      |bit2 -----> A键              |
 * |      |bit3 -----> D键              |
 * |      |bit4 -----> Q键              |
 * |      |bit5 -----> E键              |
 * |      |bit6 -----> Shift键  			  |
 * |      |bit7 -----> Ctrl键     			|
 * --------------------------------------
 */
void DST_DataProcess::DBUS_DataReceive(void)
{
	//遥控数据处理，遥控4个通道数据范围：364~1684，中间值为1024。
	//(遥控通道数据 - 0x400) 得到 -660~660 范围变化的遥控通道数据
	dbus.RC_Ctl.rc_660.ch0 = (((dbus.dbus_rx_buffer[0]| (dbus.dbus_rx_buffer[1] << 8)) & 0x07ff) - 0x400);	//!< Channel_0   value:-660~660  	
	dbus.RC_Ctl.rc_660.ch1 = ((((dbus.dbus_rx_buffer[1] >> 3) | (dbus.dbus_rx_buffer[2] << 5)) & 0x07ff) - 0x400); //!< Channel_1  value:-660~660  	
	dbus.RC_Ctl.rc_660.ch2 = ((((dbus.dbus_rx_buffer[2] >> 6) | (dbus.dbus_rx_buffer[3] << 2) | (dbus.dbus_rx_buffer[4] << 10)) & 0x07ff)- 0x400);//!< Channel_2  value:-660~660  	
	dbus.RC_Ctl.rc_660.ch3 = ((((dbus.dbus_rx_buffer[4] >> 1) | (dbus.dbus_rx_buffer[5] << 7)) & 0x07ff) - 0x400); //!< Channel_3  value:-660~660  	
	//(遥控通道数据 - 0x400) * 1.5151 +1000 得到 0~2000 范围变化的遥控通道数据
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
//	_data_pro.DBUS_DataPro();//DBUS 数据处理，底盘电机RM35控制
}

/***********DBUS数据处理函数*************/
void DST_DataProcess::DBUS_DataPro(void)
{
	Wheel_Rot = dbus.RC_Ctl.rc_660.ch0 / 20;
	Wheel_L_R = dbus.RC_Ctl.rc_660.ch3 / 20;
	Wheel_F_B = dbus.RC_Ctl.rc_660.ch2 / 20;
	if(ABS(Wheel_F_B) <= 1) //左右平移
	{
		Wheel_F_B = 0;
	}
	if(ABS(Wheel_L_R) <=1)
	{
		Wheel_L_R = 0;
	}
	/*--------------RM35底盘电机控制--------------*/
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM01,5000,Wheel_F_B + Wheel_L_R - Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM02,5000,Wheel_F_B - Wheel_L_R - Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM04,5000,Wheel_F_B + Wheel_L_R + Wheel_Rot);
	can2.RoboModule_Driver_Speed_Mode_Set(CAN_ID_SPEED_MODE_NUM03,5000,Wheel_F_B - Wheel_L_R + Wheel_Rot);
}

//得到Pit计算置位
int16_t DST_DataProcess::Get_Pit_Location(void)
{
	return Pit_location;
}
//得到Yaw计算置位
int16_t DST_DataProcess::Get_Yaw_Location(void)
{
	return Yaw_location;
}























