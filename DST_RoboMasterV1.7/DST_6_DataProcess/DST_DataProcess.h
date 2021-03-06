#ifndef __DST_DATAPROCESS_H__
#define __DST_DATAPROCESS_H__
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



class DST_DataProcess{

public:

	DST_DataProcess();

	/* 从HG900驱动在6025电机上返回的机械角度值中计算出位置信息
	 * 根据机械角返回的数据0~8191代表电机逆时针（轴心向上）转过一圈。
	 * @param  rx_msg：CAN总线接收到的数据消息，格式如下：
   * -------------------------------------------------------------------------------------------------
   * |数据域| DATA[0] | DATA[1] |  DATA[2]  |  DATA[3]  |  DATA[4]  |  DATA[5]  |  DATA[6] | DATA[7] |
	 * |------|——-----|---------|-----------|-----------|-----------|-----------|----------|---------|
	 * | 内容 |机械角高8|机械角低8|实际电流高8|实际电流低8|给定电流高8|给定电流低8|霍尔开关值|  Null   |
   * -------------------------------------------------------------------------------------------------
	 */
	void HG900_6025_DataPro(CanRxMsg* rx_msg);

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
	void DBUS_DataReceive(void);
	
	
	/***********DBUS数据处理函数*************/
	void DBUS_DataPro(void);

	//得到Pit计算置位
	int16_t Get_Pit_Location(void);
	//得到Yaw计算置位
	int16_t Get_Yaw_Location(void);

private:
	
	bool Pit_first_read;	//Pit第一次读取
	bool Yaw_first_read;	//Yaw第一次读取

	vs16 Pit_first_data;	//Pit第一次数据作为偏移
	vs16 Yaw_first_data;	//Yaw第一次数据作为偏移

	vs16 Pit_data;				//PIT轴返回数据
	vs16 Yaw_data;				//YAW轴返回数据

	vs16 Pit_olddata;			//上一次的PIT数据
	vs16 Yaw_olddata;			//上一次的YAW数据
	
	vs16 Pit_location;		//PIT轴计算位置
	vs16 Yaw_location;		//YAW轴计算位置

	vs16 Pit_location_temp;//PIT轴位置中间变量
	vs16 Yaw_location_temp;//YAW轴位置中间变量

	vs16 Wheel_Rot;				//rotation 底盘旋转
	vs16 Wheel_L_R;				//left and right 左右平移
	vs16 Wheel_F_B;				//front and back 前后移动
	
};
extern DST_DataProcess _data_pro;


















#endif















