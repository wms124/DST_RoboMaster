#ifndef __DST_ROBOMASTERINCLUDE_H_
#define __DST_ROBOMASTERINCLUDE_H_
/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/





/*==================System Include==================*/
#include "stm32f4xx.h"
#include "DST_RoboMasterConfig.h"
#include "DST_HardwarePinConfig.h"
#include "DST_HardwareInit.h"



/*==================HardwareDrivers Include==================*/
#include "DST_SysTimer.h"
#include "DST_LED.h"
#include "DST_CAN_1.h"
#include "DST_CAN_2.h"
#include "DST_DBUS_USART2.h"
#include "DST_IIC_Soft.h"
#include "DST_MPU6050.h"


/*==================PID Control Include==================*/
#include "DST_GimbleControl.h"
#include "DST_PID.h"



/*==================Data Process Include==================*/
#include "DST_DataProcess.h"



/*==================Math Class Include==================*/
#include "DST_Vector3.h"
#include "DST_Math.h"
#include "DST_AmplitudeLimit.h"
#include "DST_Quaternion.h"


void ws(CanRxMsg* rx_message);





#endif

