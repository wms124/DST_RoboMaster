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

int aim_position_201 = 0;
int position_201 = 0;
int oldPosition_201 = 0;
int oldPosition_203 = 0;
int position_203 = 0;
int aim_position_203 = 0;
int loop_201 = 0;
int loop_203 = 0;
int pp_201 = 0;
int ppold_201 = 0;
int pp_203 = 0;
int ppold_203 = 0;
u8  tick_201 = 0;
u8  tick_203 = 0;
short current_201 = 0;
short current_201_old = 0;
short current_203 = 0;
short current_203_old = 0;
float p_error_201[3]  = {0,0,0};
float p_error_203[3]  = {0,0,0};
float p_wss_201 = 0;
float p_wss_203 = 0;
//p_kp_201-->0.05  aim_position_201:0~10000 刚好是1圈
float p_kp_201 = 0.05;//0.3  
float p_ki_201 = 0.008;
float p_kp_203 = 0.3;
float p_ki_203 = 0.008;

#define ESC_MAX 5000.0f
#define abs(x) ((x)>0? (x):(-(x)))

float Position_Control_201(float current_position_201,float target_position_201)
{
	
	p_error_201[0] = target_position_201 - current_position_201;
	if(abs(p_error_201[0]) > 10)
	{
		p_wss_201 += p_kp_201 * (p_error_201[0] - p_error_201[1]) +  p_ki_201 * (p_error_201[2] - 2* p_error_201[1] + p_error_201[0]);
	}
	p_error_201[2] = p_error_201[1]; 
	p_error_201[1] = p_error_201[0];
	if(p_wss_201 > ESC_MAX)
  {
      p_wss_201 = ESC_MAX;
  }
    
  if(p_wss_201 < -ESC_MAX)
  {
      p_wss_201 = -ESC_MAX;
  }
  return p_wss_201;
}

void ws(CanRxMsg* rx_message)
{
		if(rx_message->StdId == 0x201)
		{
			tick_201++;
			if(tick_201 == 20)
			{
				tick_201 = 0;
			}
			
				
			pp_201 = (rx_message->Data[0]<<8)|(rx_message->Data[1]);
			
			if((pp_201 < 2000) && (ppold_201 > 5000))
			{
				loop_201++;
			}
			else
			if((pp_201 > 5000) && (ppold_201 < 2000))
			{
				loop_201--;
			}
			ppold_201 = pp_201;
			position_201 = loop_201 * 8191 + pp_201;
				
			if(tick_201 == 0)
			{
					oldPosition_201 = position_201;
			}
			
		}
//		else if(rx_message->StdId == 0x203)
//			{
//				tick_203++;
//				if(tick_203 == 20)
//				{
//					tick_203 = 0;
//				}
//				pp_203 = (rx_message->Data[0]<<8)|(rx_message->Data[1]);
//				if((pp_203 < 2000) && (ppold_203 > 5000))
//				{
//					loop_203++;
//				}
//				else
//					if((pp_203 > 5000) && (ppold_203 < 2000))
//					{
//						loop_203--;
//					}
//				ppold_203 = pp_203;
//				position_203 = loop_203 * 8191 + pp_203;
//				
//				if(tick_201 == 0)
//				{
//					current_203 = Position_Control_203(position_203 , aim_position_203);
//					oldPosition_203 = position_203;
//					Cmd_ESC(current_201,0,current_203);
//				}
//			}
}

//static uint8_t rx2_message[20];

int main(void)
{
	int16_t Pit_cur_L, Yaw_cur_L;//PIT, YAW 轴当前位置值
	int16_t Pit_exp_value = 0;	//PIT 轴控制期望值
	int16_t Yaw_exp_value = 0;	//YAW 轴控制期望值
	uint16_t _i_led = 0;	//LED闪烁控制i变量
	
	//所有硬件设备初始化
	hardware.DST_HardwareInit();
		
	systime.Delay_ms(2000);
	can2.RoboModule_Driver_Reset(CAN_ID_DRIVER_RESET_ALL);   //将CAN总线上所有的驱动器初始化
	systime.Delay_ms(2000);
	can2.RoboModule_Driver_Mode_Chioce(CAN_ID_MODE_CHOICE_ALL, ENTER_SPEED_MODE);
	systime.Delay_ms(2000);

	GPIO_ResetBits(GPIO_COLOURFUL_LED_D5, Pin_D7 | Pin_D9);		
	
	//GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_B);		
	GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_R);		
	GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_G);	
	
	GPIO_ResetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_B);		
	//GPIO_ResetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_R);		
	GPIO_ResetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_G);	

//	can1.Send_CAN1_CMD(4000,0,0);
//	can2.Send_CAN2_CMD(-4000,0,0);
	while(1)
	{
//			systime.Delay_ms(2000);
//			Pit_exp_value = 5000;
//			Pit_cur_L = _data_pro.Get_Pit_Location();
//			gimble_control.Location_Control(Pit_cur_L,Pit_exp_value,Yaw_L,5000);
//	
//			systime.Delay_ms(2000);
//			Pit_exp_value = -5000;
//			Pit_cur_L = _data_pro.Get_Pit_Location();
//			gimble_control.Location_Control(Pit_cur_L,Pit_exp_value,Yaw_L,5000);

		if(systime.sys_1ms >= 1)
		{
			//event._Event_DataReceive();		//数据接收处理
			systime.sys_1ms = 0;
		}
		if(systime.sys_2ms >= 2)
		{
			mpu6050.Read_ACC_Data();
			mpu6050.Read_GYRO_Data();
			
			_data_pro.DBUS_DataPro();//DBUS 数据处理，底盘电机RM35控制
			
//			_time_test.PB9_OUT_SETUP();		//PB9用于测试处理时间-->示波器观测
//			event._Event_IMU_ADCRead();		//惯导系统ADC采样
//			event._Event_IMU_ACC_MAG_Filter();//惯导系统加速度磁力计滤波
//			event._Event_IMU_GetEuler();	//惯导系统姿态解算得到欧拉角
//			event._Event_ATT_ALT_INNER();	//姿态高度内环控制
//			_time_test.PB9_OUT_SETDOWN();	//PB9用于测试处理时间-->示波器观测
			systime.sys_2ms = 0;
		}
		if(systime.sys_5ms >= 5)
		{
		
//			event._Event_ATT_OUTER();			//姿态外环控制
			systime.sys_5ms = 0;
		}
		if(systime.sys_10ms >= 10)
		{
			Pit_exp_value = (dbus.RC_Ctl.rc_2000.ch1 - 1000)* 3;// 
			Yaw_exp_value = (dbus.RC_Ctl.rc_2000.ch0 - 1000)* 5;// 
//			event._Event_DataSend();			//飞行器数据发送
//			event._Event_Baro_Update();		//气压数据采样
//			event._Event_Mag_Update();		//罗盘数据采样
			systime.sys_10ms = 0;
		}
		if(systime.sys_20ms >= 20)
		{
			current_201 = Position_Control_201(position_201 , aim_position_201);
//			can1.Send_CAN1_CMD(current_201,0,current_203);
			
			Pit_cur_L = _data_pro.Get_Pit_Location();
			Yaw_cur_L = _data_pro.Get_Yaw_Location();
			gimble_control.Location_Control(Pit_cur_L,Pit_exp_value,Yaw_cur_L,Yaw_exp_value);
		
//			event._Event_RemoteDataPro();	//遥控数据处理
//			
//			#ifdef LED_DEBUG_TEST					//LED用作调试测试开关
//				
//			#else
//				event._Event_LED_Pro();			//LED指示灯控制
//			#endif
//			
//			event._Event_CommCheck();			//通信丢失检查
			systime.sys_20ms = 0;
		}
		if(systime.sys_50ms >= 50)
		{

			

			
//			can1.Send_CAN1_CMD(4000,0,0);
//			can2.Send_CAN2_CMD(-4000,0,0);
//			event._Event_US_Motivate();		//超声波激发，100ms一次
//			event._Event_GetBatADC();			//电池ADC采样
			systime.sys_50ms = 0;
		}
		if(systime.sys_500ms >= 500)
		{
			_i_led++;
			if(_i_led==3)
			{
				//GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_B);		
				GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_R);		
				GPIO_ResetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_G);	
				
//				Pit_exp_value = 10000;
//				aim_position_201 = 10000;
				_i_led = 4;
			}
			if(_i_led==7)
			{
				GPIO_SetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_B);		
				GPIO_SetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_R);		
				GPIO_SetBits(GPIO_COLOURFUL_LED_D4, Pin_COLOURFUL_LED_D4_G);	
				
//				Pit_exp_value = 0;
//				aim_position_201 = 0;
				_i_led = 0;
			}
			systime.sys_500ms = 0;
		}

	}
}









//static u8  fac_us=0;//us延时倍乘数			   
//static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数

////初始化延迟函数
////当使用ucos的时候,此函数会初始化ucos的时钟节拍
////SYSTICK的时钟固定为HCLK时钟的1/8
////SYSCLK:系统时钟
//void delay_init(u8 SYSCLK)
//{
//#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
//	u32 reload;
//#endif
// 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//	fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
//	    
//#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
//	reload=SYSCLK/8;		//每秒钟的计数次数 单位为K	   
//	reload*=1000000/OS_TICKS_PER_SEC;//根据OS_TICKS_PER_SEC设定溢出时间
//							//reload为24位寄存器,最大值:16777216,在168M下,约合0.7989s左右	
//	fac_ms=1000/OS_TICKS_PER_SEC;//代表ucos可以延时的最少单位	   
//	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
//	SysTick->LOAD=reload; 	//每1/OS_TICKS_PER_SEC秒中断一次	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK
//#else
//	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
//#endif
//}								    
////延时nus
////nus为要延时的us数.	
////注意:nus的值,不要大于798915us
//void delay_us(u32 nus)
//{		
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; //时间加载	  		 
//	SysTick->VAL=0x00;        //清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;       //清空计数器	 
//}
////延时nms
////注意nms的范围
////SysTick->LOAD为24位寄存器,所以,最大延时为:
////nms<=0xffffff*8*1000/SYSCLK
////SYSCLK单位为Hz,nms单位为ms
////对168M条件下,nms<=798ms 
//void delay_xms(u16 nms)
//{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
//	SysTick->VAL =0x00;           //清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;       //清空计数器	  	    
//} 
////延时nms 
////nms:0~65535
//void delay_ms(u16 nms)
//{	 	 
//	u8 repeat=nms/540;	//这里用540,是考虑到某些客户可能超频使用,
//						//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
//	u16 remain=nms%540;
//	while(repeat)
//	{
//		delay_xms(540);
//		repeat--;
//	}
//	if(remain)delay_xms(remain);
//	
//} 




