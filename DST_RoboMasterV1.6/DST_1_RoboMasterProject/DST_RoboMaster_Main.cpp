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
//p_kp_201-->0.05  aim_position_201:0~10000 �պ���1Ȧ
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
	int16_t Pit_cur_L, Yaw_cur_L;//PIT, YAW �ᵱǰλ��ֵ
	int16_t Pit_exp_value = 0;	//PIT ���������ֵ
	int16_t Yaw_exp_value = 0;	//YAW ���������ֵ
	uint16_t _i_led = 0;	//LED��˸����i����
	
	//����Ӳ���豸��ʼ��
	hardware.DST_HardwareInit();
		
	systime.Delay_ms(2000);
	can2.RoboModule_Driver_Reset(CAN_ID_DRIVER_RESET_ALL);   //��CAN���������е���������ʼ��
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
			//event._Event_DataReceive();		//���ݽ��մ���
			systime.sys_1ms = 0;
		}
		if(systime.sys_2ms >= 2)
		{
			mpu6050.Read_ACC_Data();
			mpu6050.Read_GYRO_Data();
			
			_data_pro.DBUS_DataPro();//DBUS ���ݴ������̵��RM35����
			
//			_time_test.PB9_OUT_SETUP();		//PB9���ڲ��Դ���ʱ��-->ʾ�����۲�
//			event._Event_IMU_ADCRead();		//�ߵ�ϵͳADC����
//			event._Event_IMU_ACC_MAG_Filter();//�ߵ�ϵͳ���ٶȴ������˲�
//			event._Event_IMU_GetEuler();	//�ߵ�ϵͳ��̬����õ�ŷ����
//			event._Event_ATT_ALT_INNER();	//��̬�߶��ڻ�����
//			_time_test.PB9_OUT_SETDOWN();	//PB9���ڲ��Դ���ʱ��-->ʾ�����۲�
			systime.sys_2ms = 0;
		}
		if(systime.sys_5ms >= 5)
		{
		
//			event._Event_ATT_OUTER();			//��̬�⻷����
			systime.sys_5ms = 0;
		}
		if(systime.sys_10ms >= 10)
		{
			Pit_exp_value = (dbus.RC_Ctl.rc_2000.ch1 - 1000)* 3;// 
			Yaw_exp_value = (dbus.RC_Ctl.rc_2000.ch0 - 1000)* 5;// 
//			event._Event_DataSend();			//���������ݷ���
//			event._Event_Baro_Update();		//��ѹ���ݲ���
//			event._Event_Mag_Update();		//�������ݲ���
			systime.sys_10ms = 0;
		}
		if(systime.sys_20ms >= 20)
		{
			current_201 = Position_Control_201(position_201 , aim_position_201);
//			can1.Send_CAN1_CMD(current_201,0,current_203);
			
			Pit_cur_L = _data_pro.Get_Pit_Location();
			Yaw_cur_L = _data_pro.Get_Yaw_Location();
			gimble_control.Location_Control(Pit_cur_L,Pit_exp_value,Yaw_cur_L,Yaw_exp_value);
		
//			event._Event_RemoteDataPro();	//ң�����ݴ���
//			
//			#ifdef LED_DEBUG_TEST					//LED�������Բ��Կ���
//				
//			#else
//				event._Event_LED_Pro();			//LEDָʾ�ƿ���
//			#endif
//			
//			event._Event_CommCheck();			//ͨ�Ŷ�ʧ���
			systime.sys_20ms = 0;
		}
		if(systime.sys_50ms >= 50)
		{

			

			
//			can1.Send_CAN1_CMD(4000,0,0);
//			can2.Send_CAN2_CMD(-4000,0,0);
//			event._Event_US_Motivate();		//������������100msһ��
//			event._Event_GetBatADC();			//���ADC����
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









//static u8  fac_us=0;//us��ʱ������			   
//static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

////��ʼ���ӳٺ���
////��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
////SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
////SYSCLK:ϵͳʱ��
//void delay_init(u8 SYSCLK)
//{
//#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
//	u32 reload;
//#endif
// 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//	fac_us=SYSCLK/8;		//�����Ƿ�ʹ��ucos,fac_us����Ҫʹ��
//	    
//#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
//	reload=SYSCLK/8;		//ÿ���ӵļ������� ��λΪK	   
//	reload*=1000000/OS_TICKS_PER_SEC;//����OS_TICKS_PER_SEC�趨���ʱ��
//							//reloadΪ24λ�Ĵ���,���ֵ:16777216,��168M��,Լ��0.7989s����	
//	fac_ms=1000/OS_TICKS_PER_SEC;//����ucos������ʱ�����ٵ�λ	   
//	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
//	SysTick->LOAD=reload; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK
//#else
//	fac_ms=(u16)fac_us*1000;//��ucos��,����ÿ��ms��Ҫ��systickʱ����   
//#endif
//}								    
////��ʱnus
////nusΪҪ��ʱ��us��.	
////ע��:nus��ֵ,��Ҫ����798915us
//void delay_us(u32 nus)
//{		
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
//	SysTick->VAL=0x00;        //��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ���� 
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
//	SysTick->VAL =0X00;       //��ռ�����	 
//}
////��ʱnms
////ע��nms�ķ�Χ
////SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
////nms<=0xffffff*8*1000/SYSCLK
////SYSCLK��λΪHz,nms��λΪms
////��168M������,nms<=798ms 
//void delay_xms(u16 nms)
//{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
//	SysTick->VAL =0x00;           //��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
//	SysTick->VAL =0X00;       //��ռ�����	  	    
//} 
////��ʱnms 
////nms:0~65535
//void delay_ms(u16 nms)
//{	 	 
//	u8 repeat=nms/540;	//������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,
//						//���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
//	u16 remain=nms%540;
//	while(repeat)
//	{
//		delay_xms(540);
//		repeat--;
//	}
//	if(remain)delay_xms(remain);
//	
//} 




