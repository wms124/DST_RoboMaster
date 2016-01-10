/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_MPU6050.h"



// MPU6050, 硬件I2c地址 0x68，模拟i2c地址0xD0   AD0高电平时地址为0x69 模拟IIC地址0xD2
#define MPU6050_ADDRESS         0x69		//0xD2	 

#define DMP_MEM_START_ADDR 			0x6E
#define DMP_MEM_R_W 						0x6F

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

#define MPU6050_LPF_256HZ       0
#define MPU6050_LPF_188HZ       1
#define MPU6050_LPF_98HZ        2
#define MPU6050_LPF_42HZ        3
#define MPU6050_LPF_20HZ        4
#define MPU6050_LPF_10HZ        5
#define MPU6050_LPF_5HZ         6

#define MPU6050A_2mg                ((float)0.00006103f)  // 0.00006250 g/LSB
#define MPU6050A_4mg                ((float)0.00012207f)  // 0.00012500 g/LSB
#define MPU6050A_8mg                ((float)0.00024414f)  // 0.00025000 g/LSB

#define MPU6050G_s250dps            ((float)0.0076335f)  // 0.0087500 dps/LSB
#define MPU6050G_s500dps            ((float)0.0152671f)  // 0.0175000 dps/LSB
#define MPU6050G_s2000dps           ((float)0.0609756f)  // 0.0700000 dps/LSB


DST_MPU6050 mpu6050;

DST_MPU6050::DST_MPU6050()
{
	Calibrated_ACC = 0;
	Calibrated_GYRO = 0;
}

void DST_MPU6050::MPU6050_Pin_Config()
{
//	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//	GPIO_InitStructure.GPIO_Pin   = Pin_6050_INT ;
//	GPIO_Init(GPIO_6050_INT, &GPIO_InitStructure);
//	
//	GPIO_SetBits(GPIO_6050_INT, Pin_6050_INT);	
}


void DST_MPU6050::MPU6050_Init(uint16_t lpf)
{
	uint8_t default_filter;
	
	MPU6050_Pin_Config();
	
	switch (lpf)
	{
	case 5:
			default_filter = MPU6050_LPF_5HZ;
			break;
	case 10:
			default_filter = MPU6050_LPF_10HZ;
			break;
	case 20:
			default_filter = MPU6050_LPF_20HZ;
			break;
	case 42:
			default_filter = MPU6050_LPF_42HZ;
			break;
	case 98:
			default_filter = MPU6050_LPF_98HZ;
			break;
	case 188:
			default_filter = MPU6050_LPF_188HZ;
			break;
	case 256:
			default_filter = MPU6050_LPF_256HZ;
			break;
	default:
			default_filter = MPU6050_LPF_98HZ;
			break;
	}
	
	//设备复位
	IIC_Write_1Byte(MPU6050_ADDRESS,MPU_RA_PWR_MGMT_1, 0x80);
	systime.Delay_ms(5);
	//陀螺仪采样率，0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
	IIC_Write_1Byte(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/1000 - 1));
	//设置设备时钟源，陀螺仪Z轴
	IIC_Write_1Byte(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);	
	//i2c旁路模式：CPU直接连接6050的附加IIC，用于连接磁力计HMC5883
	IIC_Write_1Byte(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0x02); 
	// INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	//低通滤波频率，0x03(42Hz)
	IIC_Write_1Byte(MPU6050_ADDRESS,MPU_RA_CONFIG, default_filter);	
	 //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	IIC_Write_1Byte(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18); 
	//加速计自检、测量范围(不自检，+-8G)			
	IIC_Write_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_CONFIG, 2 << 3);		
}

void DST_MPU6050::Read_ACC_Data(void)
{	
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_XOUT_L,&mpu6050_buffer[0]); 
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_XOUT_H,&mpu6050_buffer[1]);
	acc_temp[1] = -((((int16_t)mpu6050_buffer[1]) << 8) | mpu6050_buffer[0])- ACC_Offset.y;//;  //加速度X轴

	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_YOUT_L,&mpu6050_buffer[2]);
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_YOUT_H,&mpu6050_buffer[3]);
	acc_temp[0] = ((((int16_t)mpu6050_buffer[3]) << 8) | mpu6050_buffer[2])- ACC_Offset.x;//;  //加速度Y轴

	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_ZOUT_L,&mpu6050_buffer[4]);
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_ACCEL_ZOUT_H,&mpu6050_buffer[5]);
	acc_temp[2] = ((((int16_t)mpu6050_buffer[5]) << 8) | mpu6050_buffer[4])- ACC_Offset.z;//;  //加速度Z轴
	
	ACC_ADC((float)acc_temp[0],(float)acc_temp[1],(float)acc_temp[2]);
	
	Calibration_ACC();
}


void DST_MPU6050::Read_GYRO_Data(void)
{
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_XOUT_L,&mpu6050_buffer[6]); 
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_XOUT_H,&mpu6050_buffer[7]);
	gyro_temp[1] = -((((int16_t)mpu6050_buffer[7]) << 8) | mpu6050_buffer[6])- GYRO_Offset.y;//;	//陀螺仪X轴

	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_YOUT_L,&mpu6050_buffer[8]);
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_YOUT_H,&mpu6050_buffer[9]);
	gyro_temp[0] = ((((int16_t)mpu6050_buffer[9]) << 8) | mpu6050_buffer[8])- GYRO_Offset.x;//;	//陀螺仪Y轴

	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_ZOUT_L,&mpu6050_buffer[10]);
	IIC_Read_1Byte(MPU6050_ADDRESS,MPU_RA_GYRO_ZOUT_H,&mpu6050_buffer[11]);
	gyro_temp[2] = ((((int16_t)mpu6050_buffer[11]) << 8) | mpu6050_buffer[10])- GYRO_Offset.z;//;	  //陀螺仪Z轴		
	
	GYRO_ADC((float)gyro_temp[0], (float)gyro_temp[1], (float)gyro_temp[2]);

	Calibration_GYRO();	
}


Vector3 DST_MPU6050::Get_ACC_ADC(void)
{
	return ACC_ADC;
}


Vector3 DST_MPU6050::Get_GYRO_ADC(void)
{
	return GYRO_ADC;
}



Vector3 DST_MPU6050::Get_GYRO_in_dps(void)
{
	GYRO_dps.x = Q.radians(GYRO_ADC.x * MPU6050G_s2000dps);   // dps Q.
	GYRO_dps.y = Q.radians(GYRO_ADC.y * MPU6050G_s2000dps);   // dpsQ.
	GYRO_dps.z = Q.radians(GYRO_ADC.z * MPU6050G_s2000dps);   // dps	Q.r
	return GYRO_dps;
}

void DST_MPU6050::Calibration_ACC(void)
{
	if(Calibrated_ACC)
	{
		static Vector3	tempAcc;
		static uint16_t cnt_a=0;

		if(cnt_a==0)
		{
			ACC_Offset(0, 0, 0);
			tempAcc(0, 0, 0);
			cnt_a = 1;
			return;
		}			
		tempAcc += ACC_ADC;
		if(cnt_a == CALIBRATING_ACC_CYCLES)
		{
			ACC_Offset.x = tempAcc.x/cnt_a;
			ACC_Offset.y = tempAcc.y/cnt_a;
			ACC_Offset.z = tempAcc.z/cnt_a - ACC_1G;
			cnt_a = 0;
			Calibrated_ACC = 0;
			//发送加速度校准数据
			//d_t.Send_Offset_xyz(_offset_acc,ACC_Offset.x,ACC_Offset.y,ACC_Offset.z);
			//param.SAVE_ACC_OFFSET();//保存数据
//			EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], mpu6050.ACC_Offset.x);
//			EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], mpu6050.ACC_Offset.y);
//			EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], mpu6050.ACC_Offset.z);
			return;
		}
		cnt_a++;	
	}
}

void DST_MPU6050::Calibration_GYRO(void)
{
	if(Calibrated_GYRO)
	{
		static Vector3	tempGyro;
		static uint16_t cnt_g=0;
		if(cnt_g==0)
		{
			GYRO_Offset(0, 0, 0);
			tempGyro(0, 0, 0);
			cnt_g = 1;
			return;
		}
		tempGyro += GYRO_ADC;
		if(cnt_g == CALIBRATING_GYRO_CYCLES)
		{
			GYRO_Offset.x = tempGyro.x/cnt_g;
			GYRO_Offset.y = tempGyro.y/cnt_g;
			GYRO_Offset.z = tempGyro.z/cnt_g;
			cnt_g = 0;
			Calibrated_GYRO = 0;
			//发送陀螺仪校准数据
			//d_t.Send_Offset_xyz(_offset_gyro,GYRO_Offset.x,GYRO_Offset.y,GYRO_Offset.z);
			//param.SAVE_GYRO_OFFSET();//保存数据
//			EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], mpu6050.GYRO_Offset.x);
//			EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], mpu6050.GYRO_Offset.y);
//			EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], mpu6050.GYRO_Offset.z);
			return;
		}
		cnt_g++;
	}
}




