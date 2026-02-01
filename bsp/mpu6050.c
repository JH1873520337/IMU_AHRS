#include "MPU6050.h"
//写一个字节
void MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data)
{
	IIC_Start() ;
	IIC_SendByte (MPU6050_W_ADDRESS) ;
	IIC_ReceiveAck() ;
	IIC_SendByte (RegAddress) ;
	IIC_ReceiveAck();
	IIC_SendByte (Data) ;
	IIC_ReceiveAck();
	IIC_Stop() ;
}

//写多个字节
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t* Data,uint8_t Size)
{
	uint8_t xdata i = 0;
	IIC_Start() ;
	IIC_SendByte (MPU6050_W_ADDRESS) ;
	IIC_ReceiveAck() ;
	IIC_SendByte (RegAddress) ;
	IIC_ReceiveAck();
	for(i=0; i<Size;i++)
	{
	IIC_SendByte (Data[i]) ;
	IIC_ReceiveAck();
	}
	IIC_Stop() ;
}
//读一个字节
uint8_t MPU6050_ReadByte(uint8_t RegAddress)
{
	uint8_t xdata Data;
	IIC_Start();
	IIC_SendByte(MPU6050_W_ADDRESS) ;
	IIC_ReceiveAck();
	IIC_SendByte (RegAddress) ;
	IIC_ReceiveAck();
	IIC_Start();
	IIC_SendByte(MPU6050_R_ADDRESS) ;
	IIC_ReceiveAck();
  Data = IIC_ReceiveByte();
  IIC_SendAck(1);//不给从机应答			
	IIC_Stop();
	return Data;
}
//读多个字节
void MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data,uint8_t Size)
{
	uint8_t xdata i;
	IIC_Start();
	IIC_SendByte(MPU6050_W_ADDRESS) ;
	IIC_ReceiveAck();
	IIC_SendByte (RegAddress) ;
	IIC_ReceiveAck();
	IIC_Start();
	IIC_SendByte(MPU6050_R_ADDRESS) ;
	IIC_ReceiveAck();
	for(i=0;i<Size;i++)
	{
		if(i<Size-1)
		{
			Data[i] = IIC_ReceiveByte();
	    IIC_SendAck(0);//给从机应答
		}
		else 
		{
			Data[i] = IIC_ReceiveByte();
	    IIC_SendAck(1);//不给从机应答			
		}
	}
	IIC_Stop();	
}

void MPU6050_Init()
{
	IIC_GPIO_Init();
	MPU6050_WriteByte(MPU6050_PWR_MGMT_1,0X01);   //解除睡眠，选择陀螺仪时钟
	MPU6050_WriteByte(MPU6050_PWR_MGMT_2,0X00);
	MPU6050_WriteByte(MPU6050_SMPLRT_DIV ,0X09);  //10分频
	MPU6050_WriteByte(MPU6050_C0NFIG ,0X06);
	MPU6050_WriteByte(MPU6050_GYRO_CONFIG ,0X18);
	MPU6050_WriteByte(MPU6050_ACCEL_C0NFIG ,0X18);
}
//获取传感器6轴的原始数据
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
										 int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	 uint8_t xdata DataH,DataL;
	 DataH = MPU6050_ReadByte(MPU6050_ACCEL_X0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_ACCEL_X0UT_L);
	 *AccX = (DataH<<8)|DataL;
	
	 DataH = MPU6050_ReadByte(MPU6050_ACCEL_Y0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_ACCEL_Y0UT_L);
	 *AccY = (DataH<<8)|DataL;
	
	 DataH = MPU6050_ReadByte(MPU6050_ACCEL_Z0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_ACCEL_Z0UT_L);
	 *AccZ = (DataH<<8)|DataL;
	
	 DataH = MPU6050_ReadByte(MPU6050_GYR0_X0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_GYR0_X0UT_L);
	 *GyroX = (DataH<<8)|DataL;
	
	 DataH = MPU6050_ReadByte(MPU6050_GYR0_Y0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_GYR0_Y0UT_L);
	 *GyroY = (DataH<<8)|DataL;
	
	 DataH = MPU6050_ReadByte(MPU6050_GYR0_Z0UT_H);
	 DataL = MPU6050_ReadByte(MPU6050_GYR0_Z0UT_L);
	 *GyroZ = (DataH<<8)|DataL;
}
//获取MPU6050的设备ID
uint8_t MPU6050_GetID(void)
{
	 return  MPU6050_ReadByte(MPU6050_WH0_AM_I);
}