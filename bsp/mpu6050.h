//
// Created by ASUS on 26-2-1.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "i2c.h"

#define MPU6050_W_ADDRESS           0xD0
#define MPU6050_R_ADDRESS           0xD1

#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_C0NFIG              0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_C0NFIG		0x1c

#define MPU6050_ACCEL_X0UT_H		0x3B
#define MPU6050_ACCEL_X0UT_L		0x3c
#define MPU6050_ACCEL_Y0UT_H		0x3D
#define MPU6050_ACCEL_Y0UT_L		0x3E
#define MPU6050_ACCEL_Z0UT_H		0x3F
#define MPU6050_ACCEL_Z0UT_L		0x40
#define MPU6050_TEMP_0UT_H			0x41
#define MPU6050_TEMP_0UT_L			0x42
#define MPU6050_GYR0_X0UT_H			0x43
#define MPU6050_GYR0_X0UT_L			0x44
#define MPU6050_GYR0_Y0UT_H			0x45
#define MPU6050_GYR0_Y0UT_L			0x46
#define MPU6050_GYR0_Z0UT_H			0x47
#define MPU6050_GYR0_Z0UT_L         0x48
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6c
#define MPU6050_WH0_AM_I   			0x75

void MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadByte(uint8_t RegAddress);
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t* Data,uint8_t Size);
void MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data,uint8_t Size);
void MPU6050_Init();
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
                                         int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
uint8_t MPU6050_GetID(void);
#endif //MPU6050_H
