#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"  // 根据实际MCU型号修改（如f4xx/h7xx等）
#include <stdint.h>

/* 需在主程序中定义I2C句柄并外部引用 */
extern I2C_HandleTypeDef hi2c2;
#define MPU6050_I2C_HANDLE    &hi2c2
//灵敏度
#define ACCEL_SENSITIVITY 16384.0f  // AFS_SEL=0（±2g）对应的灵敏度
#define GYRO_SENSITIVITY  131.0f    // FS_SEL=0（±250°/s）对应的灵敏度

/* MPU6050 I2C地址（7位地址0x68，左移1位+读写位） */
#define MPU6050_W_ADDRESS     0xD0  // 写地址
#define MPU6050_R_ADDRESS     0xD1  // 读地址

/* 寄存器地址（修正原代码0/O拼写错误） */
#define MPU6050_SMPLRT_DIV    0x19  // 采样率分频
#define MPU6050_CONFIG        0x1A  // 配置寄存器
#define MPU6050_GYRO_CONFIG   0x1B  // 陀螺仪配置
#define MPU6050_ACCEL_CONFIG  0x1C  // 加速度计配置

#define MPU6050_ACCEL_XOUT_H  0x3B  // 加速度X轴高字节
#define MPU6050_ACCEL_XOUT_L  0x3C  // 加速度X轴低字节
#define MPU6050_ACCEL_YOUT_H  0x3D  // 加速度Y轴高字节
#define MPU6050_ACCEL_YOUT_L  0x3E  // 加速度Y轴低字节
#define MPU6050_ACCEL_ZOUT_H  0x3F  // 加速度Z轴高字节
#define MPU6050_ACCEL_ZOUT_L  0x40  // 加速度Z轴低字节

#define MPU6050_TEMP_OUT_H    0x41  // 温度高字节
#define MPU6050_TEMP_OUT_L    0x42  // 温度低字节

#define MPU6050_GYRO_XOUT_H   0x43  // 陀螺仪X轴高字节
#define MPU6050_GYRO_XOUT_L   0x44  // 陀螺仪X轴低字节
#define MPU6050_GYRO_YOUT_H   0x45  // 陀螺仪Y轴高字节
#define MPU6050_GYRO_YOUT_L   0x46  // 陀螺仪Y轴低字节
#define MPU6050_GYRO_ZOUT_H   0x47  // 陀螺仪Z轴高字节
#define MPU6050_GYRO_ZOUT_L   0x48  // 陀螺仪Z轴低字节

#define MPU6050_PWR_MGMT_1    0x6B  // 电源管理1
#define MPU6050_PWR_MGMT_2    0x6C  // 电源管理2
#define MPU6050_WHO_AM_I      0x75  // 设备ID寄存器

/* 函数声明 */
HAL_StatusTypeDef MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size);
HAL_StatusTypeDef MPU6050_ReadByte(uint8_t RegAddress, uint8_t* Data);
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size);

void MPU6050_Init(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_GetRealData(float *ax_real, float *ay_real, float *az_real,
                         float *gx_real, float *gy_real, float *gz_real);
uint8_t MPU6050_GetID(void);

#endif // MPU6050_H