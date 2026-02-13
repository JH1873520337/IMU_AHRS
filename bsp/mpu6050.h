#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 根据硬件选择 I2C */
extern I2C_HandleTypeDef hi2c2;
#define MPU6050_I2C_HANDLE    &hi2c2
#define MPU6050_I2C_ADDR_BASE  0x68
#define MPU6050_I2C_ADDR_W     (MPU6050_I2C_ADDR_BASE << 1)
#define MPU6050_I2C_ADDR_R     (MPU6050_I2C_ADDR_BASE << 1 | 0x01)

/* MPU6050 寄存器地址 */
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C
#define MPU6050_WHO_AM_I      0x75

/* 灵敏度系数 */
#define ACCEL_SENSITIVITY     16384.0f
#define GYRO_SENSITIVITY      131.0f

/* DMA 标志位 */
extern volatile uint8_t mpu6050_i2c_rx_done;

/* 函数声明 */
void MPU6050_Init(void); // 阻塞式初始化
uint8_t MPU6050_GetID(void);

// === 核心数据读取函数 (DMA 非阻塞) ===
// 1. 发起读取请求 (在主循环开始调用)
void MPU6050_RequestData(void);
// 2. 检查数据是否就绪 (检查 mpu6050_i2c_rx_done)
// 3. 解析数据 (在主循环中，当 rx_done==1 时调用)
void MPU6050_ParseData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#endif
