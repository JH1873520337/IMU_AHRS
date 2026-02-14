//
// Created by ASUS on 26-2-14.
//

#ifndef AHRS_MW_H
#define AHRS_MW_H

#include <stdint.h>

// ============== 传感器数据结构 ==============
typedef struct {
    float ax, ay, az;   // 加速度 (g)
    float gx, gy, gz;   // 角速度 (rad/s)
    float mx, my, mz;   // 磁场 (Gauss)

    uint8_t acc_valid;  // 加速度数据有效标志
    uint8_t gyro_valid; // 陀螺仪数据有效标志
    uint8_t mag_valid;  // 磁力计数据有效标志
} IMU_Data_t;

// ============== 函数声明 ==============

/**
 * @brief 初始化所有传感器
 */
void AHRS_MW_Init(void);

/**
 * @brief 发起传感器数据DMA读取请求
 */
void AHRS_MW_RequestData(void);

/**
 * @brief 检查数据是否全部就绪
 * @return 1=就绪, 0=未就绪
 */
uint8_t AHRS_MW_IsDataReady(void);

/**
 * @brief 获取传感器数据（解析DMA缓冲区）
 * @param data 输出数据结构体指针
 * @return 1=成功, 0=数据未就绪
 */
uint8_t AHRS_MW_GetData(IMU_Data_t *data);

#endif //AHRS_MW_H
