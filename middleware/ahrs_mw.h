//
// Created by ASUS on 26-2-14.
//

#ifndef AHRS_MW_H
#define AHRS_MW_H

#include <stdint.h>

// ============== 传感器数据结构 ==============
typedef struct {
    // 加速度计数据 (单位: g)
    float ax;
    float ay;
    float az;

    // 陀螺仪数据 (单位: rad/s)
    float gx;
    float gy;
    float gz;

    // 磁力计数据 (单位: Gauss)
    float mx;
    float my;
    float mz;

    // 原始磁力计数据 (单位: Gauss)，用于标定采样
    float mx_raw;
    float my_raw;
    float mz_raw;

    // 数据有效标志位 (1=有效, 0=无效)
    uint8_t acc_valid;
    uint8_t gyro_valid;
    uint8_t mag_valid;

} IMU_Data_t;

// ============== 函数声明 ==============

/**
 * @brief 初始化所有传感器 (MPU6050 + QMC5883)
 * @note  函数内部包含【陀螺仪零偏校准】。
 *        !!! 上电后的前 1-2 秒请务必保持模块静止 !!!
 */
void AHRS_MW_Init(void);

/**
 * @brief 发起传感器数据 DMA 读取请求
 * @note  非阻塞，调用后需等待数据就绪
 */
void AHRS_MW_RequestData(void);

/**
 * @brief 检查数据是否全部就绪
 * @return 1=就绪, 0=未就绪
 */
uint8_t AHRS_MW_IsDataReady(void);

/**
 * @brief 获取传感器数据（解析 DMA 缓冲区并扣除零偏）
 * @param data 输出数据结构体指针
 * @return 1=成功获取, 0=数据未就绪
 */
uint8_t AHRS_MW_GetData(IMU_Data_t *data);

#endif //AHRS_MW_H
