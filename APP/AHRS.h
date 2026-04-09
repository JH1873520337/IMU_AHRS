//
// Created by ASUS on 26-2-2.
//

#ifndef AHRS_H
#define AHRS_H

#include "quaternion.h"
#include "ahrs_mw.h"

// ============== 优化后的算法参数 ==============
#define MAHONY_KP_DEFAULT   5.0f      // 提高比例增益，改善动态响应
#define MAHONY_KI_DEFAULT   0.005f    // 降低积分增益，减少零漂累积

// 积分限幅（防止积分饱和导致零漂）
#define INTEGRAL_LIMIT      0.5f

// 加速度计信任阈值（动态检测）
#define ACC_NORM_MIN        0.85f     // 最小加速度模值
#define ACC_NORM_MAX        1.15f     // 最大加速度模值

// 快速收敛阶段参数
#define FAST_CONVERGE_TIME  2.0f      // 快速收敛时间(秒)
#define MAHONY_KP_FAST      15.0f     // 快速收敛阶段的Kp

// ============== AHRS 状态结构体 ==============
typedef struct {
    Quaternion q;

    float integralFBx;
    float integralFBy;
    float integralFBz;

    float roll;
    float pitch;
    float yaw;

    // 动态Kp支持
    float Kp;
    float Ki;

    // 收敛计时器
    float convergeTimer;
    uint8_t isConverged;

} AHRS_State_t;

// ============== 函数声明 ==============

void AHRS_Init(AHRS_State_t *state);
void AHRS_Update(AHRS_State_t *state, const IMU_Data_t *imu, float dt);
void AHRS_GetEuler(AHRS_State_t *state);
Quaternion AHRS_GetQuaternion(const AHRS_State_t *state);

// 新增：重置积分项（在检测到静止时可调用）
void AHRS_ResetIntegral(AHRS_State_t *state);

#endif //AHRS_H
