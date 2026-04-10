//
// Created by ASUS on 26-2-2.
//

#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>
#include "quaternion.h"
#include "ahrs_mw.h"

#define MAHONY_KP_DEFAULT      2.8f
#define MAHONY_KI_DEFAULT      0.06f
#define MAHONY_KP_FAST         8.0f
#define MAHONY_KP_MAG_DEFAULT  3.6f
#define MAHONY_KI_MAG_DEFAULT  0.05f
#define FAST_CONVERGE_TIME     1.5f

#define INTEGRAL_LIMIT         0.35f

#define ACC_NORM_MIN           0.80f
#define ACC_NORM_MAX           1.20f
#define ACC_STILL_TOL          0.05f
#define GYRO_STILL_THRESHOLD   0.03f

#define MAG_FIELD_TOL_RATIO    0.15f
#define MAG_FIELD_REJECT_RATIO 0.35f
#define MAG_INNOV_TOL          0.10f
#define MAG_INNOV_REJECT       0.35f
#define MAG_USE_MIN_WEIGHT     0.10f
#define MAG_REF_ALPHA_MOVE     0.002f
#define MAG_REF_ALPHA_STILL    0.010f

// ============== AHRS 状态结构体 ==============
typedef struct {
    Quaternion q;

    float integralAccFBx;
    float integralAccFBy;
    float integralAccFBz;
    float integralMagFBx;
    float integralMagFBy;
    float integralMagFBz;

    float roll;
    float pitch;
    float yaw;

    float Kp;
    float Ki;
    float KpMag;
    float KiMag;
    float magFieldRef;

    float convergeTimer;
    uint8_t isConverged;
    uint8_t magFieldReady;
    uint16_t stillCounter;

} AHRS_State_t;

// ============== 函数声明 ==============

void AHRS_Init(AHRS_State_t *state);
void AHRS_Update(AHRS_State_t *state, const IMU_Data_t *imu, float dt);
void AHRS_GetEuler(AHRS_State_t *state);
Quaternion AHRS_GetQuaternion(const AHRS_State_t *state);

// 新增：重置积分项（在检测到静止时可调用）
void AHRS_ResetIntegral(AHRS_State_t *state);

#endif //AHRS_H
