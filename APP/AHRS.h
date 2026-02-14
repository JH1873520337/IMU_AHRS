//
// Created by ASUS on 26-2-2.
//

#ifndef AHRS_H
#define AHRS_H

#include "quaternion.h"
#include "ahrs_mw.h"

// ============== 算法参数 ==============
#define MAHONY_KP       1.0f
#define MAHONY_KI       0.0f

// ============== AHRS 状态结构体 ==============
typedef struct {
    Quaternion q;

    float integralFBx;
    float integralFBy;
    float integralFBz;

    float roll;
    float pitch;
    float yaw;
} AHRS_State_t;

// ============== 函数声明 ==============

void AHRS_Init(AHRS_State_t *state);

void AHRS_Update(AHRS_State_t *state, const IMU_Data_t *imu, float dt);

void AHRS_GetEuler(AHRS_State_t *state);

Quaternion AHRS_GetQuaternion(const AHRS_State_t *state);

#endif //AHRS_H
