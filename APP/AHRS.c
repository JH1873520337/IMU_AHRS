//
// Created by ASUS on 26-2-2.
//

#include "AHRS.h"
#include <math.h>

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void AHRS_Init(AHRS_State_t *state)
{
    state->q.q1 = 1.0f;
    state->q.q2 = 0.0f;
    state->q.q3 = 0.0f;
    state->q.q4 = 0.0f;

    state->integralFBx = 0.0f;
    state->integralFBy = 0.0f;
    state->integralFBz = 0.0f;

    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;
}

// 内部6轴更新
static void AHRS_Update6(AHRS_State_t *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q2*q4 - q1*q3;
        halfvy = q1*q2 + q3*q4;
        halfvz = q1*q1 - 0.5f + q4*q4;

        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        if (MAHONY_KI > 0.0f)
        {
            state->integralFBx += MAHONY_KI * halfex * dt;
            state->integralFBy += MAHONY_KI * halfey * dt;
            state->integralFBz += MAHONY_KI * halfez * dt;
            gx += state->integralFBx;
            gy += state->integralFBy;
            gz += state->integralFBz;
        }

        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q1; qb = q2; qc = q3;

    q1 += (-qb*gx - qc*gy - q4*gz);
    q2 += (qa*gx + qc*gz - q4*gy);
    q3 += (qa*gy - qb*gz + q4*gx);
    q4 += (qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    state->q.q1 = q1 * recipNorm;
    state->q.q2 = q2 * recipNorm;
    state->q.q3 = q3 * recipNorm;
    state->q.q4 = q4 * recipNorm;
}

// 内部9轴更新
static void AHRS_Update9(AHRS_State_t *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt)
{
    float recipNorm;
    float q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        AHRS_Update6(state, gx, gy, gz, ax, ay, az, dt);
        return;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3; q1q4 = q1*q4;
        q2q2 = q2*q2; q2q3 = q2*q3; q2q4 = q2*q4;
        q3q3 = q3*q3; q3q4 = q3*q4; q4q4 = q4*q4;

        hx = 2.0f*(mx*(0.5f - q3q3 - q4q4) + my*(q2q3 - q1q4) + mz*(q2q4 + q1q3));
        hy = 2.0f*(mx*(q2q3 + q1q4) + my*(0.5f - q2q2 - q4q4) + mz*(q3q4 - q1q2));
        bx = sqrtf(hx*hx + hy*hy);
        bz = 2.0f*(mx*(q2q4 - q1q3) + my*(q3q4 + q1q2) + mz*(0.5f - q2q2 - q3q3));

        halfvx = q2q4 - q1q3;
        halfvy = q1q2 + q3q4;
        halfvz = q1q1 - 0.5f + q4q4;

        halfwx = bx*(0.5f - q3q3 - q4q4) + bz*(q2q4 - q1q3);
        halfwy = bx*(q2q3 - q1q4) + bz*(q1q2 + q3q4);
        halfwz = bx*(q1q3 + q2q4) + bz*(0.5f - q2q2 - q3q3);

        halfex = (ay*halfvz - az*halfvy) + (my*halfwz - mz*halfwy);
        halfey = (az*halfvx - ax*halfvz) + (mz*halfwx - mx*halfwz);
        halfez = (ax*halfvy - ay*halfvx) + (mx*halfwy - my*halfwx);

        if (MAHONY_KI > 0.0f)
        {
            state->integralFBx += MAHONY_KI * halfex * dt;
            state->integralFBy += MAHONY_KI * halfey * dt;
            state->integralFBz += MAHONY_KI * halfez * dt;
            gx += state->integralFBx;
            gy += state->integralFBy;
            gz += state->integralFBz;
        }

        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;
    }

    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    qa = q1; qb = q2; qc = q3;

    q1 += (-qb*gx - qc*gy - q4*gz);
    q2 += (qa*gx + qc*gz - q4*gy);
    q3 += (qa*gy - qb*gz + q4*gx);
    q4 += (qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    state->q.q1 = q1 * recipNorm;
    state->q.q2 = q2 * recipNorm;
    state->q.q3 = q3 * recipNorm;
    state->q.q4 = q4 * recipNorm;
}

// 对外统一接口
void AHRS_Update(AHRS_State_t *state, const IMU_Data_t *imu, float dt)
{
    if (imu->mag_valid)
    {
        AHRS_Update9(state,
                     imu->gx, imu->gy, imu->gz,
                     imu->ax, imu->ay, imu->az,
                     imu->mx, imu->my, imu->mz,
                     dt);
    }
    else
    {
        AHRS_Update6(state,
                     imu->gx, imu->gy, imu->gz,
                     imu->ax, imu->ay, imu->az,
                     dt);
    }
}

void AHRS_GetEuler(AHRS_State_t *state)
{
    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;

    state->roll = atan2f(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3));

    float sinp = 2.0f*(q1*q3 - q4*q2);
    if (fabsf(sinp) >= 1.0f)
        state->pitch = copysignf(3.14159265f/2.0f, sinp);
    else
        state->pitch = asinf(sinp);

    state->yaw = atan2f(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

Quaternion AHRS_GetQuaternion(const AHRS_State_t *state)
{
    return state->q;
}
