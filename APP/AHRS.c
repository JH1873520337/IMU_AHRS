//
// Created by ASUS on 26-2-2.
//

#include "AHRS.h"
#include <math.h>

// 快速平方根倒数
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y)); // 二次迭代提高精度
    return y;
}

// 限幅函数
static float constrain_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
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

    // 初始化为快速收敛模式
    state->Kp = MAHONY_KP_FAST;
    state->Ki = MAHONY_KI_DEFAULT;
    state->convergeTimer = 0.0f;
    state->isConverged = 0;
}

void AHRS_ResetIntegral(AHRS_State_t *state)
{
    state->integralFBx = 0.0f;
    state->integralFBy = 0.0f;
    state->integralFBz = 0.0f;
}

// 优化后的6轴更新
static void AHRS_Update6(AHRS_State_t *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recipNorm;
    float vx, vy, vz;        // 修正：使用完整的重力分量
    float ex, ey, ez;
    float qa, qb, qc;
    float accNorm;
    float dynamicKp;

    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;

    // ========== 快速收敛阶段管理 ==========
    if (!state->isConverged)
    {
        state->convergeTimer += dt;
        if (state->convergeTimer >= FAST_CONVERGE_TIME)
        {
            state->isConverged = 1;
            state->Kp = MAHONY_KP_DEFAULT;
        }
    }

    // ========== 加速度计有效性检测 ==========
    accNorm = sqrtf(ax*ax + ay*ay + az*az);

    // 动态调整Kp：当加速度偏离1g时，降低对加速度计的信任
    if (accNorm > ACC_NORM_MIN && accNorm < ACC_NORM_MAX)
    {
        // 正常范围内，使用正常Kp
        dynamicKp = state->Kp;
    }
    else if (accNorm > 0.5f && accNorm < 1.5f)
    {
        // 轻微异常，降低Kp
        dynamicKp = state->Kp * 0.3f;
    }
    else
    {
        // 严重异常（自由落体或剧烈碰撞），几乎不使用加速度计
        dynamicKp = state->Kp * 0.05f;
    }

    // ========== 加速度计校正 ==========
    if (accNorm > 0.1f)  // 避免除零
    {
        // 归一化加速度计
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 【修正】正确的重力参考矢量计算（从四元数提取）
        // 这是旋转矩阵第三列，表示机体坐标系下的重力方向
        vx = 2.0f * (q2*q4 - q1*q3);
        vy = 2.0f * (q1*q2 + q3*q4);
        vz = q1*q1 - q2*q2 - q3*q3 + q4*q4;

        // 叉积计算误差
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // ========== 积分项（带限幅）==========
        if (state->Ki > 0.0f)
        {
            state->integralFBx += state->Ki * ex * dt;
            state->integralFBy += state->Ki * ey * dt;
            state->integralFBz += state->Ki * ez * dt;

            // 【关键】积分限幅，防止零漂累积
            state->integralFBx = constrain_float(state->integralFBx, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBy = constrain_float(state->integralFBy, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBz = constrain_float(state->integralFBz, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

            gx += state->integralFBx;
            gy += state->integralFBy;
            gz += state->integralFBz;
        }

        // 比例项
        gx += dynamicKp * ex;
        gy += dynamicKp * ey;
        gz += dynamicKp * ez;
    }

    // ========== 四元数微分方程积分 ==========
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q1; qb = q2; qc = q3;

    q1 += (-qb*gx - qc*gy - q4*gz);
    q2 += (qa*gx + qc*gz - q4*gy);
    q3 += (qa*gy - qb*gz + q4*gx);
    q4 += (qa*gz + qb*gy - qc*gx);

    // 归一化四元数
    recipNorm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    state->q.q1 = q1 * recipNorm;
    state->q.q2 = q2 * recipNorm;
    state->q.q3 = q3 * recipNorm;
    state->q.q4 = q4 * recipNorm;
}

// 优化后的9轴更新
static void AHRS_Update9(AHRS_State_t *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt)
{
    float recipNorm;
    float q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float qa, qb, qc;
    float accNorm, magNorm;
    float dynamicKp;

    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;

    // 磁力计数据无效时退化为6轴
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        AHRS_Update6(state, gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // 快速收敛阶段管理
    if (!state->isConverged)
    {
        state->convergeTimer += dt;
        if (state->convergeTimer >= FAST_CONVERGE_TIME)
        {
            state->isConverged = 1;
            state->Kp = MAHONY_KP_DEFAULT;
        }
    }

    // 加速度计有效性检测
    accNorm = sqrtf(ax*ax + ay*ay + az*az);
    if (accNorm > ACC_NORM_MIN && accNorm < ACC_NORM_MAX)
        dynamicKp = state->Kp;
    else if (accNorm > 0.5f && accNorm < 1.5f)
        dynamicKp = state->Kp * 0.3f;
    else
        dynamicKp = state->Kp * 0.05f;

    if (accNorm > 0.1f)
    {
        // 归一化
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        magNorm = mx*mx + my*my + mz*mz;
        if (magNorm > 0.01f)
        {
            recipNorm = invSqrt(magNorm);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
        }

        // 辅助变量
        q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3; q1q4 = q1*q4;
        q2q2 = q2*q2; q2q3 = q2*q3; q2q4 = q2*q4;
        q3q3 = q3*q3; q3q4 = q3*q4; q4q4 = q4*q4;

        // 磁场参考方向
        hx = 2.0f*(mx*(0.5f - q3q3 - q4q4) + my*(q2q3 - q1q4) + mz*(q2q4 + q1q3));
        hy = 2.0f*(mx*(q2q3 + q1q4) + my*(0.5f - q2q2 - q4q4) + mz*(q3q4 - q1q2));
        bx = sqrtf(hx*hx + hy*hy);
        bz = 2.0f*(mx*(q2q4 - q1q3) + my*(q3q4 + q1q2) + mz*(0.5f - q2q2 - q3q3));

        // 【修正】重力参考矢量
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;

        // 磁场参考矢量
        wx = bx*(0.5f - q3q3 - q4q4) + bz*(q2q4 - q1q3);
        wy = bx*(q2q3 - q1q4) + bz*(q1q2 + q3q4);
        wz = bx*(q1q3 + q2q4) + bz*(0.5f - q2q2 - q3q3);

        // 误差计算
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        // 积分项（带限幅）
        if (state->Ki > 0.0f)
        {
            state->integralFBx += state->Ki * ex * dt;
            state->integralFBy += state->Ki * ey * dt;
            state->integralFBz += state->Ki * ez * dt;

            state->integralFBx = constrain_float(state->integralFBx, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBy = constrain_float(state->integralFBy, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBz = constrain_float(state->integralFBz, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

            gx += state->integralFBx;
            gy += state->integralFBy;
            gz += state->integralFBz;
        }

        gx += dynamicKp * ex;
        gy += dynamicKp * ey;
        gz += dynamicKp * ez;
    }

    // 四元数积分
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
        state->pitch = copysignf(1.5707963f, sinp);  // ±90°
    else
        state->pitch = asinf(sinp);

    state->yaw = atan2f(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

Quaternion AHRS_GetQuaternion(const AHRS_State_t *state)
{
    return state->q;
}
