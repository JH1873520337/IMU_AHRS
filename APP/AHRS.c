#include "AHRS.h"
#include <math.h>
#include <stdint.h>

static float invSqrt(float x)
{
    union {
        float f;
        uint32_t i;
    } conv;

    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759dfU - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

static float constrain_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float accel_weight(float acc_norm)
{
    float err = fabsf(acc_norm - 1.0f);

    if (acc_norm >= ACC_NORM_MIN && acc_norm <= ACC_NORM_MAX)
    {
        return 1.0f;
    }

    if (err >= 0.25f)
    {
        return 0.0f;
    }

    return 1.0f - (err - ACC_STILL_TOL) / (0.25f - ACC_STILL_TOL);
}

static uint8_t is_stationary(float gx, float gy, float gz, float acc_norm)
{
    float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    if (fabsf(acc_norm - 1.0f) < ACC_STILL_TOL && gyro_norm < GYRO_STILL_THRESHOLD)
    {
        return 1;
    }

    return 0;
}

static void update_gain_stage(AHRS_State_t *state, float dt)
{
    if (!state->isConverged)
    {
        state->convergeTimer += dt;
        if (state->convergeTimer >= FAST_CONVERGE_TIME)
        {
            state->isConverged = 1;
            state->Kp = MAHONY_KP_DEFAULT;
        }
    }
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

    state->Kp = MAHONY_KP_FAST;
    state->Ki = MAHONY_KI_DEFAULT;
    state->convergeTimer = 0.0f;
    state->isConverged = 0;
    state->stillCounter = 0;
}

void AHRS_ResetIntegral(AHRS_State_t *state)
{
    state->integralFBx = 0.0f;
    state->integralFBy = 0.0f;
    state->integralFBz = 0.0f;
}

static void AHRS_Update6(AHRS_State_t *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc;
    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    float kp_use;
    float ki_use;
    float weight;
    uint8_t stationary;

    if (dt <= 0.0f)
    {
        return;
    }

    dt = constrain_float(dt, 0.0005f, 0.02f);
    update_gain_stage(state, dt);

    weight = accel_weight(accNorm);
    stationary = is_stationary(gx, gy, gz, accNorm);

    if (stationary)
    {
        if (state->stillCounter < 60000U)
        {
            state->stillCounter++;
        }
    }
    else
    {
        state->stillCounter = 0;
    }

    kp_use = state->Kp * weight;
    ki_use = state->Ki * weight;

    if (state->stillCounter > 20U)
    {
        kp_use *= 1.4f;
        ki_use *= 2.0f;
    }

    if (accNorm > 1e-6f)
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        vx = 2.0f * (q2 * q4 - q1 * q3);
        vy = 2.0f * (q1 * q2 + q3 * q4);
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        if (ki_use > 0.0f)
        {
            state->integralFBx += ki_use * ex * dt;
            state->integralFBy += ki_use * ey * dt;
            state->integralFBz += ki_use * ez * dt;

            state->integralFBx = constrain_float(state->integralFBx, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBy = constrain_float(state->integralFBy, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            state->integralFBz = constrain_float(state->integralFBz, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        else
        {
            state->integralFBx *= 0.995f;
            state->integralFBy *= 0.995f;
            state->integralFBz *= 0.995f;
        }

        gx += kp_use * ex + state->integralFBx;
        gy += kp_use * ey + state->integralFBy;
        gz += kp_use * ez + state->integralFBz;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q1;
    qb = q2;
    qc = q3;

    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    state->q.q1 = q1 * recipNorm;
    state->q.q2 = q2 * recipNorm;
    state->q.q3 = q3 * recipNorm;
    state->q.q4 = q4 * recipNorm;
}

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
    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    float magNorm = sqrtf(mx * mx + my * my + mz * mz);
    float kp_use;
    float ki_use;
    float weight;
    uint8_t stationary;

    if (dt <= 0.0f)
    {
        return;
    }

    if (magNorm < 1e-6f)
    {
        AHRS_Update6(state, gx, gy, gz, ax, ay, az, dt);
        return;
    }

    dt = constrain_float(dt, 0.0005f, 0.02f);
    update_gain_stage(state, dt);

    weight = accel_weight(accNorm);
    stationary = is_stationary(gx, gy, gz, accNorm);

    if (stationary)
    {
        if (state->stillCounter < 60000U)
        {
            state->stillCounter++;
        }
    }
    else
    {
        state->stillCounter = 0;
    }

    kp_use = state->Kp * weight;
    ki_use = state->Ki * weight;

    if (state->stillCounter > 20U)
    {
        kp_use *= 1.4f;
        ki_use *= 2.0f;
    }

    if (accNorm > 1e-6f)
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
    }
    else
    {
        AHRS_Update6(state, gx, gy, gz, ax, ay, az, dt);
        return;
    }

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q1q4 = q1 * q4;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q2q4 = q2 * q4;
    q3q3 = q3 * q3;
    q3q4 = q3 * q4;
    q4q4 = q4 * q4;

    hx = 2.0f * (mx * (0.5f - q3q3 - q4q4) + my * (q2q3 - q1q4) + mz * (q2q4 + q1q3));
    hy = 2.0f * (mx * (q2q3 + q1q4) + my * (0.5f - q2q2 - q4q4) + mz * (q3q4 - q1q2));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q2q4 - q1q3) + my * (q3q4 + q1q2) + mz * (0.5f - q2q2 - q3q3));

    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;

    wx = bx * (0.5f - q3q3 - q4q4) + bz * (q2q4 - q1q3);
    wy = bx * (q2q3 - q1q4) + bz * (q1q2 + q3q4);
    wz = bx * (q1q3 + q2q4) + bz * (0.5f - q2q2 - q3q3);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ki_use > 0.0f)
    {
        state->integralFBx += ki_use * ex * dt;
        state->integralFBy += ki_use * ey * dt;
        state->integralFBz += ki_use * ez * dt;

        state->integralFBx = constrain_float(state->integralFBx, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        state->integralFBy = constrain_float(state->integralFBy, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        state->integralFBz = constrain_float(state->integralFBz, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }
    else
    {
        state->integralFBx *= 0.995f;
        state->integralFBy *= 0.995f;
        state->integralFBz *= 0.995f;
    }

    gx += kp_use * ex + state->integralFBx;
    gy += kp_use * ey + state->integralFBy;
    gz += kp_use * ez + state->integralFBz;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q1;
    qb = q2;
    qc = q3;

    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
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

    state->roll = atan2f(2.0f * (q1 * q2 + q3 * q4), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    {
        float sinp = 2.0f * (q1 * q3 - q4 * q2);
        if (fabsf(sinp) >= 1.0f)
        {
            state->pitch = copysignf(1.5707963f, sinp);
        }
        else
        {
            state->pitch = asinf(sinp);
        }
    }

    state->yaw = atan2f(2.0f * (q1 * q4 + q2 * q3), 1.0f - 2.0f * (q3 * q3 + q4 * q4));
}

Quaternion AHRS_GetQuaternion(const AHRS_State_t *state)
{
    return state->q;
}
