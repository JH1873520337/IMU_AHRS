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

static float fade_weight(float err, float full_weight_err, float reject_err)
{
    if (err <= full_weight_err)
    {
        return 1.0f;
    }

    if (err >= reject_err)
    {
        return 0.0f;
    }

    return 1.0f - (err - full_weight_err) / (reject_err - full_weight_err);
}

static void constrain_integral_vector(float *x, float *y, float *z)
{
    *x = constrain_float(*x, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    *y = constrain_float(*y, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    *z = constrain_float(*z, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
}

static void decay_integral_vector(float *x, float *y, float *z, float factor)
{
    *x *= factor;
    *y *= factor;
    *z *= factor;
}

static float accel_weight(float acc_norm)
{
    float err = fabsf(acc_norm - 1.0f);

    if (acc_norm >= ACC_NORM_MIN && acc_norm <= ACC_NORM_MAX)
    {
        return 1.0f;
    }

    return fade_weight(err, ACC_STILL_TOL, 0.25f);
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

static float mag_norm_weight(AHRS_State_t *state, float mag_norm)
{
    if (mag_norm <= 1e-6f)
    {
        return 0.0f;
    }

    if (!state->magFieldReady)
    {
        state->magFieldRef = mag_norm;
        state->magFieldReady = 1U;
        return 1.0f;
    }

    if (state->magFieldRef <= 1e-6f)
    {
        return 0.0f;
    }

    return fade_weight(fabsf(mag_norm - state->magFieldRef) / state->magFieldRef,
                       MAG_FIELD_TOL_RATIO,
                       MAG_FIELD_REJECT_RATIO);
}

static float mag_innovation_weight(float ex, float ey, float ez)
{
    float innovation = sqrtf(ex * ex + ey * ey + ez * ez);
    return fade_weight(innovation, MAG_INNOV_TOL, MAG_INNOV_REJECT);
}

static void reset_mag_feedback(AHRS_State_t *state)
{
    state->integralMagFBx = 0.0f;
    state->integralMagFBy = 0.0f;
    state->integralMagFBz = 0.0f;
}

static void update_mag_reference(AHRS_State_t *state, float mag_norm, float mag_weight, uint8_t stationary)
{
    float alpha;

    if (mag_norm <= 1e-6f)
    {
        return;
    }

    if (!state->magFieldReady)
    {
        state->magFieldRef = mag_norm;
        state->magFieldReady = 1U;
        return;
    }

    if (mag_weight < 0.6f)
    {
        return;
    }

    alpha = stationary ? MAG_REF_ALPHA_STILL : MAG_REF_ALPHA_MOVE;
    state->magFieldRef += alpha * (mag_norm - state->magFieldRef);
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

    state->integralAccFBx = 0.0f;
    state->integralAccFBy = 0.0f;
    state->integralAccFBz = 0.0f;
    state->integralMagFBx = 0.0f;
    state->integralMagFBy = 0.0f;
    state->integralMagFBz = 0.0f;

    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;

    state->Kp = MAHONY_KP_FAST;
    state->Ki = MAHONY_KI_DEFAULT;
    state->KpMag = MAHONY_KP_MAG_DEFAULT;
    state->KiMag = MAHONY_KI_MAG_DEFAULT;
    state->magFieldRef = 0.0f;
    state->convergeTimer = 0.0f;
    state->isConverged = 0;
    state->magFieldReady = 0;
    state->stillCounter = 0;
}

void AHRS_ResetIntegral(AHRS_State_t *state)
{
    state->integralAccFBx = 0.0f;
    state->integralAccFBy = 0.0f;
    state->integralAccFBz = 0.0f;
    reset_mag_feedback(state);
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

        decay_integral_vector(&state->integralMagFBx, &state->integralMagFBy, &state->integralMagFBz, 0.98f);

        if (ki_use > 0.0f)
        {
            state->integralAccFBx += ki_use * ex * dt;
            state->integralAccFBy += ki_use * ey * dt;
            state->integralAccFBz += ki_use * ez * dt;
            constrain_integral_vector(&state->integralAccFBx, &state->integralAccFBy, &state->integralAccFBz);
        }
        else
        {
            decay_integral_vector(&state->integralAccFBx, &state->integralAccFBy, &state->integralAccFBz, 0.995f);
        }

        gx += kp_use * ex + state->integralAccFBx;
        gy += kp_use * ey + state->integralAccFBy;
        gz += kp_use * ez + state->integralAccFBz;
    }
    else
    {
        decay_integral_vector(&state->integralAccFBx, &state->integralAccFBy, &state->integralAccFBz, 0.995f);
        decay_integral_vector(&state->integralMagFBx, &state->integralMagFBy, &state->integralMagFBz, 0.98f);
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
    float ex_acc, ey_acc, ez_acc;
    float ex_mag, ey_mag, ez_mag;
    float qa, qb, qc;
    float q1 = state->q.q1;
    float q2 = state->q.q2;
    float q3 = state->q.q3;
    float q4 = state->q.q4;
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    float magNorm = sqrtf(mx * mx + my * my + mz * mz);
    float accWeight;
    float magWeight;
    float kp_acc;
    float ki_acc;
    float kp_mag;
    float ki_mag;
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

    accWeight = accel_weight(accNorm);
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

    ex_acc = (ay * vz - az * vy);
    ey_acc = (az * vx - ax * vz);
    ez_acc = (ax * vy - ay * vx);

    ex_mag = (my * wz - mz * wy);
    ey_mag = (mz * wx - mx * wz);
    ez_mag = (mx * wy - my * wx);

    magWeight = mag_norm_weight(state, magNorm) * mag_innovation_weight(ex_mag, ey_mag, ez_mag);

    if (magWeight < MAG_USE_MIN_WEIGHT)
    {
        reset_mag_feedback(state);
        AHRS_Update6(state, gx, gy, gz, ax, ay, az, dt);
        return;
    }

    kp_acc = state->Kp * accWeight;
    ki_acc = state->Ki * accWeight;
    kp_mag = state->KpMag * magWeight;
    ki_mag = state->KiMag * magWeight;

    if (state->stillCounter > 20U)
    {
        kp_acc *= 1.4f;
        ki_acc *= 2.0f;
        kp_mag *= 1.2f;
        ki_mag *= 1.5f;
    }

    if (ki_acc > 0.0f)
    {
        state->integralAccFBx += ki_acc * ex_acc * dt;
        state->integralAccFBy += ki_acc * ey_acc * dt;
        state->integralAccFBz += ki_acc * ez_acc * dt;
        constrain_integral_vector(&state->integralAccFBx, &state->integralAccFBy, &state->integralAccFBz);
    }
    else
    {
        decay_integral_vector(&state->integralAccFBx, &state->integralAccFBy, &state->integralAccFBz, 0.995f);
    }

    if (ki_mag > 0.0f)
    {
        state->integralMagFBx += ki_mag * ex_mag * dt;
        state->integralMagFBy += ki_mag * ey_mag * dt;
        state->integralMagFBz += ki_mag * ez_mag * dt;
        constrain_integral_vector(&state->integralMagFBx, &state->integralMagFBy, &state->integralMagFBz);
    }
    else
    {
        decay_integral_vector(&state->integralMagFBx, &state->integralMagFBy, &state->integralMagFBz, 0.98f);
    }

    gx += kp_acc * ex_acc + kp_mag * ex_mag + state->integralAccFBx + state->integralMagFBx;
    gy += kp_acc * ey_acc + kp_mag * ey_mag + state->integralAccFBy + state->integralMagFBy;
    gz += kp_acc * ez_acc + kp_mag * ez_mag + state->integralAccFBz + state->integralMagFBz;

    update_mag_reference(state, magNorm, magWeight, stationary);

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
