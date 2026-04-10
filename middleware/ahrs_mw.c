#include "ahrs_mw.h"
#include "mpu6050.h"
#include "QMC5883.h"
#include "main.h"
#include <math.h>

static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static float acc_offset[3] = {0.0f, 0.0f, 0.0f};
// 将椭球标定结果填到这里：m_cal = S * (m_raw - b)
static const float mag_hardiron_bias[3] = {0.0f, 0.0f, 0.0f};
static const float mag_softiron_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static uint16_t still_counter = 0;
static uint8_t mag_request_div = 0;

#define STARTUP_CALIB_SAMPLES      1200
#define GYRO_RUNTIME_BIAS_ALPHA    0.0025f
#define GYRO_STILL_RAD_S           0.03f
#define ACC_STILL_G_TOL            0.05f
#define GYRO_DEADBAND_RAD_S        0.0015f
#define MAG_NORM_MIN               0.01f
#define MAG_NORM_MAX               2.0f

static void Apply_Mag_Calibration(float raw_mx, float raw_my, float raw_mz,
                                  float *mx, float *my, float *mz)
{
    float bx = raw_mx - mag_hardiron_bias[0];
    float by = raw_my - mag_hardiron_bias[1];
    float bz = raw_mz - mag_hardiron_bias[2];

    *mx = mag_softiron_matrix[0][0] * bx + mag_softiron_matrix[0][1] * by + mag_softiron_matrix[0][2] * bz;
    *my = mag_softiron_matrix[1][0] * bx + mag_softiron_matrix[1][1] * by + mag_softiron_matrix[1][2] * bz;
    *mz = mag_softiron_matrix[2][0] * bx + mag_softiron_matrix[2][1] * by + mag_softiron_matrix[2][2] * bz;
}

static uint8_t Wait_For_MPU_Ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!mpu6050_i2c_rx_done)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0;
        }
    }

    return 1;
}

void AHRS_MW_Init(void)
{
    MPU6050_Init();
    QMC5883_Init();
    HAL_Delay(300);

    double gx_sum = 0.0;
    double gy_sum = 0.0;
    double gz_sum = 0.0;
    double ax_sum = 0.0;
    double ay_sum = 0.0;
    double az_sum = 0.0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < STARTUP_CALIB_SAMPLES; i++)
    {
        float gx, gy, gz, ax, ay, az;

        MPU6050_RequestData();
        if (!Wait_For_MPU_Ready(20U))
        {
            continue;
        }

        MPU6050_ParseData(&ax, &ay, &az, &gx, &gy, &gz);
        mpu6050_i2c_rx_done = 0;

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        valid_samples++;

        HAL_Delay(1);
    }

    if (valid_samples == 0U)
    {
        return;
    }

    gyro_offset[0] = (float)(gx_sum / valid_samples);
    gyro_offset[1] = (float)(gy_sum / valid_samples);
    gyro_offset[2] = (float)(gz_sum / valid_samples);

    acc_offset[0] = (float)(ax_sum / valid_samples);
    acc_offset[1] = (float)(ay_sum / valid_samples);

    {
        float az_mean = (float)(az_sum / valid_samples);
        float az_ref = (az_mean >= 0.0f) ? 1.0f : -1.0f;
        acc_offset[2] = az_mean - az_ref;
    }
}

void AHRS_MW_RequestData(void)
{
    if (HAL_I2C_GetState(MPU6050_I2C_HANDLE) == HAL_I2C_STATE_READY)
    {
        MPU6050_RequestData();
    }

    mag_request_div++;
    if (mag_request_div >= 5U)
    {
        mag_request_div = 0;

        if (HAL_I2C_GetState(QMC5883_I2C_HANDLE) == HAL_I2C_STATE_READY)
        {
            QMC5883_RequestData();
        }
    }
}

uint8_t AHRS_MW_IsDataReady(void)
{
    return mpu6050_i2c_rx_done;
}

uint8_t AHRS_MW_GetData(IMU_Data_t *data)
{
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;

    if (!mpu6050_i2c_rx_done)
    {
        return 0;
    }

    data->acc_valid = 0;
    data->gyro_valid = 0;
    data->mag_valid = 0;
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;
    data->mx_raw = 0.0f;
    data->my_raw = 0.0f;
    data->mz_raw = 0.0f;

    MPU6050_ParseData(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    mpu6050_i2c_rx_done = 0;

    {
        float gx_unbiased = raw_gx - gyro_offset[0];
        float gy_unbiased = raw_gy - gyro_offset[1];
        float gz_unbiased = raw_gz - gyro_offset[2];

        float ax = raw_ax - acc_offset[0];
        float ay = raw_ay - acc_offset[1];
        float az = raw_az - acc_offset[2];

        float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
        float gyro_norm = sqrtf(gx_unbiased * gx_unbiased + gy_unbiased * gy_unbiased + gz_unbiased * gz_unbiased);

        uint8_t is_still = (fabsf(acc_norm - 1.0f) < ACC_STILL_G_TOL) && (gyro_norm < GYRO_STILL_RAD_S);

        if (is_still)
        {
            if (still_counter < 60000U)
            {
                still_counter++;
            }

            if (still_counter > 25U)
            {
                gyro_offset[0] = (1.0f - GYRO_RUNTIME_BIAS_ALPHA) * gyro_offset[0] + GYRO_RUNTIME_BIAS_ALPHA * raw_gx;
                gyro_offset[1] = (1.0f - GYRO_RUNTIME_BIAS_ALPHA) * gyro_offset[1] + GYRO_RUNTIME_BIAS_ALPHA * raw_gy;
                gyro_offset[2] = (1.0f - GYRO_RUNTIME_BIAS_ALPHA) * gyro_offset[2] + GYRO_RUNTIME_BIAS_ALPHA * raw_gz;

                gx_unbiased = raw_gx - gyro_offset[0];
                gy_unbiased = raw_gy - gyro_offset[1];
                gz_unbiased = raw_gz - gyro_offset[2];
            }
        }
        else
        {
            still_counter = 0;
        }

        if (fabsf(gx_unbiased) < GYRO_DEADBAND_RAD_S) gx_unbiased = 0.0f;
        if (fabsf(gy_unbiased) < GYRO_DEADBAND_RAD_S) gy_unbiased = 0.0f;
        if (fabsf(gz_unbiased) < GYRO_DEADBAND_RAD_S) gz_unbiased = 0.0f;

        data->ax = ax;
        data->ay = ay;
        data->az = az;
        data->gx = gx_unbiased;
        data->gy = gy_unbiased;
        data->gz = gz_unbiased;
        data->acc_valid = 1;
        data->gyro_valid = 1;
    }

    {
        float raw_mx, raw_my, raw_mz;

        if (QMC5883_ParseData(&raw_mx, &raw_my, &raw_mz))
        {
            float mx;
            float my;
            float mz;
            float mag_norm;

            data->mx_raw = raw_mx;
            data->my_raw = raw_my;
            data->mz_raw = raw_mz;

            Apply_Mag_Calibration(raw_mx, raw_my, raw_mz, &mx, &my, &mz);
            mag_norm = sqrtf(mx * mx + my * my + mz * mz);

            if ((mag_norm > MAG_NORM_MIN) && (mag_norm < MAG_NORM_MAX))
            {
                data->mx = mx;
                data->my = my;
                data->mz = mz;
                data->mag_valid = 1;
            }
        }
    }

    return 1;
}
