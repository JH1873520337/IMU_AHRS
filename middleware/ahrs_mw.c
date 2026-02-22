//
// Created by ASUS on 26-2-14.
//

#include "ahrs_mw.h"
#include "mpu6050.h"
#include "QMC5883.h"
#include "main.h"

// ================= 全局变量 =================

// 陀螺仪零偏
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
// 加速度计零偏 (新增！)
static float acc_offset[3] = {0.0f, 0.0f, 0.0f};
// 磁力计硬铁偏移
static float mag_offset[3] = {0.0f, 0.0f, 0.0f};

// ================= 私有函数 =================

static void Wait_For_Sensor_Ready(void)
{
    uint32_t timeout = 0;
    while (!mpu6050_i2c_rx_done)
    {
        HAL_Delay(1);
        if (++timeout > 50) return;
    }
}

// ================= 公共函数 =================

void AHRS_MW_Init(void)
{
    MPU6050_Init();
    QMC5883_Init();
    HAL_Delay(500); // 等待稳定

    // === 传感器双重校准 (Gyro + Acc) ===
    // 假设上电时，飞机是静止且水平放置的

    float gx, gy, gz, ax, ay, az;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int sample_count = 200;

    for (int i = 0; i < sample_count; i++)
    {
        MPU6050_RequestData();
        Wait_For_Sensor_Ready();
        MPU6050_ParseData(&ax, &ay, &az, &gx, &gy, &gz);

        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        ax_sum += ax; ay_sum += ay; az_sum += az;

        mpu6050_i2c_rx_done = 0;
        HAL_Delay(2);
    }

    // 1. 计算陀螺仪零偏
    gyro_offset[0] = (float)(gx_sum / sample_count);
    gyro_offset[1] = (float)(gy_sum / sample_count);
    gyro_offset[2] = (float)(gz_sum / sample_count);

    // 2. 计算加速度计零偏 (消除 30 度误差的核心)
    // 理论值：平放时 ax=0, ay=0, az=1 (或-1)
    // 实际值：ax=0.5 (这就导致了30度误差)
    // 偏移量 = 测量平均值 - 理论值
    acc_offset[0] = (float)(ax_sum / sample_count) - 0.0f; // X轴理论为0
    acc_offset[1] = (float)(ay_sum / sample_count) - 0.0f; // Y轴理论为0
    // Z轴通常不需要校准零偏，或者认为剩下的就是Z轴
    // acc_offset[2] 不建议校准，因为重力是标量，这里只做简单的姿态调平
    acc_offset[2] = 0.0f;
}

void AHRS_MW_RequestData(void)
{
    MPU6050_RequestData();
    QMC5883_RequestData();
}

uint8_t AHRS_MW_IsDataReady(void)
{
    return mpu6050_i2c_rx_done;
}

uint8_t AHRS_MW_GetData(IMU_Data_t *data)
{
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    float raw_mx, raw_my, raw_mz;

    // 1. 解析 MPU6050
    if (mpu6050_i2c_rx_done)
    {
        MPU6050_ParseData(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

        // 扣除陀螺仪零偏
        raw_gx -= gyro_offset[0];
        raw_gy -= gyro_offset[1];
        raw_gz -= gyro_offset[2];

        // 【核心】扣除加速度计零偏
        raw_ax -= acc_offset[0];
        raw_ay -= acc_offset[1];
        // raw_az -= acc_offset[2]; // Z轴通常不动

        mpu6050_i2c_rx_done = 0;
        data->acc_valid = 1;
        data->gyro_valid = 1;
    }
    else return 0;

    // 2. 解析 QMC5883
    if (QMC5883_ParseData(&raw_mx, &raw_my, &raw_mz))
    {
        raw_mx -= mag_offset[0];
        raw_my -= mag_offset[1];
        raw_mz -= mag_offset[2];
        data->mag_valid = 1;
    }

    // ============================================================
    //               坐标系对齐 (至关重要！！！)
    // ============================================================
    // 如果旋转时算法发散/乱飘，请在这里交换轴！
    // 必须满足右手定则。

    // 假设 MPU6050 平放，X机头，Y右，Z上
    data->ax = raw_ax;
    data->ay = raw_ay;
    data->az = raw_az;

    data->gx = raw_gx;
    data->gy = raw_gy;
    data->gz = raw_gz;

    // QMC5883 通常需要调整。如果 Yaw 越转越偏，尝试：
    // data->mx = raw_my; data->my = -raw_mx;
    data->mx = raw_mx;
    data->my = raw_my;
    data->mz = raw_mz;

    return 1;
}
