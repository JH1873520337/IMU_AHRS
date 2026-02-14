//
// Created by ASUS on 26-2-14.
//

#include "ahrs_mw.h"
#include "mpu6050.h"
#include "QMC5883.h"

void AHRS_MW_Init(void)
{
    MPU6050_Init();
    QMC5883_Init();
}

void AHRS_MW_RequestData(void)
{
    MPU6050_RequestData();
    QMC5883_RequestData();
}

uint8_t AHRS_MW_IsDataReady(void)
{
    // 至少陀螺仪数据要就绪
    return mpu6050_i2c_rx_done;
}

uint8_t AHRS_MW_GetData(IMU_Data_t *data)
{
    // 默认清零
    data->acc_valid = 0;
    data->gyro_valid = 0;
    data->mag_valid = 0;

    // 解析 MPU6050
    if (mpu6050_i2c_rx_done)
    {
        MPU6050_ParseData(&data->ax, &data->ay, &data->az,
                          &data->gx, &data->gy, &data->gz);
        data->acc_valid = 1;
        data->gyro_valid = 1;
        mpu6050_i2c_rx_done = 0;
    }

    // 解析 QMC5883
    if (QMC5883_ParseData(&data->mx, &data->my, &data->mz))
    {
        data->mag_valid = 1;
    }

    return data->gyro_valid;
}
