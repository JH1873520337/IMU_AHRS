//
// Created by ASUS on 26-2-22.
//

#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>

/**
 * @brief 发送姿态数据到地面站 (Vofa+ JustFloat协议)
 * @param roll  横滚角 (deg)
 * @param pitch 俯仰角 (deg)
 * @param yaw   航向角 (deg)
 */
void Telemetry_SendAttitude(float roll, float pitch, float yaw);

/**
 * @brief 发送传感器原始数据 (可选，用于调试)
 */
void Telemetry_SendSensors(float ax, float ay, float az, float gx, float gy, float gz);

#endif //TELEMETRY_H
