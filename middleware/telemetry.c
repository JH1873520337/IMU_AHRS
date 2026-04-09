//
// Created by ASUS on 26-2-22.
//

#include "telemetry.h"
#include "drv_uart1.h" // 依赖驱动层
#include <string.h>

// Vofa+ JustFloat 帧尾: 00 00 80 7f
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
static uint32_t telemetry_attitude_calls = 0;
static uint32_t telemetry_sensor_calls = 0;

void Telemetry_SendAttitude(float roll, float pitch, float yaw)
{
    telemetry_attitude_calls++;

    // 缓冲区: 3个float (12字节) + 帧尾 (4字节)
    static uint8_t tx_buf[16];

    // 1. 数据打包 (小端模式)
    memcpy(&tx_buf[0], &roll, 4);
    memcpy(&tx_buf[4], &pitch, 4);
    memcpy(&tx_buf[8], &yaw, 4);

    // 2. 添加帧尾
    memcpy(&tx_buf[12], tail, 4);

    // 3. 调用驱动层发送
    Drv_UART_Transmit_DMA(tx_buf, 16);
}

void Telemetry_SendSensors(float ax, float ay, float az, float gx, float gy, float gz)
{
    telemetry_sensor_calls++;

    // 6个float + 帧尾 = 28字节
    static uint8_t tx_buf[28];

    memcpy(&tx_buf[0], &ax, 4);
    memcpy(&tx_buf[4], &ay, 4);
    memcpy(&tx_buf[8], &az, 4);
    memcpy(&tx_buf[12], &gx, 4);
    memcpy(&tx_buf[16], &gy, 4);
    memcpy(&tx_buf[20], &gz, 4);
    memcpy(&tx_buf[24], tail, 4);

    Drv_UART_Transmit_DMA(tx_buf, 28);
}

void Telemetry_GetStats(uint32_t *attitude_calls, uint32_t *sensor_calls)
{
    if (attitude_calls != 0)
    {
        *attitude_calls = telemetry_attitude_calls;
    }
    if (sensor_calls != 0)
    {
        *sensor_calls = telemetry_sensor_calls;
    }
}
