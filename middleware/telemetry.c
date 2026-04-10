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

static void Telemetry_SendFloat3(float ch1, float ch2, float ch3)
{
    static uint8_t tx_buf[16];

    memcpy(&tx_buf[0], &ch1, 4);
    memcpy(&tx_buf[4], &ch2, 4);
    memcpy(&tx_buf[8], &ch3, 4);
    memcpy(&tx_buf[12], tail, 4);

    Drv_UART_Transmit_DMA(tx_buf, 16);
}

void Telemetry_SendAttitude(float roll, float pitch, float yaw)
{
    telemetry_attitude_calls++;
    Telemetry_SendFloat3(roll, pitch, yaw);
}

void Telemetry_SendMagRaw(float mx, float my, float mz)
{
    telemetry_sensor_calls++;
    Telemetry_SendFloat3(mx, my, mz);
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
