#include "mpu6050.h"
#include "math.h"
#include "i2c.h"

// 接收缓冲区：Acc(6) + Temp(2) + Gyro(6) = 14 bytes
static uint8_t mpu6050_buffer[14];
volatile uint8_t mpu6050_i2c_rx_done = 0;
volatile uint32_t mpu6050_req_count = 0;
volatile uint32_t mpu6050_req_ok_count = 0;
volatile uint32_t mpu6050_req_err_count = 0;
volatile uint32_t mpu6050_last_req_status = 0;

/* 私有函数：阻塞式写寄存器 (仅用于初始化) */
static void MPU6050_WriteByte_Block(uint8_t RegAddress, uint8_t Data)
{
    // 使用阻塞式 API 确保初始化指令稳定执行
    HAL_I2C_Mem_Write(MPU6050_I2C_HANDLE, MPU6050_I2C_ADDR_W , RegAddress,
                      I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

HAL_StatusTypeDef MPU6050_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    // 1. 发送要读取的寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(MPU6050_I2C_HANDLE, MPU6050_I2C_ADDR_W,
                                                       &RegAddress, 1, 100);
    if(status != HAL_OK) return status;

    // 2. 读取该寄存器的数据
    return HAL_I2C_Master_Receive(MPU6050_I2C_HANDLE, MPU6050_I2C_ADDR_R,
                                  Data, 1, 100);
}

/* 初始化函数 */
void MPU6050_Init(void)
{
    HAL_Delay(100); // 上电等待

    // 复位设备
    MPU6050_WriteByte_Block(MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(50);
    // 唤醒并选择 PLL X 轴时钟
    MPU6050_WriteByte_Block(MPU6050_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    MPU6050_WriteByte_Block(MPU6050_PWR_MGMT_2, 0x00);

    // 配置采样率 (DLPF开启时: 1kHz / (1 + 0) = 1kHz)
    MPU6050_WriteByte_Block(MPU6050_SMPLRT_DIV, 0x00);
    // 配置低通滤波到 184Hz，进一步减小动态滞后
    MPU6050_WriteByte_Block(MPU6050_CONFIG, 0x01);
    // 陀螺仪量程提升到 ±500dps，动态时不容易贴边
    MPU6050_WriteByte_Block(MPU6050_GYRO_CONFIG, 0x08);
    // 加速度量程 ±2g
    MPU6050_WriteByte_Block(MPU6050_ACCEL_CONFIG, 0x00);

}

uint8_t MPU6050_GetID(void)
{
    uint8_t dev_id = 0;
    MPU6050_ReadByte(MPU6050_WHO_AM_I, &dev_id);
    return dev_id;
}

// === 核心数据流程 ===

/**
 * @brief  发起 DMA 读取请求（非阻塞）
 * @note   调用此函数后，程序会立刻返回。需等待 rx_done 标志位置1。
 */
void MPU6050_RequestData(void)
{
    HAL_StatusTypeDef status;

    mpu6050_req_count++;
    mpu6050_i2c_rx_done = 0; // 清除标志位

    // 使用 Mem_Read_DMA，一次读出14字节 (0x3B开始)
    // 此时 buffer 里的数据还是旧的，直到中断发生
    status = HAL_I2C_Mem_Read_DMA(MPU6050_I2C_HANDLE, MPU6050_I2C_ADDR_W, MPU6050_ACCEL_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT, mpu6050_buffer, 14);
    mpu6050_last_req_status = (uint32_t)status;

    if (status == HAL_OK)
    {
        mpu6050_req_ok_count++;
    }
    else
    {
        mpu6050_req_err_count++;
    }
}

void MPU6050_GetRequestStats(uint32_t *req, uint32_t *ok, uint32_t *err, uint32_t *last_status)
{
    if (req != 0)
    {
        *req = mpu6050_req_count;
    }
    if (ok != 0)
    {
        *ok = mpu6050_req_ok_count;
    }
    if (err != 0)
    {
        *err = mpu6050_req_err_count;
    }
    if (last_status != 0)
    {
        *last_status = mpu6050_last_req_status;
    }
}

/**
 * @brief  解析缓冲区数据（在确认 rx_done == 1 后调用）
 */
void MPU6050_ParseData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;

    // 原始数据拼接
    AccX = (int16_t)((mpu6050_buffer[0] << 8) | mpu6050_buffer[1]);
    AccY = (int16_t)((mpu6050_buffer[2] << 8) | mpu6050_buffer[3]);
    AccZ = (int16_t)((mpu6050_buffer[4] << 8) | mpu6050_buffer[5]);
    // buffer[6], buffer[7] 是温度，这里跳过
    GyroX = (int16_t)((mpu6050_buffer[8] << 8) | mpu6050_buffer[9]);
    GyroY = (int16_t)((mpu6050_buffer[10] << 8) | mpu6050_buffer[11]);
    GyroZ = (int16_t)((mpu6050_buffer[12] << 8) | mpu6050_buffer[13]);

    // 物理量转换
    *ax = (float)AccX / ACCEL_SENSITIVITY;//g
    *ay = (float)AccY / ACCEL_SENSITIVITY;
    *az = (float)AccZ / ACCEL_SENSITIVITY;

    *gx = ((float)GyroX / GYRO_SENSITIVITY) * 0.0174532925f; // rad/s
    *gy = ((float)GyroY / GYRO_SENSITIVITY) * 0.0174532925f;
    *gz = ((float)GyroZ / GYRO_SENSITIVITY) * 0.0174532925f;

    // 解析完清空标志（可选，取决于逻辑）
    // mpu6050_i2c_rx_done = 0;
}
