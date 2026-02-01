#include "mpu6050.h"

/* 写单个字节到MPU6050寄存器 */
/**
 *写单个字节到MPU6050寄存器
 * @param RegAddress
 * @param Data
 * @return
 */
HAL_StatusTypeDef MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data)
{
    uint8_t tx_buf[2] = {RegAddress, Data};
    // HAL库I2C主发送：地址+寄存器+数据
    return HAL_I2C_Master_Transmit(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS,
                                   tx_buf, 2, 100);  // 超时100ms
}

/* 写多个字节到MPU6050寄存器 */
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size)
{
    uint8_t tx_buf[32];  // 最大支持32字节写入（可根据需求调整）
    if(Size > 30) return HAL_ERROR;  // 防止缓冲区溢出

    tx_buf[0] = RegAddress;  // 首字节为寄存器地址
    for(uint8_t i=0; i<Size; i++)
    {
        tx_buf[i+1] = Data[i];
    }
    return HAL_I2C_Master_Transmit(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS,
                                   tx_buf, Size+1, 100);
}

/* 从MPU6050寄存器读单个字节 */
HAL_StatusTypeDef MPU6050_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    // 1. 发送要读取的寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS,
                                                       &RegAddress, 1, 100);
    if(status != HAL_OK) return status;

    // 2. 读取该寄存器的数据
    return HAL_I2C_Master_Receive(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS,
                                  Data, 1, 100);
}

/* 从MPU6050寄存器读多个字节 */
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size)
{
    // 1. 发送起始寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS,
                                                       &RegAddress, 1, 100);
    if(status != HAL_OK) return status;

    // 2. 连续读取多个字节
    return HAL_I2C_Master_Receive(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS,
                                  Data, Size, 100);
}

/* MPU6050初始化 */
void MPU6050_Init(void)
{
    // 等待I2C就绪
    HAL_Delay(100);

    // 解除睡眠模式，选择陀螺仪时钟源（X轴陀螺）
    MPU6050_WriteByte(MPU6050_PWR_MGMT_1, 0X01);
    HAL_Delay(10);

    // 关闭所有辅助电源控制
    MPU6050_WriteByte(MPU6050_PWR_MGMT_2, 0X00);
    HAL_Delay(10);

    // 采样率分频：10分频（采样率=1000/(1+9)=100Hz）
    MPU6050_WriteByte(MPU6050_SMPLRT_DIV, 0X09);
    HAL_Delay(10);

    // 配置低通滤波器：184Hz带宽
    MPU6050_WriteByte(MPU6050_CONFIG, 0X01);
    HAL_Delay(10);

    // 陀螺仪量程：±250°/s
    MPU6050_WriteByte(MPU6050_GYRO_CONFIG, 0X00);
    HAL_Delay(10);

    // 加速度计量程：±2g,高通滤波器截止频率为5hz
    MPU6050_WriteByte(MPU6050_ACCEL_CONFIG, 0X01);
    HAL_Delay(10);
}

/* 获取6轴原始数据 */
/* 获取6轴原始数据（跳过温度，仅读加速度+陀螺仪） */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    // 第一步：读加速度（0x3B-0x40，6字节）
    MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, accel_data, 6);
    // 第二步：读陀螺仪（0x43-0x48，6字节）
    MPU6050_ReadReg(MPU6050_GYRO_XOUT_H, gyro_data, 6);

    // 拼接加速度数据
    *AccX = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    *AccY = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    *AccZ = (int16_t)((accel_data[4] << 8) | accel_data[5]);

    // 拼接陀螺仪数据
    *GyroX = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    *GyroY = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    *GyroZ = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
}


/**
 * @brief 获取MPU6050的真实物理数据
 * @param 返回值，直接将变量的地址给函数，可以得到实际测算值
 */
void MPU6050_GetRealData(float *ax_real, float *ay_real, float *az_real,
                         float *gx_real, float *gy_real, float *gz_real)
{
    int16_t ax_raw, ay_raw, az_raw;  // 加速度原始16位数据
    int16_t gx_raw, gy_raw, gz_raw;  // 陀螺仪原始16位数据

    // 1. 先获取原始数据（调用你已写的MPU6050_GetData）
    MPU6050_GetData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // 2. 换算为真实物理量（原始数据 ÷ 灵敏度）
    *ax_real = (float)ax_raw / ACCEL_SENSITIVITY;
    *ay_real = (float)ay_raw / ACCEL_SENSITIVITY;
    *az_real = (float)az_raw / ACCEL_SENSITIVITY;

    *gx_real = (float)gx_raw / GYRO_SENSITIVITY;
    *gy_real = (float)gy_raw / GYRO_SENSITIVITY;
    *gz_real = (float)gz_raw / GYRO_SENSITIVITY;
}

/* 获取设备ID（正常返回0x68） */
uint8_t MPU6050_GetID(void)
{
    uint8_t dev_id = 0;
    MPU6050_ReadByte(MPU6050_WHO_AM_I, &dev_id);
    return dev_id;
}