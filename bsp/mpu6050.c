#include "mpu6050.h"
#include "math.h"
#include "i2c.h"  // 引入I2C相关定义

/* DMA传输完成标志位（volatile确保编译器不优化） */
volatile uint8_t mpu6050_i2c_tx_done = 0;
volatile uint8_t mpu6050_i2c_rx_done = 0;

/* 私有函数：等待DMA传输完成并处理超时 */
/* 私有函数：等待DMA传输完成并处理超时（重写版，无类型判断，直接适配I2C2） */
static HAL_StatusTypeDef MPU6050_WaitDMAComplete(volatile uint8_t *flag, uint32_t timeout)
{
    uint32_t tick_start = HAL_GetTick();  // 记录等待起始时间

    // 循环等待DMA完成标志置1，或超时退出
    while(!(*flag))
    {
        // 超时判断：超过设定时间则执行总线重置
        if((HAL_GetTick() - tick_start) > timeout)
        {
            HAL_I2C_DeInit(MPU6050_I2C_HANDLE);  // 解初始化I2C2，重置总线
            HAL_Delay(10);                       // 等待总线电气特性稳定
            MX_I2C2_Init();                      // 重新初始化I2C2（直接适配，无需判断）
            *flag = 0;                           // 清零标志位，防止后续死等
            return HAL_TIMEOUT;                  // 返回超时错误，上层感知
        }
    }

    // DMA传输正常完成：清零标志位，供下一次传输使用
    *flag = 0;
    return HAL_OK;  // 返回正常完成状态
}

/* 写单个字节到MPU6050寄存器（DMA版，带同步等待） */
HAL_StatusTypeDef MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[2] = {RegAddress, Data};

    // 重置传输完成标志
    mpu6050_i2c_tx_done = 0;

    // 启动DMA发送
    status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, tx_buf, 2);
    if(status != HAL_OK) return status;

    // 等待DMA发送完成
    return MPU6050_WaitDMAComplete(&mpu6050_i2c_tx_done, MPU6050_DMA_TIMEOUT);
}

/* 写多个字节到MPU6050寄存器（DMA版，带同步等待） */
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[32];

    if(Size > 30) return HAL_ERROR;  // 防止缓冲区溢出

    // 构造发送缓冲区：首字节为寄存器地址，后续为数据
    tx_buf[0] = RegAddress;
    for(uint8_t i=0; i<Size; i++)
    {
        tx_buf[i+1] = Data[i];
    }

    // 重置传输完成标志
    mpu6050_i2c_tx_done = 0;

    // 启动DMA发送
    status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, tx_buf, Size+1);
    if(status != HAL_OK) return status;

    // 等待DMA发送完成
    return MPU6050_WaitDMAComplete(&mpu6050_i2c_tx_done, MPU6050_DMA_TIMEOUT);
}

/* 从MPU6050寄存器读单个字节（DMA版，带同步等待） */
HAL_StatusTypeDef MPU6050_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    HAL_StatusTypeDef status;

    // 步骤1：发送寄存器地址（DMA写）
    mpu6050_i2c_tx_done = 0;
    status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, &RegAddress, 1);
    if(status != HAL_OK) return status;
    if(MPU6050_WaitDMAComplete(&mpu6050_i2c_tx_done, MPU6050_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    // 步骤2：接收数据（DMA读）
    mpu6050_i2c_rx_done = 0;
    status = HAL_I2C_Master_Receive_DMA(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS, Data, 1);
    if(status != HAL_OK) return status;
    if(MPU6050_WaitDMAComplete(&mpu6050_i2c_rx_done, MPU6050_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    return HAL_OK;
}

/* 从MPU6050寄存器读多个字节（DMA版，带同步等待） */
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size)
{
    HAL_StatusTypeDef status;

    // 步骤1：发送寄存器地址（DMA写）
    mpu6050_i2c_tx_done = 0;
    status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, &RegAddress, 1);
    if(status != HAL_OK) return status;
    if(MPU6050_WaitDMAComplete(&mpu6050_i2c_tx_done, MPU6050_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    // 步骤2：接收多个字节（DMA读）
    mpu6050_i2c_rx_done = 0;
    status = HAL_I2C_Master_Receive_DMA(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS, Data, Size);
    if(status != HAL_OK) return status;
    if(MPU6050_WaitDMAComplete(&mpu6050_i2c_rx_done, MPU6050_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    return HAL_OK;
}

/* 检查MPU6050设备ID是否匹配（新增） */
uint8_t MPU6050_CheckID(void)
{
    uint8_t dev_id = 0;
    if(MPU6050_ReadByte(MPU6050_WHO_AM_I, &dev_id) != HAL_OK)
        return 0;  // 读取失败

    return (dev_id == MPU6050_EXPECTED_ID) ? 1 : 0;
}

/* MPU6050初始化（增加ID检查+错误容错） */
void MPU6050_Init(void)
{
    // 等待I2C总线稳定
    HAL_Delay(100);

    // 检查设备ID，不匹配则直接返回
    if(!MPU6050_CheckID())
    {
        // 可在此处添加错误提示（如LED闪烁、串口打印）
        return;
    }

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


/* 获取6轴原始数据（优化为burst读14字节：acc + temp + gyro） */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t raw_data[14] = {0};  // 缓冲区：acc(0-5), temp(6-7), gyro(8-13)

    // 一次性burst读从0x3B到0x48的14字节
    if(MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, raw_data, 14) != HAL_OK)
    {
        *AccX = *AccY = *AccZ = *GyroX = *GyroY = *GyroZ = 0;
        return;  // 读取失败，全置0返回
    }

    // 拼接加速度数据
    *AccX = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    *AccY = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    *AccZ = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // 跳过温度（如果需要，可加int16_t temp = (raw_data[6] << 8) | raw_data[7];）

    // 拼接陀螺仪数据
    *GyroX = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    *GyroY = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    *GyroZ = (int16_t)((raw_data[12] << 8) | raw_data[13]);
}

/* 获取MPU6050的真实物理数据（增加数据有效性判断） */
void MPU6050_GetRealData(float *ax_real, float *ay_real, float *az_real,
                         float *gx_real, float *gy_real, float *gz_real)
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    // 1. 先获取原始数据
    MPU6050_GetData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // 2. 换算为真实物理量（单位：g 和 rad/s）
    *ax_real = (float)ax_raw / ACCEL_SENSITIVITY ; //单位为g
    *ay_real = (float)ay_raw / ACCEL_SENSITIVITY ;
    *az_real = (float)az_raw / ACCEL_SENSITIVITY ;

    *gx_real = (float)gx_raw / GYRO_SENSITIVITY * M_PI / 180.0f;  // 转换为rad/s
    *gy_real = (float)gy_raw / GYRO_SENSITIVITY * M_PI / 180.0f;
    *gz_real = (float)gz_raw / GYRO_SENSITIVITY * M_PI / 180.0f;
}