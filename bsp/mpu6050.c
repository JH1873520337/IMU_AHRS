#include "mpu6050.h"
#include "math.h"

// 私有宏：DMA传输超时时间（ms），根据实际场景调整
#define MPU6050_DMA_TIMEOUT 100

// 私有函数：等待I2C DMA传输完成（封装复用）
static HAL_StatusTypeDef MPU6050_Wait_DMA_Complete(I2C_HandleTypeDef *hi2c)
{
  uint32_t tick_start = HAL_GetTick();
  // 等待I2C状态变为就绪，或超时
  while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
    if ((HAL_GetTick() - tick_start) > MPU6050_DMA_TIMEOUT)
    {
      hi2c->State = HAL_I2C_STATE_READY; // 重置状态，避免卡死
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/* 写单个字节到MPU6050寄存器（DMA版） */
HAL_StatusTypeDef MPU6050_WriteByte(uint8_t RegAddress, uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t tx_buf[2] = {RegAddress, Data};

  // 1. 检查I2C是否就绪
  if (HAL_I2C_GetState(MPU6050_I2C_HANDLE) != HAL_I2C_STATE_READY)
  {
    return HAL_BUSY;
  }

  // 2. 启动DMA写传输
  status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, tx_buf, 2);
  if (status != HAL_OK)
  {
    return status;
  }

  // 3. 等待DMA传输完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);

  // 4. 检查传输结果（HAL_I2C_GetError获取具体错误）
  if (status == HAL_OK && HAL_I2C_GetError(MPU6050_I2C_HANDLE) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }

  return status;
}

/* 写多个字节到MPU6050寄存器（DMA版） */
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t tx_buf[32];

  // 1. 缓冲区边界检查（修正原错误：tx_buf[0]是寄存器地址，剩余31字节存数据）
  if (Size > 31 || Data == NULL)
  {
    return HAL_ERROR;
  }

  // 2. 检查I2C就绪状态
  if (HAL_I2C_GetState(MPU6050_I2C_HANDLE) != HAL_I2C_STATE_READY)
  {
    return HAL_BUSY;
  }

  // 3. 填充发送缓冲区（首字节=寄存器地址，后续=数据）
  tx_buf[0] = RegAddress;
  for (uint8_t i = 0; i < Size; i++)
  {
    tx_buf[i + 1] = Data[i];
  }

  // 4. 启动DMA写传输
  status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, tx_buf, Size + 1);
  if (status != HAL_OK)
  {
    return status;
  }

  // 5. 等待DMA传输完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);

  // 6. 检查传输错误
  if (status == HAL_OK && HAL_I2C_GetError(MPU6050_I2C_HANDLE) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }

  return status;
}

/* 从MPU6050寄存器读单个字节（DMA版） */
HAL_StatusTypeDef MPU6050_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
  HAL_StatusTypeDef status = HAL_OK;

  // 1. 入参检查
  if (Data == NULL)
  {
    return HAL_ERROR;
  }

  // 2. 检查I2C就绪状态
  if (HAL_I2C_GetState(MPU6050_I2C_HANDLE) != HAL_I2C_STATE_READY)
  {
    return HAL_BUSY;
  }

  // 第一步：DMA发送要读取的寄存器地址
  status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, &RegAddress, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  // 等待地址发送完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);
  if (status != HAL_OK)
  {
    return status;
  }

  // 第二步：DMA接收寄存器数据
  status = HAL_I2C_Master_Receive_DMA(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS, Data, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  // 等待数据接收完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);

  // 检查最终错误
  if (status == HAL_OK && HAL_I2C_GetError(MPU6050_I2C_HANDLE) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }

  return status;
}

/* 从MPU6050寄存器读多个字节（DMA版） */
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;

  // 1. 入参检查
  if (Size == 0 || Data == NULL)
  {
    return HAL_ERROR;
  }

  // 2. 检查I2C就绪状态
  if (HAL_I2C_GetState(MPU6050_I2C_HANDLE) != HAL_I2C_STATE_READY)
  {
    return HAL_BUSY;
  }

  // 第一步：DMA发送起始寄存器地址
  status = HAL_I2C_Master_Transmit_DMA(MPU6050_I2C_HANDLE, MPU6050_W_ADDRESS, &RegAddress, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  // 等待地址发送完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);
  if (status != HAL_OK)
  {
    return status;
  }

  // 第二步：DMA连续接收多个字节
  status = HAL_I2C_Master_Receive_DMA(MPU6050_I2C_HANDLE, MPU6050_R_ADDRESS, Data, Size);
  if (status != HAL_OK)
  {
    return status;
  }
  // 等待数据接收完成
  status = MPU6050_Wait_DMA_Complete(MPU6050_I2C_HANDLE);

  // 检查最终错误
  if (status == HAL_OK && HAL_I2C_GetError(MPU6050_I2C_HANDLE) != HAL_I2C_ERROR_NONE)
  {
    return HAL_ERROR;
  }

  return status;
}

// 以下保留原初始化/数据读取函数（仅补充错误检查）
/* MPU6050初始化 */
void MPU6050_Init(void)
{
  HAL_StatusTypeDef status;
  uint8_t dev_id;

  // 等待I2C总线稳定
  HAL_Delay(100);

  // 先检查设备ID是否正确
  status = MPU6050_ReadByte(MPU6050_WHO_AM_I, &dev_id);
  if (status != HAL_OK || (dev_id & 0x7F) != MPU6050_EXPECTED_ID )
  {
    // 设备ID错误/通信失败，可添加报错逻辑（如LED闪烁、串口打印）
    while (1);
  }

  // 解除睡眠模式，选择陀螺仪时钟源（X轴陀螺）
  status = MPU6050_WriteByte(MPU6050_PWR_MGMT_1, 0X01);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  // 关闭所有辅助电源控制
  status = MPU6050_WriteByte(MPU6050_PWR_MGMT_2, 0X00);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  // 采样率分频：10分频（采样率=1000/(1+9)=100Hz）
  status = MPU6050_WriteByte(MPU6050_SMPLRT_DIV, 0X09);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  // 配置低通滤波器：184Hz带宽
  status = MPU6050_WriteByte(MPU6050_CONFIG, 0X01);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  // 陀螺仪量程：±250°/s
  status = MPU6050_WriteByte(MPU6050_GYRO_CONFIG, 0X00);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  // 加速度计量程：±2g,高通滤波器截止频率为5hz
  status = MPU6050_WriteByte(MPU6050_ACCEL_CONFIG, 0X01);
  if (status != HAL_OK) goto init_error;
  HAL_Delay(10);

  return;

init_error:
  // 初始化失败，死循环报错（可替换为自定义错误处理）
  while (1);
}

/* 获取6轴原始数据 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
  uint8_t accel_data[6];
  uint8_t gyro_data[6];
  HAL_StatusTypeDef status;

  // 第一步：读加速度（0x3B-0x40，6字节）
  status = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, accel_data, 6);
  if (status != HAL_OK) return;

  // 第二步：读陀螺仪（0x43-0x48，6字节）
  status = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H, gyro_data, 6);
  if (status != HAL_OK) return;

  // 拼接加速度数据
  *AccX = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  *AccY = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  *AccZ = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  // 拼接陀螺仪数据
  *GyroX = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  *GyroY = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  *GyroZ = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
}

/* 获取MPU6050的真实物理数据 */
void MPU6050_GetRealData(float *ax_real, float *ay_real, float *az_real,
                         float *gx_real, float *gy_real, float *gz_real)
{
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  // 1. 先获取原始数据
  MPU6050_GetData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // 2. 换算为真实物理量（原始数据 ÷ 灵敏度）
  *ax_real = (float)ax_raw / ACCEL_SENSITIVITY;
  *ay_real = (float)ay_raw / ACCEL_SENSITIVITY;
  *az_real = (float)az_raw / ACCEL_SENSITIVITY;

  // 陀螺仪：°/s 转 rad/s
  *gx_real = (float)gx_raw * M_PI / 180.0f / GYRO_SENSITIVITY;
  *gy_real = (float)gy_raw * M_PI / 180.0f / GYRO_SENSITIVITY;
  *gz_real = (float)gz_raw * M_PI / 180.0f / GYRO_SENSITIVITY;
}

/* 获取设备ID（正常返回 MPU6050_EXPECTED_ID ） */
uint8_t MPU6050_GetID(void)
{
  uint8_t dev_id = 0;
  if (HAL_OK == MPU6050_ReadByte(MPU6050_WHO_AM_I, &dev_id))
  {
    return dev_id & 0x7F; // 仅保留低7位有效ID
  }
  return 0xFF; // 读取失败返回无效值
}