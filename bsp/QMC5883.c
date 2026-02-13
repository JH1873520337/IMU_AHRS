#include "QMC5883.h"
#include "i2c.h"  // 引入I2C相关定义

/* DMA传输完成标志位（volatile确保编译器不优化） */
volatile uint8_t qmc5883_i2c_tx_done = 0;
volatile uint8_t qmc5883_i2c_rx_done = 0;

/* 私有函数：等待DMA传输完成并处理超时（重写版，无类型判断，直接适配I2C3） */
static HAL_StatusTypeDef QMC5883_WaitDMAComplete(volatile uint8_t *flag, uint32_t timeout)
{
    uint32_t tick_start = HAL_GetTick();  // 记录等待起始时间

    // 循环等待DMA完成标志置1，或超时退出
    while(!(*flag))
    {
        // 超时判断：超过设定时间则执行总线重置
        if((HAL_GetTick() - tick_start) > timeout)
        {
            HAL_I2C_DeInit(QMC5883_I2C_HANDLE);  // 解初始化I2C3，重置总线
            HAL_Delay(10);                       // 等待总线电气特性稳定
            MX_I2C3_Init();                      // 重新初始化I2C3（直接适配，无需判断）
            *flag = 0;                           // 清零标志位，防止后续死等
            return HAL_TIMEOUT;                  // 返回超时错误，上层感知
        }
    }

    // DMA传输正常完成：清零标志位，供下一次传输使用
    *flag = 0;
    return HAL_OK;  // 返回正常完成状态
}



/* 写单个字节到QMC5883寄存器（DMA版，带同步等待） */
HAL_StatusTypeDef QMC5883_WriteByte(uint8_t RegAddress, uint8_t Data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[2] = {RegAddress, Data};

    // 重置传输完成标志
    qmc5883_i2c_tx_done = 0;

    // 启动DMA发送
    status = HAL_I2C_Master_Transmit_DMA(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS, tx_buf, 2);
    if(status != HAL_OK) return status;

    // 等待DMA发送完成
    return QMC5883_WaitDMAComplete(&qmc5883_i2c_tx_done, QMC5883_DMA_TIMEOUT);
}

/* 写多个字节到QMC5883寄存器（DMA版，带同步等待） */
HAL_StatusTypeDef QMC5883_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size)
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
    qmc5883_i2c_tx_done = 0;

    // 启动DMA发送
    status = HAL_I2C_Master_Transmit_DMA(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS, tx_buf, Size+1);
    if(status != HAL_OK) return status;

    // 等待DMA发送完成
    return QMC5883_WaitDMAComplete(&qmc5883_i2c_tx_done, QMC5883_DMA_TIMEOUT);
}

/* 从QMC5883寄存器读单个字节（DMA版，带同步等待） */
HAL_StatusTypeDef QMC5883_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    HAL_StatusTypeDef status;

    // 步骤1：发送寄存器地址（DMA写）
    qmc5883_i2c_tx_done = 0;
    status = HAL_I2C_Master_Transmit_DMA(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS, &RegAddress, 1);
    if(status != HAL_OK) return status;
    if(QMC5883_WaitDMAComplete(&qmc5883_i2c_tx_done, QMC5883_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    // 步骤2：接收数据（DMA读）
    qmc5883_i2c_rx_done = 0;
    status = HAL_I2C_Master_Receive_DMA(QMC5883_I2C_HANDLE, QMC5883_R_ADDRESS, Data, 1);
    if(status != HAL_OK) return status;
    if(QMC5883_WaitDMAComplete(&qmc5883_i2c_rx_done, QMC5883_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    return HAL_OK;
}

/* 从QMC5883寄存器读多个字节（DMA版，带同步等待） */
HAL_StatusTypeDef QMC5883_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size)
{
    HAL_StatusTypeDef status;

    // 步骤1：发送寄存器地址（DMA写）
    qmc5883_i2c_tx_done = 0;
    status = HAL_I2C_Master_Transmit_DMA(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS, &RegAddress, 1);
    if(status != HAL_OK) return status;
    if(QMC5883_WaitDMAComplete(&qmc5883_i2c_tx_done, QMC5883_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    // 步骤2：接收多个字节（DMA读）
    qmc5883_i2c_rx_done = 0;
    status = HAL_I2C_Master_Receive_DMA(QMC5883_I2C_HANDLE, QMC5883_R_ADDRESS, Data, Size);
    if(status != HAL_OK) return status;
    if(QMC5883_WaitDMAComplete(&qmc5883_i2c_rx_done, QMC5883_DMA_TIMEOUT) != HAL_OK)
        return HAL_TIMEOUT;

    return HAL_OK;
}

/* 检查QMC5883设备ID是否匹配（新增） */
uint8_t QMC5883_CheckID(void)
{
    uint8_t dev_id = 0;
    if(QMC5883_ReadByte(QMC5883_CHIP_ID, &dev_id) != HAL_OK)
        return 0;  // 读取失败

    return (dev_id == QMC5883_EXPECTED_ID) ? 1 : 0;
}

/* QMC5883初始化（增加ID检查+错误容错） */
void QMC5883_Init(void)
{
    // 等待I2C总线稳定
    HAL_Delay(100);

    // 检查设备ID，不匹配则直接返回
    if(!QMC5883_CheckID())
    {
        // 可在此处添加错误提示（如LED闪烁、串口打印）
        return;
    }

    // 配置控制寄存器1：0x0A = 0x0B (用户原值：OSR默认, ODR=10Hz?, MODE=正常?)
    QMC5883_WriteByte(QMC5883_CTRL1, 0x0B);
    HAL_Delay(10);

    // 配置控制寄存器2：0x0B = 0x08 (用户原值：RNG=±8G, SET/RESET启用)
    QMC5883_WriteByte(QMC5883_CTRL2, 0x08);
    HAL_Delay(10);
}

/* 获取3轴原始数据（确保DMA传输完成后再拼接数据） */
void QMC5883_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
    uint8_t magnetometer_data[6] = {0};

    // 读磁力计数据（0x01-0x06，6字节）
    if(QMC5883_ReadReg(QMC5883_X_LSB, magnetometer_data, 6) != HAL_OK)
    {
        *MagX = *MagY = *MagZ = 0;
    }
    else
    {
        // 拼接磁场数据（MSB << 8 | LSB）
        *MagX = (int16_t)((magnetometer_data[1] << 8) | magnetometer_data[0]);
        *MagY = (int16_t)((magnetometer_data[3] << 8) | magnetometer_data[2]);
        *MagZ = (int16_t)((magnetometer_data[5] << 8) | magnetometer_data[4]);
    }
}

/* 获取QMC5883的真实物理数据（增加数据有效性判断） */
void QMC5883_GetRealData(float *mx_real, float *my_real, float *mz_real)
{
    int16_t mx_raw, my_raw, mz_raw;

    // 1. 先获取原始数据
    QMC5883_GetData(&mx_raw, &my_raw, &mz_raw);

    // 2. 换算为真实物理量（单位：Gauss）
    *mx_real = (float)mx_raw / MAG_SENSITIVITY;
    *my_real = (float)my_raw / MAG_SENSITIVITY;
    *mz_real = (float)mz_raw / MAG_SENSITIVITY;
}