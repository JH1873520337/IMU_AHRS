#include "QMC5883.h"

// --- 全局变量 ---
// 接收缓冲区: X_L, X_H, Y_L, Y_H, Z_L, Z_H
static uint8_t qmc_dma_buffer[6];
// 完成标志位
volatile uint8_t qmc5883_i2c_rx_done = 0;

/* ============================================================
   保留部分：辅助函数（用于初始化和阻塞读ID）
   ============================================================ */

// 写一个字节 (保持不变，用于初始化)
HAL_StatusTypeDef QMC5883_WriteByte(uint8_t RegAddress, uint8_t Data)
{
    uint8_t tx_buf[2] = {RegAddress, Data};
    return HAL_I2C_Master_Transmit(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                                   tx_buf, 2, 100);
}

// 读一个字节 (保持不变，用于读ID)
HAL_StatusTypeDef QMC5883_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    // 使用 Mem_Read 简化原本的 Transmit + Receive
    return HAL_I2C_Mem_Read(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                            RegAddress, I2C_MEMADD_SIZE_8BIT,
                            Data, 1, 100);
}

/* ============================================================
   核心改动：DMA 功能区
   ============================================================ */

/**
 * @brief  初始化 (完全保留你验证过的逻辑)
 */
void QMC5883_Init(void)
{
    HAL_Delay(100); // 上电延时

    // 你原来的配置值
    QMC5883_WriteByte(QMC5883_Control_Registers1, 0X0B);
    HAL_Delay(10);

    QMC5883_WriteByte(QMC5883_Control_Registers2, 0X08);
    HAL_Delay(10);
}

/**
 * @brief  步骤1：发起 DMA 读取请求
 *         原理：使用 HAL_I2C_Mem_Read_DMA 自动完成 写寄存器+读数据
 */
void QMC5883_RequestData(void)
{
    qmc5883_i2c_rx_done = 0; // 清除标志

    // 参数说明：
    // 1. I2C句柄
    // 2. 设备地址 (通常 HAL库 Mem_Read 接收的是 Write Address，内部自动处理读写位)
    // 3. 寄存器地址 (QMC5883_DataOut_XLSB)
    // 4. 寄存器地址长度 (8bit)
    // 5. 数据缓冲区
    // 6. 数据长度 (6字节)
    HAL_I2C_Mem_Read_DMA(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                         QMC5883_DataOut_XLSB, I2C_MEMADD_SIZE_8BIT,
                         qmc_dma_buffer, 6);
}

/**
 * @brief  步骤2：解析 DMA 数据
 *         在 main 循环中检查 qmc5883_rx_done == 1 后调用
 */
uint8_t QMC5883_ParseData(float *mx_real, float *my_real, float *mz_real)
{
    if (qmc5883_i2c_rx_done)
    {
        int16_t raw_x, raw_y, raw_z;

        // 1. 拼接原始数据 (Little Endian)
        raw_x = (int16_t)((qmc_dma_buffer[1] << 8) | qmc_dma_buffer[0]);
        raw_y = (int16_t)((qmc_dma_buffer[3] << 8) | qmc_dma_buffer[2]);
        raw_z = (int16_t)((qmc_dma_buffer[5] << 8) | qmc_dma_buffer[4]);

        // 2. 转换为真实值 (Gauss)
        *mx_real = (float)raw_x / MAG_SENSITIVITY;
        *my_real = (float)raw_y / MAG_SENSITIVITY;
        *mz_real = (float)raw_z / MAG_SENSITIVITY; //单位为G

        // 3. 清除标志位，防止重复读取同一帧数据
        qmc5883_i2c_rx_done = 0;
        return 1;
    }
    return 0;
}

// 读ID (保留)
uint8_t QMC5883_GetID(void)
{
    uint8_t dev_id = 0;
    QMC5883_ReadByte(QMC5883_Chip_ID_Register, &dev_id);
    return dev_id;
}
