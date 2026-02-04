//
// Created by ASUS on 26-2-2.
//

#include "QMC5883.h"
//写一个字节
HAL_StatusTypeDef QMC5883_WriteByte(uint8_t RegAddress, uint8_t Data)
{
    uint8_t tx_buf[2] = {RegAddress, Data};
    // HAL库I2C主发送：地址+寄存器+数据
    return HAL_I2C_Master_Transmit(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                                   tx_buf, 2, 100);  // 超时100ms
}

HAL_StatusTypeDef QMC5883_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size)
{
    uint8_t tx_buf[32];  // 最大支持32字节写入（可根据需求调整）
    if(Size > 30) return HAL_ERROR;  // 防止缓冲区溢出

    tx_buf[0] = RegAddress;  // 首字节为寄存器地址
    for(uint8_t i=0; i<Size; i++)
    {
        tx_buf[i+1] = Data[i];
    }
    return HAL_I2C_Master_Transmit(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                                   tx_buf, Size+1, 100);
}

HAL_StatusTypeDef QMC5883_ReadByte(uint8_t RegAddress, uint8_t* Data)
{
    // 1. 发送要读取的寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                                                       &RegAddress, 1, 100);
    if(status != HAL_OK) return status;

    // 2. 读取该寄存器的数据
    return HAL_I2C_Master_Receive(QMC5883_I2C_HANDLE, QMC5883_R_ADDRESS,
                                  Data, 1, 100);
}

/* 从QMC5883寄存器读多个字节 */
HAL_StatusTypeDef QMC5883_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size)
{
    // 1. 发送起始寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(QMC5883_I2C_HANDLE, QMC5883_W_ADDRESS,
                                                       &RegAddress, 1, 100);
    if(status != HAL_OK) return status;

    // 2. 连续读取多个字节
    return HAL_I2C_Master_Receive(QMC5883_I2C_HANDLE, QMC5883_R_ADDRESS,
                                  Data, Size, 100);
}

void QMC5883_Init(void)
{
    HAL_Delay(100);

    QMC5883_WriteByte(QMC5883_Control_Registers1, 0X0B);
    HAL_Delay(10);
    QMC5883_WriteByte(QMC5883_Control_Registers2, 0X08);
    HAL_Delay(10);

}

void QMC5883_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
    uint8_t magnetometer_data[6];
    // 读磁力计（0x01-0x06，6字节）
    QMC5883_ReadReg(QMC5883_DataOut_XLSB, magnetometer_data, 6);

    // 拼接加速度数据
    *MagX = (int16_t)((magnetometer_data[1] << 8) | magnetometer_data[0]);
    *MagY = (int16_t)((magnetometer_data[3] << 8) | magnetometer_data[2]);
    *MagZ = (int16_t)((magnetometer_data[5] << 8) | magnetometer_data[4]);
}

//获取测量的真实值，返回值单位为G
void QMC5883_GetRealData(float *mx_real, float *my_real, float *mz_real)
{
    int16_t Mx_raw, My_raw, Mz_raw;

    QMC5883_GetData(&Mx_raw, &My_raw, &Mz_raw);

    *mx_real = (float)Mx_raw/MAG_SENSITIVITY ;
    *my_real = (float)My_raw/MAG_SENSITIVITY ;
    *mz_real = (float)Mz_raw/MAG_SENSITIVITY ;

}


uint8_t QMC5883_GetID(void)
{
    uint8_t dev_id = 0;
    QMC5883_ReadByte(QMC5883_Chip_ID_Register, &dev_id);
    return dev_id;
}
