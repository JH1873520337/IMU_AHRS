//
// Created by ASUS on 26-2-2.
//

#ifndef QMC5883_H
#define QMC5883_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stm32f4xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c3;
#define QMC5883_I2C_HANDLE    &hi2c3

#define QMC5883_I2C_ADDRESS  0X0D   // 7 位设备地址
#define QMC5883_W_ADDRESS    (QMC5883_I2C_ADDRESS << 1)  // 写地址：7位地址左移1位，最低位补0
#define QMC5883_R_ADDRESS    (QMC5883_I2C_ADDRESS << 1 | 1)  // 读地址：7位地址左移1位，最低位补1

#define QMC58883_DataOut_XLSB  0X00         //X轴低位数据
#define QMC58883_DataOut_XMSB  0X01         //x轴高位数据
#define QMC58883_DataOut_YLSB  0X02         //Y轴低位数据
#define QMC58883_DataOut_YMSB  0X03         //Y轴高位数据
#define QMC58883_DataOut_ZLSB  0X04         //Z轴低位数据
#define QMC58883_DataOut_ZMSB  0X05         //Z轴高位数据

#define QMC58883_Status_Register  0X06      //状态寄存器

#define QMC58883_Tout_LSB  0X07             //温度寄存器低位
#define QMC58883_Tout_MSB  0X08             //温度寄存器高位

#define QMC5883_Control_Registers1  0X09     //控制寄存器1
#define QMC5883_Control_Registers2  0X0A     //控制寄存器2

#define QMC5883_SET_RESET_Period_Register 0X0B  //设置/复位寄存器
#define QMC5883_Chip_ID_Register 0X0D           //设备ID寄存器


HAL_StatusTypeDef QMC5883_WriteByte(uint8_t RegAddress, uint8_t Data);
HAL_StatusTypeDef QMC5883_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size);
HAL_StatusTypeDef QMC5883_ReadByte(uint8_t RegAddress, uint8_t* Data);
HAL_StatusTypeDef QMC5883_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size);

void QMC5883_Init(void);
uint8_t QMC5883_GetID(void);
#endif //QMC5883_H
