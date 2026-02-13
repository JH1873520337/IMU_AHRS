#ifndef __QMC5883_H
#define __QMC5883_H

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;
#define QMC5883_I2C_HANDLE    &hi2c3

#define MAG_SENSITIVITY    3750.0f

#define QMC5883_I2C_ADDRESS  0X2C   // 7 位设备地址
#define QMC5883_W_ADDRESS    (QMC5883_I2C_ADDRESS << 1)  // 写地址：7位地址左移1位，最低位补0
#define QMC5883_R_ADDRESS    (QMC5883_I2C_ADDRESS << 1 | 1)  // 读地址：7位地址左移1位，最低位补1

#define QMC5883_Chip_ID_Register 0X00           //设备ID寄存器

#define QMC5883_DataOut_XLSB  0X01         //X轴低位数据
#define QMC5883_DataOut_XMSB  0X02         //x轴高位数据
#define QMC5883_DataOut_YLSB  0X03         //Y轴低位数据
#define QMC5883_DataOut_YMSB  0X04         //Y轴高位数据
#define QMC5883_DataOut_ZLSB  0X05         //Z轴低位数据
#define QMC5883_DataOut_ZMSB  0X06         //Z轴高位数据

#define QMC5883_Status_Register  0X09      //状态寄存器

#define QMC5883_Control_Registers1  0X0A     //控制寄存器1
#define QMC5883_Control_Registers2  0X0B    //控制寄存器2

/* --- 新增 DMA 相关变量 --- */
extern volatile uint8_t qmc5883_i2c_rx_done; // 完成标志

/* --- 函数接口 --- */
// 1. 初始化 (保持阻塞，不用变)
void QMC5883_Init(void);

// 2. 发起 DMA 请求 (替代原来的 GetData 里的 I2C 部分)
void QMC5883_RequestData(void);

// 3. 解析数据 (替代原来的 GetData 里的移位拼接部分)
// 返回 1 表示更新成功，0 表示数据未准备好
uint8_t QMC5883_ParseData(float *mx_real, float *my_real, float *mz_real);

// 保持 ID 读取方便调试
uint8_t QMC5883_GetID(void);

#endif
