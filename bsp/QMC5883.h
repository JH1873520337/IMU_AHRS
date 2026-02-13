#ifndef QMC5883_H
#define QMC5883_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 按需修改I2C实例（匹配硬件连接） */
#ifndef QMC5883_I2C_HANDLE
#define QMC5883_I2C_HANDLE    &hi2c3
#endif
extern I2C_HandleTypeDef hi2c3;

/* 灵敏度定义 */
#define MAG_SENSITIVITY       3750.0f  // ±8G量程对应灵敏度（基于PDF和用户配置）
#define QMC5883_DMA_TIMEOUT   100       // DMA传输超时时间(ms)

/* QMC5883 I2C地址（7位地址0x0D，左移1位+读写位） */
#define QMC5883_W_ADDRESS     0x1A  // 写地址 (0x0D << 1)
#define QMC5883_R_ADDRESS     0x1B  // 读地址 (0x0D << 1 | 0x01)

/* 寄存器地址（基于PDF） */
#define QMC5883_CHIP_ID       0x00  // 芯片ID寄存器，默认0x80
#define QMC5883_EXPECTED_ID   0x80  // 预期ID值（基于PDF）

#define QMC5883_X_LSB         0x01  // X轴低位字节
#define QMC5883_X_MSB         0x02  // X轴高位字节
#define QMC5883_Y_LSB         0x03  // Y轴低位字节
#define QMC5883_Y_MSB         0x04  // Y轴高位字节
#define QMC5883_Z_LSB         0x05  // Z轴低位字节
#define QMC5883_Z_MSB         0x06  // Z轴高位字节

#define QMC5883_STATUS        0x09  // 状态寄存器
#define QMC5883_CTRL1         0x0A  // 控制寄存器1 (MODE, ODR, OSR)
#define QMC5883_CTRL2         0x0B  // 控制寄存器2 (软复位, 自检, RNG, SET/RESET)

/* DMA传输状态标志位（外部可引用，用于调试） */
extern volatile uint8_t qmc5883_i2c_tx_done;
extern volatile uint8_t qmc5883_i2c_rx_done;

/* 函数声明 */
HAL_StatusTypeDef QMC5883_WriteByte(uint8_t RegAddress, uint8_t Data);
HAL_StatusTypeDef QMC5883_WriteReg(uint8_t RegAddress, const uint8_t* Data, uint8_t Size);
HAL_StatusTypeDef QMC5883_ReadByte(uint8_t RegAddress, uint8_t* Data);
HAL_StatusTypeDef QMC5883_ReadReg(uint8_t RegAddress, uint8_t* Data, uint8_t Size);

void QMC5883_Init(void);
uint8_t QMC5883_CheckID(void);  // 新增：检查设备ID是否匹配
void QMC5883_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);
void QMC5883_GetRealData(float *mx_real, float *my_real, float *mz_real);

#endif // QMC5883_H