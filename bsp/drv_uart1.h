//
// Created by ASUS on 26-2-22.
//

#ifndef DRV_UART1_H
#define DRV_UART1_H
#include "stm32f4xx_hal.h"
#include <stdint.h>

// 定义使用的串口句柄 (假设是 USART1)
extern UART_HandleTypeDef huart1;

/**
 * @brief 初始化串口驱动 (配置DMA等)
 */
void Drv_UART_Init(void);

/**
 * @brief 发送数据 (非阻塞，使用DMA)
 * @param data 数据指针
 * @param len  长度
 */
void Drv_UART_Transmit_DMA(uint8_t *data, uint16_t len);

#endif //DRV_UART1_H
