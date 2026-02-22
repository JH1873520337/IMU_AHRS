//
// Created by ASUS on 26-2-22.
//

#include "drv_uart1.h"

void Drv_UART_Init(void)
{
    // 这里的初始化通常在 main.c 的 MX_USART1_UART_Init 中完成了
    // 如果有额外的环形缓冲区初始化，写在这里
}

void Drv_UART_Transmit_DMA(uint8_t *data, uint16_t len)
{
    // 检查串口是否忙 (防止上一帧没发完就发下一帧，导致数据覆盖)
    // 实际工程中这里应该用环形缓冲区(RingBuffer)来缓冲，这里简化处理
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart1, data, len);
    }
    // HAL_UART_Transmit(&huart1, data, len, 10);
}