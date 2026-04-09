//
// Created by ASUS on 26-2-22.
//

#include "drv_uart1.h"

static volatile uint32_t uart_tx_req_cnt = 0;
static volatile uint32_t uart_tx_start_cnt = 0;
static volatile uint32_t uart_tx_drop_cnt = 0;

void Drv_UART_Init(void)
{
    // 这里的初始化通常在 main.c 的 MX_USART1_UART_Init 中完成了
    // 如果有额外的环形缓冲区初始化，写在这里
}

void Drv_UART_Transmit_DMA(uint8_t *data, uint16_t len)
{
    uart_tx_req_cnt++;

    // 检查串口是否忙 (防止上一帧没发完就发下一帧，导致数据覆盖)
    // 实际工程中这里应该用环形缓冲区(RingBuffer)来缓冲，这里简化处理
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Transmit_DMA(&huart1, data, len) == HAL_OK)
        {
            uart_tx_start_cnt++;
        }
        else
        {
            uart_tx_drop_cnt++;
        }
    }
    else
    {
        uart_tx_drop_cnt++;
    }
    // HAL_UART_Transmit(&huart1, data, len, 10);
}

void Drv_UART_GetStats(uint32_t *req, uint32_t *start, uint32_t *drop)
{
    if (req != 0)
    {
        *req = uart_tx_req_cnt;
    }
    if (start != 0)
    {
        *start = uart_tx_start_cnt;
    }
    if (drop != 0)
    {
        *drop = uart_tx_drop_cnt;
    }
}
