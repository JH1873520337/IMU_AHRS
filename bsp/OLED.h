 //
// Created by ASUS on 26-2-1.
//
#ifndef OLED_H
#define OLED_H

#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "OLED_Font.h"
#include "stm32f4xx_hal_i2c.h"  // 引入硬件I2C头文件

#define OLED_I2C_HANDLE    &hi2c1    //移植时需要修改
// 声明硬件I2C句柄（与CubeMX生成的一致）
extern I2C_HandleTypeDef hi2c1;
// OLED I2C从机地址（7位地址0x3C → 写地址0x78，读地址0x79）
#define OLED_I2C_ADDR 0x78

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLen, uint8_t DecLen);
#endif //OLED_H