//
// Created by ASUS on 26-2-1.
//

// ============== 重要：在第一个包含 OLED_Font.h 的地方定义这个宏 ==============
#define OLED_FONT_DEFINE
// ==========================================================================

#include "OLED.h"

/**
  * @brief  OLED写命令（硬件I2C版）
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
    // HAL_I2C_Mem_Write(句柄, 从机地址, 寄存器地址, 地址长度, 数据指针, 数据长度, 超时)
    // 0x00 = 命令寄存器，I2C_MEMADD_SIZE_8BIT=地址长度1字节
    HAL_I2C_Mem_Write(OLED_I2C_HANDLE, OLED_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &Command, 1, 100);
}

/**
  * @brief  OLED写数据（硬件I2C版）
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
    // 0x40 = 数据寄存器
    HAL_I2C_Mem_Write(OLED_I2C_HANDLE, OLED_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 行偏移（0~7），X 列偏移（0~127）
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					// 设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// 设置X高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			// 设置X低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示单个字符（8x16点阵）
  * @param  Line 行（1~4），Column 列（1~16），Char 待显示字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		// 上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	// 下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行，Column 起始列，String 字符串指针
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  次方辅助函数
  * @param  X 底数，Y 指数
  * @retval X^Y
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--) Result *= X;
	return Result;
}

/**
  * @brief  OLED显示无符号十进制数字
  * @param  Line/Column 位置，Number 数字，Length 显示长度
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示有符号十进制数字
  * @param  Line/Column 位置，Number 数字，Length 显示长度
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示十六进制数字
  * @param  Line/Column 位置，Number 数字，Length 显示长度
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		OLED_ShowChar(Line, Column + i, SingleNumber < 10 ? SingleNumber + '0' : SingleNumber - 10 + 'A');
	}
}

/**
  * @brief  OLED显示二进制数字
  * @param  Line/Column 位置，Number 数字，Length 显示长度
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化（硬件I2C版）
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint32_t i, j;

	// 上电延时（OLED要求）
	for (i = 0; i < 1000; i++)
		for (j = 0; j < 1000; j++);

	// 硬件I2C已由CubeMX的MX_I2C1_Init初始化，无需再初始化引脚

	// OLED寄存器配置（与原软件I2C一致）
	OLED_WriteCommand(0xAE);	// 关闭显示
	OLED_WriteCommand(0xD5);	// 时钟分频
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);	// 多路复用率
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);	// 显示偏移
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);	// 起始行
	OLED_WriteCommand(0xA1);	// 左右方向（0xA1正常）
	OLED_WriteCommand(0xC8);	// 上下方向（0xC8正常）
	OLED_WriteCommand(0xDA);	// COM引脚配置
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);	// 对比度
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);	// 预充电周期
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);	// VCOMH
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);	// 全局显示开启
	OLED_WriteCommand(0xA6);	// 正常显示（非反色）
	OLED_WriteCommand(0x8D);	// 充电泵使能
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0xAF);	// 开启显示

	OLED_Clear();				// 清屏
}