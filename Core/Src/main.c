/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6050.h"
#include "QMC5883.h"
#include "quaternion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t mag_divider = 0;
#define MAG_DIV_RATIO 4  // MPU读4次，MAG读1次

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int8_t qmc_id,mpu_id;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();                    // 初始化OLED
  HAL_TIM_Base_Start_IT(&htim6);

  MPU6050_Init();
  QMC5883_Init();

  // 变量定义
  float ax, ay, az, gx, gy, gz;
  float mx, my, mz;
  qmc_id = QMC5883_GetID();
  mpu_id = MPU6050_GetID();
  // 状态标志
  uint8_t mpu_requested = 0;
  uint8_t mag_requested = 0;

  // 启动第一次请求
  MPU6050_RequestData();
  mpu_requested = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // === 任务1: 处理 MPU6050 (优先级高) ===
    if (mpu_requested && mpu6050_i2c_rx_done)
    {
      MPU6050_ParseData(&ax, &ay, &az, &gx, &gy, &gz);
      mpu_requested = 0; // 标记处理完成

      // 在这里进行姿态解算 (IMU Update)
      // MahonyAHRSupdateIMU(...)

      // 显示数据 (仅调试用，飞行时不要每圈都刷屏)
      OLED_ShowFloat(1, 1, az, 1, 2);
      OLED_ShowHexNum(4,1,mpu_id, 2);
      // === 调度逻辑 ===
      // 准备下一次读取
      // 这里可以加入简单的延时控制循环频率，或者使用定时器中断标志来触发
      HAL_Delay(2);

      // 决定是否读取磁力计 (分频处理)
      mag_divider++;
      if (mag_divider >= MAG_DIV_RATIO) {
        if (mag_requested == 0) { // 只有上一次已经读完了才发新请求
          QMC5883_RequestData();
          mag_requested = 1;
        }
        mag_divider = 0;
      }

      // 再次发起 MPU 读取 (高频连续)
      MPU6050_RequestData();
      mpu_requested = 1;
    }

    // === 任务2: 处理 QMC5883 (优先级低) ===
    // 注意：这里是并行的。I2C3 和 I2C2 的数据传输互不干扰
    if (mag_requested && qmc5883_i2c_rx_done)
    {
      QMC5883_ParseData(&mx, &my, &mz);

      // 处理数据 (打印或给 AHRS)
      // printf("Mag: %d, %d, %d\n", mx, my, mz);

      // === 重点 ===
      // 不要马上死循环狂发请求，控制一下频率
      // 或者在这里简单延时，或者用定时器标志
      HAL_Delay(10); // 例如 100Hz 采样

      // 发起下一次请求
      QMC5883_RequestData();
      OLED_ShowFloat(2,1, mx, 1, 2);
      OLED_ShowHexNum(3,1,qmc_id, 2);
    }

    // === 任务3: 其他逻辑 ===
    // 这里依然是非阻塞的！
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // 判断是TIM6的中断
  if(htim->Instance == TIM6)
  {

  }
  // 如果有TIM3中断，继续加else if(htim->Instance == TIM3)即可
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
