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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AHRS.h"
#include "ahrs_mw.h"
#include "telemetry.h"
#include "OLED.h"
#include "quaternion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RAD_TO_DEG 57.2957795f
#define VOFA_SELFTEST 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AHRS_State_t ahrs;
IMU_Data_t imu_data;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
#if VOFA_SELFTEST
  OLED_ShowString(1, 1, "VOFA TEST");
  OLED_ShowString(2, 1, "CH1 +1.0");
  OLED_ShowString(3, 1, "CH2 +2.0");
  OLED_ShowString(4, 1, "CH3 +3.0");

  uint32_t last_tx_tick = HAL_GetTick();
  uint32_t last_led_tick = last_tx_tick;
  uint32_t tx_count = 0;
#else
  AHRS_MW_Init(); // 初始化传感器
  AHRS_Init(&ahrs); // 初始化算法

  OLED_ShowString(1, 1, "VOFA RUN");

  uint32_t imu_tick = HAL_GetTick();
  uint32_t last_req_tick = imu_tick;
  uint32_t last_oled_tick = imu_tick;
  uint32_t last_telemetry_tick = imu_tick;
  uint16_t led_div = 0;

  AHRS_MW_RequestData();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if VOFA_SELFTEST
    uint32_t now = HAL_GetTick();

    if ((now - last_tx_tick) >= 10U)
    {
      last_tx_tick = now;
      tx_count++;
      Telemetry_SendAttitude(1.0f, 2.0f, 3.0f);
    }

    if ((now - last_led_tick) >= 250U)
    {
      last_led_tick = now;
      HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
      OLED_ShowNum(1, 11, tx_count % 100000U, 5);
    }
#else
    uint32_t now;

    I2C_ServiceRecover();
    now = HAL_GetTick();

    if (!AHRS_MW_IsDataReady())
    {
      if ((now - last_req_tick) >= 1U)
      {
        AHRS_MW_RequestData();
        last_req_tick = now;
      }
    }

    if (AHRS_MW_IsDataReady())
    {
      float dt = (now - imu_tick) * 0.001f;
      imu_tick = now;

      if (dt <= 0.0f)
      {
        dt = 0.001f;
      }
      if (dt > 0.02f)
      {
        dt = 0.02f;
      }

      if (AHRS_MW_GetData(&imu_data))
      {
        AHRS_Update(&ahrs, &imu_data, dt);
        AHRS_GetEuler(&ahrs);

        led_div++;

        if (led_div >= 250U)
        {
          led_div = 0;
          HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
        }
      }

      AHRS_MW_RequestData();
      last_req_tick = now;
    }

    if ((now - last_telemetry_tick) >= 10U)
    {
      last_telemetry_tick = now;
      Telemetry_SendAttitude(
          ahrs.roll * RAD_TO_DEG,
          ahrs.pitch * RAD_TO_DEG,
          ahrs.yaw * RAD_TO_DEG
      );
    }

    if ((now - last_oled_tick) >= 50U)
    {
      OLED_ShowString(2, 1, "R");
      OLED_ShowString(3, 1, "P");
      OLED_ShowString(4, 1, "Y");
      OLED_ShowFloat(2, 2, ahrs.roll * RAD_TO_DEG, 3, 1);
      OLED_ShowFloat(3, 2, ahrs.pitch * RAD_TO_DEG, 3, 1);
      OLED_ShowFloat(4, 2, ahrs.yaw * RAD_TO_DEG, 3, 1);

      last_oled_tick = now;
    }
#endif
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
