/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sths34pf80_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STHS34PF80_I2C_ADDRESS_7BIT   0x5A
#define VLX_ADDR_R 0x52

// VLX --------
#define MODEL_ID 0x010F // Value should be 0xEA
#define MODULE_TYPE 0x0110 // Value should be 0xAA
// VLX --------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    BOOT_TIME         10 //ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t err_buf[12] = "Error\n\r";
uint8_t STH_BUF[8];
uint8_t *val;
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx;
static int wakeup_thread = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	return HAL_I2C_Mem_Write(handle, (STHS34PF80_I2C_ADDRESS_7BIT << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
}
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	return HAL_I2C_Mem_Read(handle, (STHS34PF80_I2C_ADDRESS_7BIT << 1), reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}
static void platform_delay(uint32_t ms)
{
	return HAL_Delay(ms);
}
void sths34pf80_tmos_presence_detection_handler(void)
{
	wakeup_thread = 1;
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//HAL_StatusTypeDef res;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	uint8_t whoami;
	sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.mdelay = platform_delay;
	dev_ctx.handle = &hi2c1;

	/* Initialize platform specific hardware */

	// Set CS to High
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	sths34pf80_device_id_get(&dev_ctx, &whoami);

	while(whoami != STHS34PF80_ID){
		HAL_UART_Transmit(&huart2, err_buf, 12, 1000);
		HAL_Delay(1000);
		sths34pf80_device_id_get(&dev_ctx, &whoami);
	}
	snprintf((char *)tx_buffer, sizeof(tx_buffer), "Device Found! WHO_AM_I: 0x%02X\r\n", whoami);
	tx_com(tx_buffer, strlen((char const *)tx_buffer));

	sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
	sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

	/* read filters */
	sths34pf80_lpf_m_bandwidth_get(&dev_ctx, &lpf_m);
	sths34pf80_lpf_p_bandwidth_get(&dev_ctx, &lpf_p);
	sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx, &lpf_p_m);
	sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx, &lpf_a_t);

	snprintf((char *)tx_buffer, sizeof(tx_buffer),
			"lpf_m: %02d, lpf_p: %02d, lpf_p_m: %02d, lpf_a_t: %02d\r\n",
			 lpf_m, lpf_p, lpf_p_m, lpf_a_t);

	tx_com(tx_buffer, strlen((char const *)tx_buffer));

	/* Set BDU */
	sths34pf80_block_data_update_set(&dev_ctx, 1);

	sths34pf80_presence_threshold_set(&dev_ctx, 200);
	sths34pf80_presence_hysteresis_set(&dev_ctx, 20);
	sths34pf80_motion_threshold_set(&dev_ctx, 300);
	sths34pf80_motion_hysteresis_set(&dev_ctx, 30);

	sths34pf80_algo_reset(&dev_ctx);

	/* Set interrupt */
	sths34pf80_int_or_set(&dev_ctx, STHS34PF80_INT_PRESENCE);
	sths34pf80_route_int_set(&dev_ctx, STHS34PF80_INT_OR);

	/* Set ODR */
	sths34pf80_odr_set(&dev_ctx, STHS34PF80_ODR_AT_30Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	sths34pf80_func_status_t func_status;

	uint16_t buzzer_strength;

	int16_t object_temp_raw;
	int16_t ambient_temp_raw;
	float ambient_temp_celsius;

	uint8_t motion = 0;
	uint8_t presence = 0;

while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* handle event in a "thread" alike code */

	  if (wakeup_thread)
	  {
		wakeup_thread = 0;
		//motion = 0;
		//presence = 0;

		  sths34pf80_func_status_get(&dev_ctx, &func_status);

		  sths34pf80_tobject_raw_get(&dev_ctx, &object_temp_raw);
		  sths34pf80_tambient_raw_get(&dev_ctx, &ambient_temp_raw);

		  ambient_temp_celsius = (float)ambient_temp_raw / 100.0f;

		  	int16_t sensor_min = 200;  // Example: a reading below this is no one
			uint16_t sensor_max = 22000; // Example: a reading above this is a large crowd
			uint16_t pwm_min = 0;       // 0% duty cycle
			uint16_t pwm_max = 999;     // Max duty cycle (matches our Counter Period)

			/* BUZZER OFFSET FROM USART */
			if (func_status.pres_flag)
			{
			buzzer_strength = map(object_temp_raw, sensor_min, sensor_max, pwm_min, pwm_max);
			if (buzzer_strength < pwm_min) buzzer_strength = pwm_min;
			if (buzzer_strength > pwm_max) buzzer_strength = pwm_max;

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, buzzer_strength);
			platform_delay(BOOT_TIME);
			}
			else
			{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			platform_delay(BOOT_TIME);
			}
			/* BUZZER OFFSET FROM USART */

		  if (func_status.pres_flag != presence)
		  {
			presence = func_status.pres_flag;

			if (presence)
			{
			  snprintf((char *)tx_buffer, sizeof(tx_buffer), "Start of Presence\r\n");
			  tx_com(tx_buffer, strlen((char const *)tx_buffer));

				snprintf((char *)tx_buffer, sizeof(tx_buffer),
					  "--> IR Value: %d, Ambient Temp: %.2f C\r\n",
					  object_temp_raw, ambient_temp_celsius);

				tx_com(tx_buffer, strlen((char const *)tx_buffer));
			}
			else
			{
			  snprintf((char *)tx_buffer, sizeof(tx_buffer), "End of Presence\r\n");
			  tx_com(tx_buffer, strlen((char const *)tx_buffer));
			}
		  }

		  if (func_status.mot_flag != motion)
		  {
			motion = func_status.mot_flag;

			if (motion)
			{
			  snprintf((char *)tx_buffer, sizeof(tx_buffer), "Motion Detected!\r\n");
			  tx_com(tx_buffer, strlen((char const *)tx_buffer));

			}
		  }
		} //while (func_status.pres_flag);
	} HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 180;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 164;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS__1_GPIO_Port, CS__1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS__1_Pin */
  GPIO_InitStruct.Pin = CS__1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS__1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  External Line Detection Callback.
  * @param  GPIO_Pin: The Pin that triggered the interrupt.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // This line correctly checks if the interrupt came from PIN #0.
  if (GPIO_Pin == GPIO_PIN_0)
  {
    // Since you configured PA0 as EXTI0, this code will now run
    // when the sensor's INT pin sends a signal.
    wakeup_thread = 1;
  }
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
