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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t err_buf[12] = "Error\n\r";
uint8_t STH_BUF[8];
uint8_t *val;
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t dev_ctx_2;
static int wakeup_thread = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void); // IR Sensor 1
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void); // IR Sensor 2
static void MX_TIM3_Init(void); // IR Sensor 1 PWM
static void MX_I2C3_Init(void); // Arduino Ultra Sonic Sensor
static void MX_TIM8_Init(void); // IR Sensor 2 PWM
static void MX_TIM12_Init(void); // Arduino PWM
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

int16_t map(float x, float in_min, float in_max, float out_min, float out_max)
{
	//x = in_min || 0
	//x = 0 || 13
	//x = in_max || 999

  float y = (x - in_min) * (out_max) / (in_max - in_min);
  return y;
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
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	uint8_t whoami;
	sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

	uint8_t whoami_2;
	sths34pf80_lpf_bandwidth_t lpf_m_2, lpf_p_2, lpf_p_m_2, lpf_a_t_2;

	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.mdelay = platform_delay;
	dev_ctx.handle = &hi2c1;

	dev_ctx_2.write_reg = platform_write;
	dev_ctx_2.read_reg = platform_read;
	dev_ctx_2.mdelay = platform_delay;
	dev_ctx_2.handle = &hi2c2;

	/* Initialize platform specific hardware */

	// Set CS to High
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	sths34pf80_device_id_get(&dev_ctx, &whoami);
	sths34pf80_device_id_get(&dev_ctx_2, &whoami_2);

	while(whoami != STHS34PF80_ID){
		HAL_UART_Transmit(&huart2, err_buf, 12, 1000);
		HAL_Delay(1000);
		sths34pf80_device_id_get(&dev_ctx, &whoami);
	}
	while(whoami_2 != STHS34PF80_ID){
		HAL_UART_Transmit(&huart2, err_buf, 12, 1000);
		HAL_Delay(1000);
		sths34pf80_device_id_get(&dev_ctx_2, &whoami_2);
	}
	snprintf((char *)tx_buffer, sizeof(tx_buffer), "Device Found! WHO_AM_I: 0x%02X\r\n", whoami);
	tx_com(tx_buffer, strlen((char const *)tx_buffer));

	sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
	sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

	sths34pf80_avg_tobject_num_set(&dev_ctx_2, STHS34PF80_AVG_TMOS_32);
	sths34pf80_avg_tambient_num_set(&dev_ctx_2, STHS34PF80_AVG_T_8);

	/* read filters */
	sths34pf80_lpf_m_bandwidth_get(&dev_ctx, &lpf_m);
	sths34pf80_lpf_p_bandwidth_get(&dev_ctx, &lpf_p);
	sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx, &lpf_p_m);
	sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx, &lpf_a_t);

	sths34pf80_lpf_m_bandwidth_get(&dev_ctx_2, &lpf_m_2);
	sths34pf80_lpf_p_bandwidth_get(&dev_ctx_2, &lpf_p_2);
	sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx_2, &lpf_p_m_2);
	sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx_2, &lpf_a_t_2);

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

	sths34pf80_block_data_update_set(&dev_ctx_2, 1);

	sths34pf80_presence_threshold_set(&dev_ctx_2, 200);
	sths34pf80_presence_hysteresis_set(&dev_ctx_2, 20);
	sths34pf80_motion_threshold_set(&dev_ctx_2, 300);
	sths34pf80_motion_hysteresis_set(&dev_ctx_2, 30);

	sths34pf80_algo_reset(&dev_ctx_2);

	/* Set interrupt */
	sths34pf80_int_or_set(&dev_ctx, STHS34PF80_INT_PRESENCE);
	sths34pf80_route_int_set(&dev_ctx, STHS34PF80_INT_OR);

	sths34pf80_int_or_set(&dev_ctx_2, STHS34PF80_INT_PRESENCE);
	sths34pf80_route_int_set(&dev_ctx_2, STHS34PF80_INT_OR);

	/* Set ODR */
	sths34pf80_odr_set(&dev_ctx, STHS34PF80_ODR_AT_30Hz);
	sths34pf80_odr_set(&dev_ctx_2, STHS34PF80_ODR_AT_30Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	sths34pf80_func_status_t func_status;
	sths34pf80_func_status_t func_status_2;

	uint16_t buzzer_strength;
	uint16_t buzzer_strength_2;
	uint16_t buzzer_strength_3;

	int16_t object_temp_raw;
	int16_t ambient_temp_raw;
	float ambient_temp_celsius;

	int16_t object_temp_raw_2;
	int16_t ambient_temp_raw_2;
	float ambient_temp_celsius_2;

	uint8_t motion = 0;
	uint8_t presence = 0;

	uint8_t motion_2 = 0;
	uint8_t presence_2 = 0;

	float sensor_min = -50.0;  // Example: a reading below this is no one
	float sensor_max = 20000.0; // Example: a reading above this is a large crowd

	float sensor_min_2 = -50.0;  // Example: a reading below this is no one
	float sensor_max_2 = 20000.0; // Example: a reading above this is a large crowd

	float pwm_min = 0.0;       // 0% duty cycle
	float pwm_max = 999.0;     // Max duty cycle (matches the Counter Period)
	uint8_t counter = 0;
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

		  sths34pf80_func_status_get(&dev_ctx_2, &func_status_2);

		  sths34pf80_tobject_raw_get(&dev_ctx_2, &object_temp_raw_2);
		  sths34pf80_tambient_raw_get(&dev_ctx_2, &ambient_temp_raw_2);

		  ambient_temp_celsius = (float)ambient_temp_raw / 100.0f;
		  ambient_temp_celsius_2 = (float)ambient_temp_raw_2 / 100.0f;

		  // RIGHT SIDE SENSOR
		if (func_status.mot_flag != motion)
		  {
			motion = func_status.mot_flag;

			if (motion)
			{
			  snprintf((char *)tx_buffer, sizeof(tx_buffer), "\nMotion Detected!(Right Side)\r\n");
			  tx_com(tx_buffer, strlen((char const *)tx_buffer));

			}
		  }
		// LEFT SIDE SENSOR
		if (func_status_2.mot_flag != motion_2)
			  {
				motion_2 = func_status_2.mot_flag;

				if (motion_2)
				{
				  snprintf((char *)tx_buffer, sizeof(tx_buffer), "\nMotion Detected!(Left Side)\r\n");
				  tx_com(tx_buffer, strlen((char const *)tx_buffer));

				}
			  }

		  if (func_status.pres_flag != presence || func_status_2.pres_flag != presence_2)
		  {
			presence = func_status.pres_flag;
			presence_2 = func_status_2.pres_flag;
			 snprintf((char *)tx_buffer, sizeof(tx_buffer), "\n**Start of Presence**\r\n");
			 tx_com(tx_buffer, strlen((char const *)tx_buffer));
			while (presence || presence_2)
			{

			  buzzer_strength = map((float)object_temp_raw, sensor_min, sensor_max, pwm_min, pwm_max);
			  if (object_temp_raw < sensor_min) buzzer_strength = pwm_min;
			  if (object_temp_raw > sensor_max) buzzer_strength = pwm_max;

			  buzzer_strength = map((float)object_temp_raw_2, sensor_min_2, sensor_max_2, pwm_min, pwm_max);
			  if (object_temp_raw_2 < sensor_min_2) buzzer_strength_2 = pwm_min;
			  if (object_temp_raw_2 > sensor_max_2) buzzer_strength_2 = pwm_max;

			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, buzzer_strength);
			  platform_delay(BOOT_TIME);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, buzzer_strength_2);
			  platform_delay(BOOT_TIME);

				snprintf((char *)tx_buffer, sizeof(tx_buffer),
					  "--> Right Side IR Value: %d, Ambient Temp: %.2f C, Strength: %d\r\n",
					  object_temp_raw, ambient_temp_celsius, buzzer_strength);

				tx_com(tx_buffer, strlen((char const *)tx_buffer));

				snprintf((char *)tx_buffer, sizeof(tx_buffer),
					  "--> Left Side IR Value: %d, Ambient Temp: %.2f C, Strength: %d\r\n",
					  object_temp_raw_2, ambient_temp_celsius_2, buzzer_strength_2);

				tx_com(tx_buffer, strlen((char const *)tx_buffer));

				sths34pf80_func_status_get(&dev_ctx, &func_status);

			    sths34pf80_tobject_raw_get(&dev_ctx, &object_temp_raw);
			    sths34pf80_tambient_raw_get(&dev_ctx, &ambient_temp_raw);

			    ambient_temp_celsius = (float)ambient_temp_raw / 100.0f;

			    sths34pf80_func_status_get(&dev_ctx_2, &func_status_2);

				sths34pf80_tobject_raw_get(&dev_ctx_2, &object_temp_raw_2);
				sths34pf80_tambient_raw_get(&dev_ctx_2, &ambient_temp_raw_2);

				ambient_temp_celsius_2 = (float)ambient_temp_raw_2 / 100.0f;

			    if (counter < 50){
					  if (object_temp_raw < sensor_min) sensor_min = object_temp_raw;
					  if (object_temp_raw > sensor_max) sensor_max = object_temp_raw;

					  if (object_temp_raw_2 < sensor_min) sensor_min_2 = object_temp_raw_2;
					  if (object_temp_raw_2 > sensor_max) sensor_max_2 = object_temp_raw_2;

					  counter = counter + 1;
				  }
				  else {
					  if (object_temp_raw < sensor_min) sensor_min = -100.0;
					  if (object_temp_raw > sensor_max) sensor_max = 9999.0;

					  if (object_temp_raw_2 < sensor_min) sensor_min_2 = -100.0;
					  if (object_temp_raw_2 > sensor_max) sensor_max_2 = 9999.0;
					  counter = 0;
				  }
			    HAL_Delay(100);
			}
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			  platform_delay(BOOT_TIME);

			  snprintf((char *)tx_buffer, sizeof(tx_buffer), "\n**End of Presence**\r\n");
			  tx_com(tx_buffer, strlen((char const *)tx_buffer));
		  }
		} //while (func_status.pres_flag);
	} HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
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
  hi2c2.Init.OwnAddress1 = 180;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 174;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 63;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 63;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  HAL_GPIO_WritePin(CS__1_GPIO_Port, CS__1_Pin, GPIO_PIN_RESET);

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
  // Checks if the interrupt came from PIN A0
  if (GPIO_Pin == GPIO_PIN_0)
  {
    // INT Pin recieves signal
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
