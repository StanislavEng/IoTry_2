/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static const uint8_t LSM6_ADDR = 0x6A << 1; // I'm remember what this is? I think this is the 1101010 in 1101 010x
static const uint8_t XALL_ADDR = 0x28; // next  8 bits (will I need this if I try to read 2 bytes?)
//static const uint8_t INT1_CTRL = 0x0D;
static uint8_t config = 0x60;	// 01100000; high speed
//static uint16_t xread;
//static uint16_t yread;
//static uint16_t zread;
//static uint16_t AccelVal[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
//void readXval(uint8_t* buf);
//void readYval(uint8_t* buf);
//void readZval(uint8_t* buf);
void LEDBlink();
//uint16_t * readAccel();
// in C you can't pass by reference
//void readAccel(uint16_t *retval); //
void readAccel(uint16_t *retval,char *retSi);
void LEDfunc(uint16_t *accVal, char *acSi);
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
		HAL_StatusTypeDef ret; // status of I2C commands
		uint8_t text[50] = "Hello!\r\n";  // text array for outputting in UART
		//uint8_t buf[6]; 			 // buffer for register values supposed to be
		//uint16_t val;  				 // the combined registers of x accelerometer value
		//float xval;
		uint16_t AccelVal[3];      // want an array
		char acSign[3];
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim7);
  HAL_Delay(15);

  // print out start message for debugging
  HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY);

  //// configure the I2C register for IMU
  ret = HAL_I2C_Mem_Write(&hi2c2, LSM6_ADDR, 0x10, 1, &config, 1, 50);
  if(ret != HAL_OK)
  {
		//return ret;
		strcpy((char*)text,"I2C Error\r\n");
		while (1){HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY);}
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  LEDBlink();
	  HAL_Delay(100);
	  readAccel(AccelVal,acSign);
	  HAL_Delay(100);
	  LEDfunc(AccelVal,acSign);
	  HAL_Delay(500);
//	  readXval(&buf);
//	  HAL_Delay(10);
//	  readYval(&buf);
//	  HAL_Delay(10);
//	  readZval(&buf);
//	  HAL_Delay(70);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8000 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 20000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|SPBTLE_RF_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STSAFE_A100_RESET_GPIO_Port, STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin SPBTLE_RF_RST_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|SPBTLE_RF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin SPSGRF_915_SDN_Pin */
  GPIO_InitStruct.Pin = ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM3MDL_DRDY_EXTI8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_WIFI__LED4_BLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STSAFE_A100_RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  // some overlap with timers prventing working
	if (htim == &htim6){
		uint8_t buff[] = "Hello World!!\r\n";
		HAL_UART_Transmit(&huart1, buff, sizeof(buff), 1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	if (htim == &htim7){
		readXval();
	}
}*/
void LEDBlink (){
	uint8_t buff[] = "\n\n\n\n\n\n\n\n\n\n\nNew Data Acquired: \r\n";
	HAL_UART_Transmit(&huart1, buff, sizeof(buff), 1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}
//void readAccel(uint16_t *retval){
void readAccel(uint16_t *retval, char *retSi){
//void readAccel(uint16_t ** retval){
	HAL_StatusTypeDef ret; // status of I2C commands
	uint8_t text[50];      // text array for outputting in UART
	uint8_t buf[6];        // takes in data from IMU
	uint16_t val;          // combined value
//	uint16_t retarr[3];
	// read values from I2C
	ret = HAL_I2C_Mem_Read(&hi2c2, LSM6_ADDR, XALL_ADDR, 1, &buf, 6, HAL_MAX_DELAY);
	if (ret != HAL_OK){  // if I2C fails
	  strcpy((char*)text,"ERROR RX1\r\n");
	  HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY);
	}

	//// combing upper and lower values /////
	val = (((uint16_t)buf[1] << 8) | buf[0]);                             // creates X values
	/////////////////////////////////////////
	// dealing with the two's complement and making printable string
	if ( val > 0x7FFF ) {
		val = ~val & 0x7FFF;
		sprintf((char*)text,"X value: -%u \r\n",((unsigned int)(val)));   // negative condition
		retSi[0] = '-';
	}
	else {
		sprintf((char*)text,"X value: %u \r\n",((unsigned int)(val)));    // not negative
		retSi[0] = '+';
	}
	retval[0] = val;
	HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY); // prints

	/////////////////////////////////////////
	val = (((uint16_t)buf[3] << 8) | buf[2]);                             // creates Y values
	/////////////////////////////////////////
	if ( val > 0x7FFF ) {
		val = ~val & 0x7FFF;
		sprintf((char*)text,"Y value: -%u \r\n",((unsigned int)(val)));
		retSi[1] = '-';
	}
	else {
		sprintf((char*)text,"Y value: %u \r\n",((unsigned int)(val)));
		retSi[1] = '+';
	}
	retval[1] = val;
	HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY);

	/////////////////////////////////////////
	val = (((uint16_t)buf[5] << 8) | buf[4]);                             // creates Z values
	/////////////////////////////////////////
	if ( val > 0x7FFF ) {
		val = ~val & 0x7FFF;
		sprintf((char*)text,"Z value: -%u \r\n",((unsigned int)(val)));
		retSi[2] = '-';
	}
	else {
//		sprintf((char*)text,"Z value: %u \r\n\n\n\n\n",((unsigned int)(val)));
		sprintf((char*)text,"Z value: %u \r\n",((unsigned int)(val)));
		retSi[2] = '+';
	}
	retval[2] = val;
	HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY); // prints
//	return &retarr;
}
//void LEDfunc(uint16_t *accVal){
void LEDfunc(uint16_t *accVal, char *acSi){
	uint8_t out[16];
	if (accVal[0] > 5000){
		//strcpy((char*)out,"X is pos");
		sprintf((char*)out,"X is %c%d\r\n",acSi[0],accVal[0]);
	}
	else{
		strcpy((char*)out,"X is low\r\n");
	}
	HAL_UART_Transmit(&huart1, out, strlen((char*)out), HAL_MAX_DELAY);
	if (accVal[1] > 5000){
		sprintf((char*)out,"Y is %c%d\r\n",acSi[1],accVal[1]);
	}
	else{
		strcpy((char*)out,"Y is low\r\n");
	}
	HAL_UART_Transmit(&huart1, out, strlen((char*)out), HAL_MAX_DELAY);
	if (accVal[2] > 5000){
		sprintf((char*)out,"Z is %c%d\r\n",acSi[2],accVal[2]);
	}
	else{
		strcpy((char*)out,"Z is low\r\n");
	}
	HAL_UART_Transmit(&huart1, out, strlen((char*)out), HAL_MAX_DELAY);


}

/*
void readXval(uint8_t *buf){

		HAL_StatusTypeDef ret; // status of I2C commands
		uint8_t text[50];  // text array for outputting in UART
		//uint8_t buf[6];
		uint16_t val;
		// read values from I2C
		ret = HAL_I2C_Mem_Read(&hi2c2, LSM6_ADDR, XALL_ADDR, 1, &buf, 6, HAL_MAX_DELAY); //
		if (ret != HAL_OK){  // if I2C fails
		  strcpy((char*)text,"ERROR RX1\r\n");
		  HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY);
		}

		// combing upper and lower values
		val = (((uint16_t)buf[1] << 8) | buf[0]);// / 0x4009;

		// dealing with the two's complement
		if ( val > 0x7FFF ) {
		//if ((val & 0x8000) == 0x800){
		  //val |= 0xF000;
			val = ~val & 0x7FFF;
		}
		//val = val/350;
		// create string value of combined accelerometer x value
		sprintf((char*)text,"X value: %u \r\n",((unsigned int)(val)));
		HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY); // prints


}
void readYval(uint8_t *buf){
	uint8_t text[50];  // text array for outputting in UART
	uint16_t val;

	val = (((uint16_t)buf[3] << 8) | buf[2]);// / 0x4009;

	// dealing with the two's complement
	if ( val > 0x7FFF ) {
	//if ((val & 0x8000) == 0x800){
	  //val |= 0xF000;
		val = ~val & 0x7FFF;
	}
	//val = val/350;
	// create string value of combined accelerometer x value
	sprintf((char*)text,"Y value: %u \r\n",((unsigned int)(val)));
	HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY); // prints
}
void readZval(uint8_t *buf){
	uint8_t text[50];  // text array for outputting in UART
	uint16_t val;

	val = (((uint16_t)buf[5] << 8) | buf[4]);// / 0x4009;

	// dealing with the two's complement
	if ( val > 0x7FFF ) {
	//if ((val & 0x8000) == 0x800){
	  //val |= 0xF000;
		val = ~val & 0x7FFF;
	}
	//val = val/350;
	// create string value of combined accelerometer x value
	sprintf((char*)text,"Z value: %u \r\n",((unsigned int)(val)));
	HAL_UART_Transmit(&huart1, text, strlen((char*)text), HAL_MAX_DELAY); // prints
}*/

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
