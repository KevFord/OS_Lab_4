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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "semphr.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId defaultTaskHandle;
osThreadId blinkyTaskHandle;
osThreadId ReadADCHandle;
osThreadId RGB_LedHandle;
osThreadId WriteLCDHandle;
/* USER CODE BEGIN PV */
uint16_t ADC_Data; // 3.1.3

TextLCDType lcd;

char g_str_top[16] = "ADC Value:      ";
char g_str_btm[16];

SemaphoreHandle_t adcMutex;
SemaphoreHandle_t strMutex;

enum color_state { RED, BLUE, YELLOW, GREEN, OFF };
enum color_state state;

TickType_t lcd_diff;
TickType_t rgb_diff;
TickType_t default_diff;
TickType_t adc_diff;
TickType_t blink_diff;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void StartBlinkyTask(void const * argument);
void StartTaskADC(void const * argument);
void StartTaskRGB(void const * argument);
void StartTaskWriteLCD(void const * argument);

/* USER CODE BEGIN PFP */
void setRGB(uint8_t r, uint8_t g, uint8_t b); // 3.2.3
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setRGB(uint8_t r, uint8_t g, uint8_t b){ // 3.2.1/2

	r = r ? GPIO_PIN_SET : GPIO_PIN_RESET; // ternary
	g = g ? GPIO_PIN_SET : GPIO_PIN_RESET;
	b = b ? GPIO_PIN_SET : GPIO_PIN_RESET;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, r);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, g);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, b);
}
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
  vTraceEnable(TRC_START);
  chn0 = xTraceRegisterString("chn0");
  adc_ch = xTraceRegisterString("ADC:"); // 3.1.5
  vTracePrintF(chn0, "I am tracing channel %d", 0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  TextLCD_Init(&lcd, &hi2c1, 0x4E); // "startar" LCD
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  adcMutex = xSemaphoreCreateMutex();
  strMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of blinkyTask */
  osThreadDef(blinkyTask, StartBlinkyTask, osPriorityNormal, 0, 128);
  blinkyTaskHandle = osThreadCreate(osThread(blinkyTask), NULL);

  /* definition and creation of ReadADC */
  osThreadDef(ReadADC, StartTaskADC, osPriorityNormal, 0, 128);
  ReadADCHandle = osThreadCreate(osThread(ReadADC), NULL);

  /* definition and creation of RGB_Led */
  osThreadDef(RGB_Led, StartTaskRGB, osPriorityNormal, 0, 128);
  RGB_LedHandle = osThreadCreate(osThread(RGB_Led), NULL);

  /* definition and creation of WriteLCD */
  osThreadDef(WriteLCD, StartTaskWriteLCD, osPriorityLow, 0, 128);
  WriteLCDHandle = osThreadCreate(osThread(WriteLCD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	TickType_t default_start;
	TickType_t default_stop;
  /* Infinite loop */
  for(;;)
  {
	  default_start = xTaskGetTickCount();
	  osDelay(1);
	  default_stop = xTaskGetTickCount();
	  default_diff = default_stop - default_start;
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlinkyTask */
/**
* @brief Function implementing the blinkyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkyTask */
void StartBlinkyTask(void const * argument)
{
  /* USER CODE BEGIN StartBlinkyTask */
	TickType_t blink_start;
	TickType_t blink_stop;
	/* Infinite loop */
	for(;;)
	{
//		void vTaskFunction( void * pvParameters )
//		 {
//		 // Block for 500ms.
//		 const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
//
//			 for( ;; )
//			 {
//				 // Simply toggle the LED every 500ms, blocking between each toggle.
//				 vToggleLED();
//				 vTaskDelay( xDelay );
//			 }
//		 }
		blink_start = xTaskGetTickCount();
		vTaskDelay(500);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		blink_stop = xTaskGetTickCount();
		blink_diff = blink_stop - blink_start;
	}
  /* USER CODE END StartBlinkyTask */
}

/* USER CODE BEGIN Header_StartTaskADC */
/**
* @brief Function implementing the ReadADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskADC */
void StartTaskADC(void const * argument) // 3.1.4
{
  /* USER CODE BEGIN StartTaskADC */
	TickType_t adc_start;
	TickType_t adc_stop;
  /* Infinite loop */
  for(;;)
  {
	  adc_start = xTaskGetTickCount();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  if( xSemaphoreTake( adcMutex, ( TickType_t ) 10 ) == pdTRUE ) // Thread safety. 3.3
	  {
		  ADC_Data = HAL_ADC_GetValue(&hadc1); // Write data to the buffer.
	      xSemaphoreGive( adcMutex );
	  }
	  HAL_ADC_Stop(&hadc1);

	  char adc_str[30];
	  sprintf(adc_str, "ADC Value: % 4d", ADC_Data);

	  if( xSemaphoreTake( strMutex, ( TickType_t ) 10 ) == pdTRUE ) // Thread safety. 3.3
	  {
		  sprintf(g_str_btm, "%04d           ", ADC_Data);
		  xSemaphoreGive( strMutex );
	  }

	  vTracePrintF(adc_ch, adc_str, 0); // Tracing purposes. 3.1.6
	  vTaskDelay(100);
	  adc_stop = xTaskGetTickCount();
	  adc_diff = adc_stop - adc_start;
  }
  /* USER CODE END StartTaskADC */
}
/* USER CODE BEGIN Header_StartTaskRGB */
/**
* @brief Function implementing the RGB_Led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRGB */
void StartTaskRGB(void const * argument) // 3.2. Working as intended:)
{
  /* USER CODE BEGIN StartTaskRGB */
	TickType_t wakeup_time;
	const TickType_t periodTime = 150;
	TickType_t last_red_tick = 0;
	TickType_t last_blue_tick = 0;

	TickType_t rgb_start;
	TickType_t rgb_stop;

	wakeup_time = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  rgb_start = xTaskGetTickCount();
	  uint16_t ADC_DataLocal; // Local copy of the global ADC value.
	  if( xSemaphoreTake( adcMutex, ( TickType_t ) 10 ) == pdTRUE ) // Thread safety, 3.3
	  {
		  ADC_DataLocal = ADC_Data; // Local copy of the global ADC value.
		  xSemaphoreGive( adcMutex );
	  }
	  // Naive first pass, just set the color states as per 3.2.5:
	  if(ADC_DataLocal > 3000)
	  {
		  state = RED; // Set color. 3.2.5

		  last_red_tick = wakeup_time + 1400; // wakeup_time is reset after every iteration of the loop.
		  last_blue_tick = last_red_tick + 600; // Basically wakeup_time + 2k.
	  }
	  else if(ADC_DataLocal > 2000)
	  {
		  state = YELLOW;
	  }
	  else if(ADC_DataLocal > 1000)
	  {
		  state = GREEN;
	  }
	  else
	  {
		  state = OFF;
	  }
	  // Check if the timing has changed, is it time to delay the color state changes.
	  if(last_blue_tick > wakeup_time)
	  {
		  state = BLUE;
		  if(last_red_tick > wakeup_time)
		  {
			  state = RED;
		  }
	  }

	  switch (state)
	  {
	  case RED :
		  setRGB(1, 0, 0);
		  break;
	  case BLUE :
		  setRGB(0, 0, 1);
		  break;
	  case YELLOW :
		  setRGB(1, 1, 0);
		  break;
	  case GREEN :
		  setRGB(0, 1, 0);
		  break;
	  case OFF :
		  setRGB(0, 0, 0);
		  break;
	  }

	  vTaskDelayUntil(&wakeup_time, periodTime); // Set period to 150 ticks.
	  rgb_stop = xTaskGetTickCount();
	  rgb_diff = rgb_stop - rgb_start;
  }
  /* USER CODE END StartTaskRGB */
}

/* USER CODE BEGIN Header_StartTaskWriteLCD */
/**
* @brief Function implementing the WriteLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskWriteLCD */
void StartTaskWriteLCD(void const * argument) // OK.
{
  /* USER CODE BEGIN StartTaskWriteLCD */

	TickType_t LCD_wakeup_time;
	const TickType_t periodTime = 700;

	TickType_t lcd_start;
	TickType_t lcd_stop;

	char hello[16] = "Hello           ";
	char world[16] = "Word!           ";

	char str_top[16];
	char str_btm[16];

	LCD_wakeup_time = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  lcd_start = xTaskGetTickCount();

//	  if( xSemaphoreTake( strMutex, ( TickType_t ) 10 ) == pdTRUE ) // Thread safety, 3.3
//	  {
//		  for (int i = 0; i < 16; i++)
//		  {
//			  str_top[i] = g_str_top[i];
//			  str_btm[i] = g_str_btm[i];
//		  }
//		  xSemaphoreGive( strMutex );
//	  }

	  TextLCD_Home(&lcd);
	  TextLCD_Puts(&lcd, hello); //str_top);
	  TextLCD_Position(&lcd, 0, 1); // Second row, first character.
	  TextLCD_Puts(&lcd, world); //str_btm);

	  vTaskDelayUntil(&LCD_wakeup_time, periodTime); // Swap string every 700 ticks.

	  TextLCD_Home(&lcd);
	  TextLCD_Puts(&lcd, world); //str_btm);
	  TextLCD_Position(&lcd, 0, 1); // Second row, first character.
	  TextLCD_Puts(&lcd, hello); //str_top);

	  vTaskDelayUntil(&LCD_wakeup_time, periodTime); // Swap string every 700 ticks.
	  lcd_stop = xTaskGetTickCount();
	  lcd_diff = lcd_stop - lcd_start;
  }
  /* USER CODE END StartTaskWriteLCD */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
