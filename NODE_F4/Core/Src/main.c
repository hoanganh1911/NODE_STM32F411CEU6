/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TFT.h"
#include "DS3231.h"
#include "../../lvgl/lvgl.h"
#include "../../lvgl/examples/lv_examples.h"
#include "lcd_lvgl.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId XuLy_Ban_PhimHandle;
osThreadId XuLy_LoraHandle;
osThreadId XuLy_Cap_NhatHandle;
osThreadId XuLy_MHCHandle;
osThreadId XuLy_MHSHandle;
osThreadId XuLy_MHLHandle;
osSemaphoreId Ngat_Nhan_Tu_BPHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void Start_XuLy_Ban_Phim(void const * argument);
void Start_XuLy_Lora(void const * argument);
void Start_XuLy_UPD(void const * argument);
void Start_XuLy_MHC(void const * argument);
void Start_XuLy_MHS(void const * argument);
void Start_XuLy_MHL(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}
void delay_ms(uint32_t time)
{
	for(int i = 0;i<1000;i++)
		delay_us(time);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == PCF_INT_Pin)
	{
		delay_us(10);
		if(HAL_GPIO_ReadPin(PCF_INT_GPIO_Port, PCF_INT_Pin) == 0)
		{
			osSemaphoreRelease(Ngat_Nhan_Tu_BPHandle);
		}
		delay_us(10);
	}
}
rtc_t time = {
		  .DayOfWeek = THURSDAY,
		  .Date 	 = 23,
		  .Month	 = 3,
		  .Hour		 = 17,
		  .Min	 	 = 10,
		  .Sec		 = 00,
		  .Year		 = 02,
		  .hi2c		 = &hi2c1
};
uint16_t ID = 0;
volatile uint8_t _lastkey;
bool _keypressed = false;
volatile bool _man_hinh_chinh 	= false;
volatile bool _man_hinh_setting	= false;
volatile bool _man_hinh_login	= false;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, 1);
  HAL_TIM_Base_Start(&htim1);
  lv_init();
  Display_init(3);
  HAL_Delay(500);
  lv_example_anim_2();
//  ID = readID();
//  HAL_Delay(100);
//  ds3231_setTime(&time);
//  tft_init(ID);
//  setRotation(3);
//  testFillScreen();
//  testLines(CYAN);
//  testFastLines(RED, BLUE);
//  testFilledCircles(10, MAGENTA);
//  testCircles(10, WHITE);
//  fillScreen(BLACK);
  _man_hinh_chinh = true; // Bật màn hình chính lúc đầu
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Ngat_Nhan_Tu_BP */
  osSemaphoreDef(Ngat_Nhan_Tu_BP);
  Ngat_Nhan_Tu_BPHandle = osSemaphoreCreate(osSemaphore(Ngat_Nhan_Tu_BP), 1);

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
  /* definition and creation of XuLy_Ban_Phim */
  osThreadDef(XuLy_Ban_Phim, Start_XuLy_Ban_Phim, osPriorityHigh, 0, 128);
  XuLy_Ban_PhimHandle = osThreadCreate(osThread(XuLy_Ban_Phim), NULL);

  /* definition and creation of XuLy_Lora */
  osThreadDef(XuLy_Lora, Start_XuLy_Lora, osPriorityHigh, 0, 128);
  XuLy_LoraHandle = osThreadCreate(osThread(XuLy_Lora), NULL);

  /* definition and creation of XuLy_Cap_Nhat */
  osThreadDef(XuLy_Cap_Nhat, Start_XuLy_UPD, osPriorityNormal, 0, 128);
  XuLy_Cap_NhatHandle = osThreadCreate(osThread(XuLy_Cap_Nhat), NULL);

  /* definition and creation of XuLy_MHC */
  osThreadDef(XuLy_MHC, Start_XuLy_MHC, osPriorityLow, 0, 128);
  XuLy_MHCHandle = osThreadCreate(osThread(XuLy_MHC), NULL);

  /* definition and creation of XuLy_MHS */
  osThreadDef(XuLy_MHS, Start_XuLy_MHS, osPriorityLow, 0, 128);
  XuLy_MHSHandle = osThreadCreate(osThread(XuLy_MHS), NULL);

  /* definition and creation of XuLy_MHL */
  osThreadDef(XuLy_MHL, Start_XuLy_MHL, osPriorityLow, 0, 128);
  XuLy_MHLHandle = osThreadCreate(osThread(XuLy_MHL), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_1_Pin|Relay_2_Pin|RS485_DERE_Pin|TFT_DB3_Pin
                          |TFT_DB2_Pin|TFT_DB1_Pin|TFT_DB0_Pin|TFT_LED_Pin
                          |TFT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_NSS_Pin|W25Q_CS_Pin|LORA_RST_Pin|TFT_DB7_Pin
                          |TFT_DB6_Pin|TFT_DB5_Pin|TFT_DB4_Pin|TFT_RD_Pin
                          |TFT_WR_Pin|TFT_RS_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_1_Pin Relay_2_Pin RS485_DERE_Pin TFT_DB3_Pin
                           TFT_DB2_Pin TFT_DB1_Pin TFT_DB0_Pin TFT_LED_Pin
                           TFT_RST_Pin */
  GPIO_InitStruct.Pin = Relay_1_Pin|Relay_2_Pin|RS485_DERE_Pin|TFT_DB3_Pin
                          |TFT_DB2_Pin|TFT_DB1_Pin|TFT_DB0_Pin|TFT_LED_Pin
                          |TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_NSS_Pin W25Q_CS_Pin LORA_RST_Pin TFT_DB7_Pin
                           TFT_DB6_Pin TFT_DB5_Pin TFT_DB4_Pin TFT_RD_Pin
                           TFT_WR_Pin TFT_RS_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin|W25Q_CS_Pin|LORA_RST_Pin|TFT_DB7_Pin
                          |TFT_DB6_Pin|TFT_DB5_Pin|TFT_DB4_Pin|TFT_RD_Pin
                          |TFT_WR_Pin|TFT_RS_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_INT_Pin */
  GPIO_InitStruct.Pin = LORA_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PCF_INT_Pin */
  GPIO_InitStruct.Pin = PCF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PCF_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_XuLy_Ban_Phim */
/**
  * @brief  Function implementing the XuLy_Ban_Phim thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_XuLy_Ban_Phim */
void Start_XuLy_Ban_Phim(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(Ngat_Nhan_Tu_BPHandle, osWaitForever);
	  _lastkey = getkey(&hi2c1);
	  if(_lastkey != 17)
	  {
		  if(_man_hinh_chinh == true)
		  	  {
		  		  if (_lastkey == 3)
		  		  {
		  			  _man_hinh_login = true;
		  			  _man_hinh_chinh = false;
		  		  }
		  		  else if (_lastkey == 7)
		  		  {
		  			  _man_hinh_setting = true;
		  			  _man_hinh_chinh = false;
		  		  }
		  	  }
		  	  else if(_man_hinh_setting == true)
		  	  {
		  		  if(_lastkey == 11)
		  		  {
		  			  _man_hinh_chinh = true;
		  			  _man_hinh_setting = false;

		  		  }
		  	  }
		  	  else if(_man_hinh_login == true)
		  	  {

		  		  if(_lastkey == 11)
		  		  {
		  			  _man_hinh_chinh = true;
		  			  _man_hinh_login = false;

		  		  }
		  	  }
	  }
	  else
	  {
		  fillScreen(BLACK);
		  _keypressed = true;
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_XuLy_Lora */
/**
* @brief Function implementing the XuLy_Lora thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_XuLy_Lora */
void Start_XuLy_Lora(void const * argument)
{
  /* USER CODE BEGIN Start_XuLy_Lora */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_XuLy_Lora */
}

/* USER CODE BEGIN Header_Start_XuLy_UPD */
/**
* @brief Function implementing the XuLy_Cap_Nhat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_XuLy_UPD */
void Start_XuLy_UPD(void const * argument)
{
  /* USER CODE BEGIN Start_XuLy_UPD */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_XuLy_UPD */
}

/* USER CODE BEGIN Header_Start_XuLy_MHC */
/**
* @brief Function implementing the XuLy_MHC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_XuLy_MHC */
void Start_XuLy_MHC(void const * argument)
{
  /* USER CODE BEGIN Start_XuLy_MHC */
  /* Infinite loop */
  for(;;)
  {
	  if(_man_hinh_chinh ==  true && _man_hinh_login == false && _man_hinh_setting == false && _keypressed == true)
	  {
		  _keypressed == false;
		  printnewtstr(10,RED, &mono9x7bold, 1, "Dang chay task XuLy MHC");

	  }
    osDelay(1);
  }
  /* USER CODE END Start_XuLy_MHC */
}

/* USER CODE BEGIN Header_Start_XuLy_MHS */
/**
* @brief Function implementing the XuLy_MHS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_XuLy_MHS */
void Start_XuLy_MHS(void const * argument)
{
  /* USER CODE BEGIN Start_XuLy_MHS */
  /* Infinite loop */
  for(;;)
  {
	  if(_man_hinh_setting ==  true && _man_hinh_chinh ==  false && _man_hinh_login ==  false && _keypressed == true)
	  {
		  _keypressed == false;
		  printnewtstr(10,RED, &mono9x7bold, 1, "Dang chay task XuLy MHS");
	  }
    osDelay(1);
  }
  /* USER CODE END Start_XuLy_MHS */
}

/* USER CODE BEGIN Header_Start_XuLy_MHL */
/**
* @brief Function implementing the XuLy_MHL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_XuLy_MHL */
void Start_XuLy_MHL(void const * argument)
{
  /* USER CODE BEGIN Start_XuLy_MHL */
  /* Infinite loop */
  for(;;)
  {
	  if(_man_hinh_login == true && _man_hinh_chinh ==  false && _man_hinh_setting == false && _keypressed == true)
	  {
		  _keypressed == false;
		  printnewtstr(10,RED, &mono9x7bold, 1, "Dang chay task XuLy MHL");
	  }
    osDelay(1);
  }
  /* USER CODE END Start_XuLy_MHL */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	lv_tick_inc(1);
	osSystickHandler();
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
