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
#include "event_groups.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

///// define TASK_DEBUG // в main.h, 109 строка

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 64 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 64 * 4
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for myEvent01_but */
osEventFlagsId_t myEvent01_butHandle;
const osEventFlagsAttr_t myEvent01_but_attributes = {
  .name = "myEvent01_but"
};
/* USER CODE BEGIN PV */

#define NUMBErOfMUSKS 4 // число битовых масок

volatile uint8_t threshold_uart_bitmaskbbbbllll[2] =
{ 0 }; // защелка(порог включения светодиода) и битовая маска, получаемые с uart ESP32. (из ISR -> в задачи 03 и 05)
uint8_t bitmaskbbbbllll[NUMBErOfMUSKS]; // битовые маски. (из задачи 03 -> в 02)
		// bbbb		llll
		// button№  led№
		// 1234 	1234

volatile uint8_t value = 0; // освещенность с ацп, передаваемая в задачу 05 и на uart в ESP32. (04 -> ISR и 05)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void Task01_Button(void *argument);
void Task02_Led(void *argument);
void Task03_Uart(void *argument);
void Task04_ADC(void *argument);
void Task05_Light(void *argument);

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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

#ifdef TASK_DEBUG
  assert_param(1);//#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#endif

  //int x = 1;
  //assert_param(x);//#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  // нужно самому написать в assert_failed: while(1){}

  //configASSERT(x);//#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
  // это использовать в задаче. если пошло не так, то запрет прерываний задач, и в бесконечный цикл

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */

#ifdef TASK_DEBUG
	uint32_t heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myTask01 */
  myTask01Handle = osThreadNew(Task01_Button, NULL, &myTask01_attributes);

#ifdef TASK_DEBUG
	uint32_t heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(Task02_Led, NULL, &myTask02_attributes);

#ifdef TASK_DEBUG
	uint32_t heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(Task03_Uart, NULL, &myTask03_attributes);

#ifdef TASK_DEBUG
	uint32_t heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(Task04_ADC, NULL, &myTask04_attributes);

#ifdef TASK_DEBUG
	uint32_t heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(Task05_Light, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

#ifdef TASK_DEBUG
	heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of myEvent01_but */
  myEvent01_butHandle = osEventFlagsNew(&myEvent01_but_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */

#ifdef TASK_DEBUG
	heap = xPortGetFreeHeapSize(); // Свободная память heap
#endif

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_NVIC_SystemReset();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_8;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|deb5_Pin|LD2_Pin|deb2_Pin
                          |led4_Pin|led1_Pin|deb1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, deb4_Pin|led3_Pin|deb3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, deb7_Pin|led2_Pin|deb6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin deb5_Pin LD2_Pin deb2_Pin
                           led4_Pin led1_Pin deb1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|deb5_Pin|LD2_Pin|deb2_Pin
                          |led4_Pin|led1_Pin|deb1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : deb4_Pin led3_Pin deb3_Pin */
  GPIO_InitStruct.Pin = deb4_Pin|led3_Pin|deb3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : but3_Pin but2_Pin but1_Pin */
  GPIO_InitStruct.Pin = but3_Pin|but2_Pin|but1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : deb7_Pin led2_Pin deb6_Pin */
  GPIO_InitStruct.Pin = deb7_Pin|led2_Pin|deb6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : but4_Pin */
  GPIO_InitStruct.Pin = but4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(but4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

///// HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // Сон, vApplicationIdleHook в app_freertos.c

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef TASK_DEBUG
	GPIOC->BSRR = GPIO_PIN_6; ///// Отладка. Вывод момента входа/выхода из прерывания на порты ввода/вывода МК
#endif

	HAL_UART_Transmit(&huart5, (uint8_t*) &value, sizeof(value), 0);
	huart->RxXferCount = 0U; // UART Rx Transfer Counter. Защита от глюков в Нуклео при внезапной перезагрузке ESP32
	HAL_UART_Receive_IT(&huart4, (uint8_t*) threshold_uart_bitmaskbbbbllll, 2);

	BaseType_t pxHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(myBinarySem01Handle, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); // вызываем запрос на досрочное (до очередного тика) переключения контекста на более приоритетную задачу Task03_Uart

#ifdef TASK_DEBUG
	GPIOC->BSRR = (uint32_t) GPIO_PIN_6 << 16U; ///// Отладка. Вывод момента входа/выхода из прерывания на порты ввода/вывода МК
#endif
}

///// Отладка. Вывод момента входа/выхода из задачи на порты ввода/вывода МК
#ifdef TASK_DEBUG

void TaskSwitchedIn(int tag)
{
	switch (tag)
	{
	case 1: // Task01
		GPIOA->BSRR = GPIO_PIN_10;
		break;
	case 2: // Task02
		GPIOA->BSRR = GPIO_PIN_6;
		break;
	case 3:
		GPIOB->BSRR = GPIO_PIN_9;
		break;
	case 4:
		GPIOB->BSRR = GPIO_PIN_0;
		break;
	case 5:
		GPIOA->BSRR = GPIO_PIN_4;
		break;
	default:
		;
	}
}

void TaskSwitchedOut(int tag)
{
	switch (tag)
	{
	case 1:
		GPIOA->BSRR = (uint32_t) GPIO_PIN_10 << 16U;
		break;
	case 2:
		GPIOA->BSRR = (uint32_t) GPIO_PIN_6 << 16U;
		break;
	case 3:
		GPIOB->BSRR = (uint32_t) GPIO_PIN_9 << 16U;
		break;
	case 4:
		GPIOB->BSRR = (uint32_t) GPIO_PIN_0 << 16U;
		break;
	case 5:
		GPIOA->BSRR = (uint32_t) GPIO_PIN_4 << 16U;
		break;
	default:
		;
	}
}

#endif

//#ifdef DEBUG
//#endif

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task01_Button */
/**
 * @brief  Function implementing the myTask01 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task01_Button */
void Task01_Button(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t ButCount1;
	uint8_t ButCount2;
	uint8_t ButCount3;
	uint8_t ButCount4;

#ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*) 1); ///// Отладка. Вывод момента входа/выхода из задачи на порты ввода/вывода МК
#endif

	/* Infinite loop */
	for (;;)
	{
		// Антидребезг
		ButCount1 = (ButCount1 << 1) + !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5); // исх = 1, нажали кнопку1 = 0
		ButCount2 = (ButCount2 << 1) + !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4); // исх = 1, нажали кнопку2 = 0
		ButCount3 = (ButCount3 << 1) + !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // исх = 1,нажали кнопку3 = 0
		ButCount4 = (ButCount4 << 1) + !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8); // исх = 1, нажали кнопку4 = 0

		if (ButCount1 == 0b00011111) // кнопка была нажата в течение 40 мс
		{
			xEventGroupSetBits(myEvent01_butHandle, 0x08); // 1000
		}
		if (ButCount2 == 0b00011111)
		{
			xEventGroupSetBits(myEvent01_butHandle, 0x04); // 0100
		}
		if (ButCount3 == 0b00011111)
		{
			xEventGroupSetBits(myEvent01_butHandle, 0x02); // 0010
		}
		if (ButCount4 == 0b00011111)
		{
			xEventGroupSetBits(myEvent01_butHandle, 0x01); // 0001
		}

		osDelay(10); // опрашиваем кнопки каждые 10 мс
	}
	vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task02_Led */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task02_Led */
void Task02_Led(void *argument)
{
  /* USER CODE BEGIN Task02_Led */

	int8_t number; // переменная для номера сработавшей битовой маски
	uint8_t copy_bitmaskbbbbllll; // переменная для битовой маски, чтобы не попортить
	EventBits_t cnt;

#ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*) 2);
#endif

	//bitmaskbbbbllll[0] = 0b10011001; // 1001 1001 BIN = 153 DEC
	//bitmaskbbbbllll[1] = 0b10000101; // 1000 0101 = 133
	//bitmaskbbbbllll[2] = 0b01000001; // 0100 0001 = 65
	//bitmaskbbbbllll[3] = 0b00110011; // 0011 0011 = 51

	/* Infinite loop */
	for (;;)
	{

		cnt = xEventGroupWaitBits(myEvent01_butHandle, 0b1111, pdTRUE, pdFALSE,
				0); // ждем нажатие любой кнопки
		number = 0;

		for (int i = 0; i < NUMBErOfMUSKS; i++)
		{
			copy_bitmaskbbbbllll = bitmaskbbbbllll[i]; // копируем 1-ую, 2, 3 или 4-ую битовые маски, полученные с сервера

			copy_bitmaskbbbbllll >>= 4; // выбираем 4 старших бита маски (соответствующей нажатым кнопкам)

			///// если нажатые клавиши совпадают с битовой маской и битовая маска не равна нулю, то включаем соответствующие леды

			//if ((cnt == (copy_bitmaskbbbbllll >>= 4)) && (copy_bitmaskbbbbllll)) //bbbb copy_bitmaskbbbbllll >> 4
			if (((cnt & copy_bitmaskbbbbllll) == copy_bitmaskbbbbllll)
					&& copy_bitmaskbbbbllll)
			{
				number = i + 1;
			}
		}

		if (number)
		{
			copy_bitmaskbbbbllll = bitmaskbbbbllll[number - 1];

			///// Разные варианты реализации
			//copy_bitmaskbbbbllll <<= 4;
			//copy_bitmaskbbbbllll >>= 7; // выбираем 4 младших бита битовой маски
			//copy_bitmaskbbbbllll &= 4; // выбираем 1-ый из 4-х младших бита битовой маски
			//HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, copy_bitmaskbbbbllll);
			//HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, (copy_bitmaskbbbbllll &= 4));

			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin,
					(copy_bitmaskbbbbllll & (1 << 3))); // выбираем 1-ый из 4-х младших бит битовой маски
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin,
					(copy_bitmaskbbbbllll & (1 << 2)));
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin,
					(copy_bitmaskbbbbllll & (1 << 1)));
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin,
					(copy_bitmaskbbbbllll & 1));
		}

		vTaskDelay(700); // для успевания нажатия сразу на несколько кнопок!
	}
	vTaskDelete(NULL);
  /* USER CODE END Task02_Led */
}

/* USER CODE BEGIN Header_Task03_Uart */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task03_Uart */
void Task03_Uart(void *argument)
{
  /* USER CODE BEGIN Task03_Uart */

	HAL_UART_Receive_IT(&huart4, (uint8_t*) threshold_uart_bitmaskbbbbllll, 2);

	uint8_t old_uart_bitmaskbbbbllll = 0;
	uint8_t counter = 0; // счетчик для записи маски с uart в маски локальные

#ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*) 3);
#endif

	/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY);

		if (threshold_uart_bitmaskbbbbllll[1] != old_uart_bitmaskbbbbllll)
		{
			bitmaskbbbbllll[counter] = threshold_uart_bitmaskbbbbllll[1];
			old_uart_bitmaskbbbbllll = threshold_uart_bitmaskbbbbllll[1];
			counter++;
		}
		if (counter == 4)
		{
			counter = 0;
		}

	}
	vTaskDelete(NULL);
  /* USER CODE END Task03_Uart */
}

/* USER CODE BEGIN Header_Task04_ADC */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task04_ADC */
void Task04_ADC(void *argument)
{
  /* USER CODE BEGIN Task04_ADC */

#ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*) 4);
#endif

	/* Infinite loop */
	for (;;)
	{
		//taskENTER_CRITICAL();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		//taskEXIT_CRITICAL();

		osDelay(20);
	}
	vTaskDelete(NULL);
  /* USER CODE END Task04_ADC */
}

/* USER CODE BEGIN Header_Task05_Light */
/**
 * @brief Function implementing the myTask05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task05_Light */
void Task05_Light(void *argument)
{
  /* USER CODE BEGIN Task05_Light */

	uint8_t threshold = 150;
	uint8_t old_threshold = 0;

#ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*) 5);
#endif

	/* Infinite loop */
	for (;;)
	{

		if (threshold_uart_bitmaskbbbbllll[0] != old_threshold)
		{
			threshold = threshold_uart_bitmaskbbbbllll[0];
			old_threshold = threshold_uart_bitmaskbbbbllll[0];
		}

		if (value > threshold) // освещенное помещение
		{
			HAL_GPIO_WritePin(GPIOA, LED_Pin, 0); // LED не горит
		}
		else // темное помещение
		{
			HAL_GPIO_WritePin(GPIOA, LED_Pin, 1); // LED горит
		}

		osDelay(20);
	}
	vTaskDelete(NULL);
  /* USER CODE END Task05_Light */
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
	while(1){}
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
