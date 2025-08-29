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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "fft.h"
#include "plot.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.h"
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

osThreadId samplingTaskHandle;
osThreadId triggerTaskHandle;
osThreadId analysisTaskHandle;
osThreadId timeDomainVisuaHandle;
osThreadId pdsVisualizatioHandle;
osThreadId DSRTaskHandle;
osThreadId triggerVisualizHandle;
osMessageQId DSRQueueHandle;
/* USER CODE BEGIN PV */
static uint32_t s_time_domain[TIME_DOMAIN_LENGTH];
static uint32_t s_trigger[TIME_DOMAIN_LENGTH];
static float s_frequency_domain[PDS_LENGTH];
static uint8_t s_position = 0;
enum AppState g_state;
static uint8_t foundTrigger = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
void startSamplingTask(void const * argument);
void startTriggerTask(void const * argument);
void startAnalysisTask(void const * argument);
void startTimeDomainVisualizationTask(void const * argument);
void startPdsVisualization(void const * argument);
void startDSRTask(void const * argument);
void startTriggerVisualization(void const * argument);

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);

  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_DisplayOn();

  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  BSP_TS_ITConfig();
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  g_state = TimeNoTriggerState;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of DSRQueue */
  osMessageQDef(DSRQueue, 16, uint64_t);
  DSRQueueHandle = osMessageCreate(osMessageQ(DSRQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of samplingTask */
  osThreadDef(samplingTask, startSamplingTask, osPriorityRealtime, 0, 1024);
  samplingTaskHandle = osThreadCreate(osThread(samplingTask), NULL);

  /* definition and creation of triggerTask */
  osThreadDef(triggerTask, startTriggerTask, osPriorityHigh, 0, 1024);
  triggerTaskHandle = osThreadCreate(osThread(triggerTask), NULL);

  /* definition and creation of analysisTask */
  osThreadDef(analysisTask, startAnalysisTask, osPriorityBelowNormal, 0, 1024);
  analysisTaskHandle = osThreadCreate(osThread(analysisTask), NULL);

  /* definition and creation of timeDomainVisua */
  osThreadDef(timeDomainVisua, startTimeDomainVisualizationTask, osPriorityNormal, 0, 1024);
  timeDomainVisuaHandle = osThreadCreate(osThread(timeDomainVisua), NULL);

  /* definition and creation of pdsVisualizatio */
  osThreadDef(pdsVisualizatio, startPdsVisualization, osPriorityBelowNormal, 0, 1024);
  pdsVisualizatioHandle = osThreadCreate(osThread(pdsVisualizatio), NULL);

  /* definition and creation of DSRTask */
  osThreadDef(DSRTask, startDSRTask, osPriorityAboveNormal, 0, 1024);
  DSRTaskHandle = osThreadCreate(osThread(DSRTask), NULL);

  /* definition and creation of triggerVisualiz */
  osThreadDef(triggerVisualizationTask, startTriggerVisualization, osPriorityHigh, 0, 1024);
  triggerVisualizHandle = osThreadCreate(osThread(triggerVisualizationTask), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  /** Initializes LTDC clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}


/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TP_INT1_Pin)
    {
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        TS_StateTypeDef ts;
        BSP_TS_GetState(&ts);
        xQueueSendFromISR(DSRQueueHandle, &ts, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        // Clear interrupt inside FT5336 controller
        BSP_TS_ITClear();
    }
}

enum ClickAction decode_click(uint16_t x, uint16_t y)
{
	if (x >= 220)
	{
		if (y >= 50 && y <= 85)
		{
			return DisplayTime;
		}
		if (y >= 85 && y <= 110)
		{
			return DisplayPDS;
		}
		if (y >= 165 && y <= 185)
		{
			return TriggerOn;
		}
		if (y >= 185 && y <= 210)
		{
			return TriggerOff;
		}
		if (y >= 260 && y <= 290)
		{
			return TLevelRise;
		}
		if (y >= 290)
		{
			return TLevelFall;
		}
	}
	return Invalid;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startSamplingTask */
/**
  * @brief  Function implementing the samplingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startSamplingTask */
void startSamplingTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  const TickType_t xPeriod = pdMS_TO_TICKS(4);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uint32_t sample = HAL_ADC_GetValue(&hadc1);
	  s_time_domain[s_position] = sample;
	  s_position = (s_position + 1) % TIME_DOMAIN_LENGTH;

	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTriggerTask */
/**
* @brief Function implementing the triggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTriggerTask */
void startTriggerTask(void const * argument)
{
  /* USER CODE BEGIN startTriggerTask */
  const TickType_t xPeriod = pdMS_TO_TICKS(4);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	uint8_t prev_pos = (s_position + TIME_DOMAIN_LENGTH - 32) % TIME_DOMAIN_LENGTH;
	uint8_t current_pos = (s_position + TIME_DOMAIN_LENGTH - 31) % TIME_DOMAIN_LENGTH;

	if (g_state & 0b000001 && g_state & 0b000100)  // display in time domain and trigger enabled
	{
		if (g_state & 0b010000 && s_time_domain[prev_pos] < TRIGGER_LEVEL && s_time_domain[current_pos] >= TRIGGER_LEVEL)
		{
			// rising edge
			foundTrigger = 1;
			xTaskNotifyGive(triggerVisualizHandle);
		}
		else if (g_state & 0b100000 && s_time_domain[prev_pos] > TRIGGER_LEVEL && s_time_domain[current_pos] <= TRIGGER_LEVEL)
		{
			// falling edge
			foundTrigger = 1;
			xTaskNotifyGive(triggerVisualizHandle);
		}
	}
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END startTriggerTask */
}

/* USER CODE BEGIN Header_startAnalysisTask */
/**
* @brief Function implementing the analysisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startAnalysisTask */
void startAnalysisTask(void const * argument)
{
  /* USER CODE BEGIN startAnalysisTask */
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	easy_pds(s_time_domain, s_frequency_domain, TIME_DOMAIN_LENGTH);
	vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END startAnalysisTask */
}

/* USER CODE BEGIN Header_startTimeDomainVisualizationTask */
/**
* @brief Function implementing the timeDomainVisua thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTimeDomainVisualizationTask */
void startTimeDomainVisualizationTask(void const * argument)
{
  /* USER CODE BEGIN startTimeDomainVisualizationTask */
  const TickType_t xPeriod = pdMS_TO_TICKS(250);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	if (g_state & 0b000100 && g_state & 0b000001 && foundTrigger)
	{
		plot(s_trigger, TIME_DOMAIN_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}
	else if (g_state & 0b000001 && !foundTrigger)
	{
		plot(s_time_domain, TIME_DOMAIN_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END startTimeDomainVisualizationTask */
}

/* USER CODE BEGIN Header_startPdsVisualization */
/**
* @brief Function implementing the pdsVisualizatio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPdsVisualization */
void startPdsVisualization(void const * argument)
{
  /* USER CODE BEGIN startPdsVisualization */
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	if (g_state & 0b000010)
	{
		plot_pds(s_frequency_domain, PDS_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END startPdsVisualization */
}

/* USER CODE BEGIN Header_startDSRTask */
/**
* @brief Function implementing the DSRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDSRTask */
void startDSRTask(void const * argument)
{
  /* USER CODE BEGIN startDSRTask */
  TS_StateTypeDef ts;
  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(DSRQueueHandle, &ts, portMAX_DELAY) == pdTRUE)
	  {
		if (ts.TouchDetected)
	    {
	      enum ClickAction a = decode_click(ts.X, 320 - ts.Y);
	      switch (a)
	      {
	      case DisplayTime:
          g_state = (g_state | 0b000001) & ~0b000010;
	    	  break;
	      case DisplayPDS:
	    	  g_state = (g_state | 0b000010) & ~0b000001;
	    	  foundTrigger = 0;
	    	  break;
	      case TriggerOn:
          if (g_state & 0b000001)  // g_state display time
	    	  {
	    		  g_state = (g_state | 0b000100) & ~0b001000;
	    	  }
	    	  break;
	      case TriggerOff:
	    	  if (g_state & 0b000001)  // g_state display time
	    	  {
	    		  g_state = (g_state | 0b001000) & ~0b000100;
	    		  foundTrigger = 0;
	    	  }
	    	  break;
	      case TLevelRise:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b010000) & ~0b100000;
	    		  foundTrigger = 0;
	    	  }
	    	  break;
	      case TLevelFall:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b100000) & ~0b010000;
	    		  foundTrigger = 0;
	    	  }
	    	  break;
	      default:
	    	  break;
	      }
	    }
	  }
  }
  /* USER CODE END startDSRTask */
}

/* USER CODE BEGIN Header_startTriggerVisualization */
/**
* @brief Function implementing the triggerVisualiz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTriggerVisualization */
void startTriggerVisualization(void const * argument)
{
  /* USER CODE BEGIN startTriggerVisualization */
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    size_t trigger_pos = (s_position + TIME_DOMAIN_LENGTH - 31) % TIME_DOMAIN_LENGTH;
	size_t center = TIME_DOMAIN_LENGTH / 2;
	for (size_t i = 0; i < TIME_DOMAIN_LENGTH; ++i)
	{
		size_t src_idx = (trigger_pos + i - center + TIME_DOMAIN_LENGTH) % TIME_DOMAIN_LENGTH;
		s_trigger[i] = s_time_domain[src_idx];
	}
  }
  /* USER CODE END startTriggerVisualization */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
