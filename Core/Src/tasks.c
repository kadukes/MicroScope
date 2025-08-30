/*
 * tasks.c
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "tasks.h"
#include "fft.h"
#include "plot.h"

/* Private variables ---------------------------------------------------------*/
osThreadId samplingTaskHandle;
osThreadId triggerTaskHandle;
osThreadId analysisTaskHandle;
osThreadId timeDomainVisuaHandle;
osThreadId pdsVisualizatioHandle;
osThreadId DSRTaskHandle;
osThreadId triggerVisualizHandle;
osMessageQId DSRQueueHandle;

static uint32_t s_time_domain[TIME_DOMAIN_LENGTH];
static uint32_t s_trigger[TIME_DOMAIN_LENGTH];
static float s_frequency_domain[PDS_LENGTH];
static uint8_t s_position = 0;
enum AppState g_state;
static uint8_t foundTrigger = 0;

/* Private function prototypes -----------------------------------------------*/
void startSamplingTask(void const * argument);
void startTriggerTask(void const * argument);
void startAnalysisTask(void const * argument);
void startTimeDomainVisualizationTask(void const * argument);
void startPdsVisualization(void const * argument);
void startDSRTask(void const * argument);
void startTriggerVisualization(void const * argument);

void setupTasks()
{
	g_state = TimeNoTriggerState;

	/* add mutexes, ... */
	/* add semaphores, ... */
	/* start timers, add new ones, ... */

	/* Create the queue(s) */
	/* definition and creation of DSRQueue */
	osMessageQDef(DSRQueue, 16, uint64_t);
	DSRQueueHandle = osMessageCreate(osMessageQ(DSRQueue), NULL);

	/* add queues, ... */

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

	/* definition and creation of timeDomainVisualization */
	osThreadDef(timeDomainVisualization, startTimeDomainVisualizationTask, osPriorityNormal, 0, 1024);
	timeDomainVisuaHandle = osThreadCreate(osThread(timeDomainVisualization), NULL);

	/* definition and creation of pdsVisualization */
	osThreadDef(pdsVisualization, startPdsVisualization, osPriorityBelowNormal, 0, 1024);
	pdsVisualizatioHandle = osThreadCreate(osThread(pdsVisualization), NULL);

	/* definition and creation of DSRTask */
	osThreadDef(DSRTask, startDSRTask, osPriorityAboveNormal, 0, 1024);
	DSRTaskHandle = osThreadCreate(osThread(DSRTask), NULL);

	/* definition and creation of triggerVisualization */
	osThreadDef(triggerVisualizationTask, startTriggerVisualization, osPriorityHigh, 0, 1024);
	triggerVisualizHandle = osThreadCreate(osThread(triggerVisualizationTask), NULL);

	/* add threads, ... */

	/* further initialization */
	plot_drawCoordSystem();
}


void handleTouchInterrupt()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TS_StateTypeDef ts;
    BSP_TS_GetState(&ts);
    xQueueSendFromISR(DSRQueueHandle, &ts, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Clear interrupt inside FT5336 controller
    BSP_TS_ITClear();
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


/**
  * @brief  Function implementing the samplingTask thread.
  * WOET: ~50µs
  * @param  argument: Not used
  * @retval None
  */
void startSamplingTask(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(4);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uint32_t sample = HAL_ADC_GetValue(&hadc1);
	  s_time_domain[s_position] = sample;
	  s_position = (s_position + 1) % TIME_DOMAIN_LENGTH;

	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


/**
* @brief Function implementing the triggerTask thread.
* WOET: ~50µs
* @param argument: Not used
* @retval None
*/
void startTriggerTask(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(4);
  TickType_t xLastWakeTime = xTaskGetTickCount();

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
}


/**
* @brief Function implementing the analysisTask thread.
* WOET: 5ms
* @param argument: Not used
* @retval None
*/
void startAnalysisTask(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	easy_pds(s_time_domain, s_frequency_domain, TIME_DOMAIN_LENGTH);
	vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


/**
* @brief Function implementing the timeDomainVisualization thread.
* WOET: 1ms
* @param argument: Not used
* @retval None
*/
void startTimeDomainVisualizationTask(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(250);
  TickType_t xLastWakeTime = xTaskGetTickCount();

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
}


/**
* @brief Function implementing the pdsVisualization thread.
* WOET: 1ms
* @param argument: Not used
* @retval None
*/
void startPdsVisualization(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	if (g_state & 0b000010)
	{
		plot_pds(s_frequency_domain, PDS_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


/**
* @brief Function implementing the DSRTask thread.
* @param argument: Not used
* @retval None
*/
void startDSRTask(void const * argument)
{
  TS_StateTypeDef ts;

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
	    	  plot_drawCoordSystem();
	    	  break;
	      case DisplayPDS:
	    	  g_state = (g_state | 0b000010) & ~0b000001;
	    	  foundTrigger = 0;
	    	  plot_drawCoordSystem();
	    	  break;
	      case TriggerOn:
          if (g_state & 0b000001)  // g_state display time
	    	  {
	    		  g_state = (g_state | 0b000100) & ~0b001000;
	    		  plot_drawCoordSystem();
	    	  }
	    	  break;
	      case TriggerOff:
	    	  if (g_state & 0b000001)  // g_state display time
	    	  {
	    		  g_state = (g_state | 0b001000) & ~0b000100;
	    		  foundTrigger = 0;
	    		  plot_drawCoordSystem();
	    	  }
	    	  break;
	      case TLevelRise:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b010000) & ~0b100000;
	    		  foundTrigger = 0;
	    		  plot_drawCoordSystem();
	    	  }
	    	  break;
	      case TLevelFall:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b100000) & ~0b010000;
	    		  foundTrigger = 0;
	    		  plot_drawCoordSystem();
	    	  }
	    	  break;
	      default:
	    	  break;
	      }
	    }
	  }
  }
}


/**
* @brief Function implementing the triggerVisualization thread.
* @param argument: Not used
* @retval None
*/
void startTriggerVisualization(void const * argument)
{
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
}
