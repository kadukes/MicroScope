/*
 * tasks.c
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "tasks.h"
#include "fft.h"
#include "plot.h"
#include "touch.h"
#include "state.h"

/* Private variables ---------------------------------------------------------*/
osThreadId samplingTaskHandle;
osThreadId triggerTaskHandle;
osThreadId analysisTaskHandle;
osThreadId timeDomainVisualizationHandle;
osThreadId pdsVisualizationHandle;
osThreadId triggerVisualizationHandle;

static uint32_t s_time_domain[TIME_DOMAIN_LENGTH];
static uint32_t s_trigger[TIME_DOMAIN_LENGTH];
static float s_frequency_domain[PDS_LENGTH];
static uint8_t s_position = 0;

/* Private function prototypes -----------------------------------------------*/
void startSamplingTask(void const * argument);
void startTriggerTask(void const * argument);
void startAnalysisTask(void const * argument);
void startTimeDomainVisualizationTask(void const * argument);
void startPdsVisualization(void const * argument);
void startTriggerVisualization(void const * argument);

void setupTasks()
{
	/* add mutexes, ... */
	/* add semaphores, ... */
	/* start timers, add new ones, ... */
	/* add queues, ... */

	/* Create the thread(s) */
	/* definition and creation of samplingTask */
	osThreadDef(samplingTask, startSamplingTask, osPriorityRealtime, 0, 1024);
	samplingTaskHandle = osThreadCreate(osThread(samplingTask), NULL);

	/* definition and creation of triggerTask */
	osThreadDef(triggerTask, startTriggerTask, osPriorityHigh, 0, 1024);
	triggerTaskHandle = osThreadCreate(osThread(triggerTask), NULL);

	/* definition and creation of analysisTask */
	osThreadDef(analysisTask, startAnalysisTask, osPriorityNormal, 0, 1024);
	analysisTaskHandle = osThreadCreate(osThread(analysisTask), NULL);

	/* definition and creation of timeDomainVisualization */
	osThreadDef(timeDomainVisualization, startTimeDomainVisualizationTask, osPriorityNormal, 0, 1024);
	timeDomainVisualizationHandle = osThreadCreate(osThread(timeDomainVisualization), NULL);

	/* definition and creation of pdsVisualization */
	osThreadDef(pdsVisualization, startPdsVisualization, osPriorityNormal, 0, 1024);
	pdsVisualizationHandle = osThreadCreate(osThread(pdsVisualization), NULL);

	/* definition and creation of triggerVisualization */
	osThreadDef(triggerVisualizationTask, startTriggerVisualization, osPriorityHigh, 0, 1024);
	triggerVisualizationHandle = osThreadCreate(osThread(triggerVisualizationTask), NULL);

	/* add threads, ... */
}


/**
  * @brief  Function implementing the samplingTask thread.
  * WOET: ~50µs
  * @param  argument: Not used
  * @retval None
  */
void startSamplingTask(void const * argument)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(1);
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
  const TickType_t xPeriod = pdMS_TO_TICKS(1);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	uint8_t prev_pos = (s_position + TIME_DOMAIN_LENGTH - 32) % TIME_DOMAIN_LENGTH;
	uint8_t current_pos = (s_position + TIME_DOMAIN_LENGTH - 31) % TIME_DOMAIN_LENGTH;

	if (isTimeDomain() && isTriggerEnabled())  // display in time domain and trigger enabled
	{
		if (isTriggerRising() && s_time_domain[prev_pos] < TRIGGER_LEVEL && s_time_domain[current_pos] >= TRIGGER_LEVEL)
		{
			// rising edge
			g_foundTrigger = 1;
			xTaskNotifyGive(triggerVisualizationHandle);
		}
		else if (isTriggerFalling() && s_time_domain[prev_pos] > TRIGGER_LEVEL && s_time_domain[current_pos] <= TRIGGER_LEVEL)
		{
			// falling edge
			g_foundTrigger = 1;
			xTaskNotifyGive(triggerVisualizationHandle);
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
  const TickType_t xPeriod = pdMS_TO_TICKS(33);
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
  const TickType_t xPeriod = pdMS_TO_TICKS(33);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	if (isTriggerEnabled() && isTimeDomain() && g_foundTrigger)
	{
		plot(s_trigger, TIME_DOMAIN_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}
	else if (isTimeDomain() && !g_foundTrigger)
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
  const TickType_t xPeriod = pdMS_TO_TICKS(33);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	if (isFrequencyDomain())
	{
		plot_pds(s_frequency_domain, PDS_LENGTH, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	}

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
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
