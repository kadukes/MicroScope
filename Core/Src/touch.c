/*
 * touch.c
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.h"

/* Private includes ----------------------------------------------------------*/
#include "touch.h"
#include "plot.h"

/* Private variables ---------------------------------------------------------*/
osThreadId DSRTaskHandle;
osMessageQId DSRQueueHandle;
enum AppState g_state;
uint8_t g_foundTrigger = 0;

/* Private function prototypes -----------------------------------------------*/
void startDSRTask(void const * argument);

void setupTouch()
{
	g_state = TimeNoTriggerState;

	/* Create the queue(s) */
	/* definition and creation of DSRQueue */
	osMessageQDef(DSRQueue, 16, sizeof(TS_StateTypeDef));
	DSRQueueHandle = osMessageCreate(osMessageQ(DSRQueue), NULL);

	/* definition and creation of DSRTask */
	osThreadDef(DSRTask, startDSRTask, osPriorityAboveNormal, 0, 1024);
	DSRTaskHandle = osThreadCreate(osThread(DSRTask), NULL);
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
	      enum ClickAction a = decode_click(ts.X, BSP_LCD_GetYSize() - ts.Y);
	      enum AppState prevState = g_state;
	      switch (a)
	      {
	      case DisplayTime:
	    	  g_state = (g_state | 0b000001) & ~0b000010;
	    	  break;
	      case DisplayPDS:
	    	  g_state = (g_state | 0b000010) & ~0b000001;
	    	  g_foundTrigger = 0;
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
	    		  g_foundTrigger = 0;
	    	  }
	    	  break;
	      case TLevelRise:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b010000) & ~0b100000;
	    		  g_foundTrigger = 0;
	    	  }
	    	  break;
	      case TLevelFall:
	    	  if (g_state & 0b000001 && g_state & 0b000100)  // g_state display time with trigger
	    	  {
	    		  g_state = (g_state | 0b100000) & ~0b010000;
	    		  g_foundTrigger = 0;
	    	  }
	    	  break;
	      default:
	    	  break;
	      }

	      if (prevState != g_state)
	      {
	    	  plot_drawCoordSystem();
	      }
	    }
	  }
  }
}

