/*
 * plot.c
 *
 */

#include <assert.h>
#include "tasks.h"
#include "screen.h"
#include "plot.h"
#include "touch.h"
#include "state.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"

void clearPixel(uint16_t x, uint16_t y)
{
	uint8_t color_key = background_screen[y * 240 + x];
	uint32_t color = 0;
	switch (color_key)
	{
	case 0:  // Background
		color = LCD_COLOR_BLACK;
		break;
	case 1:  // Grid
		color = LCD_COLOR_GRAY;
		break;
	case 2:  // Text
		color = LCD_COLOR_WHITE;
		break;
	case 3:  // display box
		color = LCD_COLOR_BLUE;
		break;
	case 4:  // signal box
		if (isTimeDomain())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 5:  // pds box
		if (isFrequencyDomain())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 6:  // trigger box
		if (isFrequencyDomain())
		{
			color = LCD_COLOR_GRAY;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 7:  // on box
		if (isFrequencyDomain())
		{
			color = LCD_COLOR_GRAY;
		}
		else if (isTriggerEnabled())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 8:  // off box
		if (isFrequencyDomain())
		{
			color = LCD_COLOR_GRAY;
		}
		else if (isTriggerDisabled())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 9:  // tlevel box
		if (isFrequencyDomain() || isTriggerDisabled())
		{
			color = LCD_COLOR_GRAY;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 10:  // rise box
		if (isFrequencyDomain() || isTriggerDisabled())
		{
			color = LCD_COLOR_GRAY;
		}
		else if (isTriggerRising())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	case 11:  // fall box
		if (isFrequencyDomain() || isTriggerDisabled())
		{
			color = LCD_COLOR_GRAY;
		}
		else if (isTriggerFalling())
		{
			color = LCD_COLOR_LIGHTMAGENTA;
		}
		else
		{
			color = LCD_COLOR_BLUE;
		}
		break;
	default:
		color = LCD_COLOR_RED;
	}
	BSP_LCD_DrawPixel(x, y, color);
}

void plot_drawCoordSystem(void) {
	for (uint16_t x = 0; x < 240; ++x)
	{
		for (uint16_t y = 0; y < 320; ++y)
		{
			clearPixel(x, y);
		}
	}
}

void plot_clearPlot(void) {
	for (uint16_t i = 0; i < plotPixelCount; ++i)
	{
		uint16_t x = plotPixels[i].X;
		uint16_t y = plotPixels[i].Y;
		clearPixel(x, y);
	}
	plotPixelCount = 0;
}

void plot(uint32_t data[], size_t length, uint32_t foreground,
		  uint32_t background) {
	plot_clearPlot();
	BSP_LCD_SetTextColor(foreground);
    {
        size_t step = BSP_LCD_GetYSize() / length;
        for (size_t i = 1; i < length; ++i) {
            assert(data[i] < 4096);
            uint16_t y_prev = (((int) data[i-1]) * BSP_LCD_GetXSize()) / 4500;
            uint16_t y = (((int) data[i]) * BSP_LCD_GetXSize()) / 4500;

            BSP_LCD_DrawLineUpdatingBuffer(y_prev, (i-1)*step, y, i*step);
        }
    }
}

void plot_pds(float spectrum[], size_t length, uint32_t foreground,
		      uint32_t background) {
	plot_clearPlot();
    BSP_LCD_SetTextColor(foreground);
    {
      	size_t step = BSP_LCD_GetYSize() / length;
        for (size_t i = 1; i < length; ++i) {
          uint16_t y_prev = max(spectrum[i-1], -140.0f) + 140.0f;
          uint16_t y = max(spectrum[i], -140.0f) + 140.0f;
          BSP_LCD_DrawLineUpdatingBuffer(y_prev, (i-1)*step, y, i*step);
        }
    }
}

