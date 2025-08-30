/*
 * plot.c
 *
 */

#include <assert.h>
#include "tasks.h"
#include "screen.h"
#include "plot.h"
#include "../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"

void plot_drawCoordSystem(void) {
	for (int y = 0; y < 320; ++y)
	{
		for (int x = 0; x < 240; ++x)
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
				if (g_state & 0b000001)
				{
					color = LCD_COLOR_LIGHTMAGENTA;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 5:  // pds box
				if (g_state & 0b000010)
				{
					color = LCD_COLOR_LIGHTMAGENTA;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 6:  // trigger box
				if (g_state & 0b000010)
				{
					color = LCD_COLOR_GRAY;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 7:  // on box
				if (g_state & 0b000010)
				{
					color = LCD_COLOR_GRAY;
				}
				else if (g_state & 0b000100)
				{
					color = LCD_COLOR_LIGHTMAGENTA;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 8:  // off box
				if (g_state & 0b000010)
				{
					color = LCD_COLOR_GRAY;
				}
				else if (g_state & 0b001000)
				{
					color = LCD_COLOR_LIGHTMAGENTA;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 9:  // tlevel box
				if (g_state & 0b000010 || g_state & 0b001000)
				{
					color = LCD_COLOR_GRAY;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 10:  // rise box
				if (g_state & 0b000010 || g_state & 0b001000)
				{
					color = LCD_COLOR_GRAY;
				}
				else if (g_state & 0b010000)
				{
					color = LCD_COLOR_LIGHTMAGENTA;
				}
				else
				{
					color = LCD_COLOR_BLUE;
				}
				break;
			case 11:  // fall box
				if (g_state & 0b000010 || g_state & 0b001000)
				{
					color = LCD_COLOR_GRAY;
				}
				else if (g_state & 0b100000)
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
	}
}

void plot(uint32_t data[], size_t length, uint32_t foreground,
		  uint32_t background) {
	plot_drawCoordSystem();
	BSP_LCD_SetTextColor(foreground);
    {
        size_t step = BSP_LCD_GetYSize() / length;
        for (size_t i = 1; i < length; ++i) {
            assert(data[i] < 4096);
            uint16_t y_prev = (((int) data[i-1]) * BSP_LCD_GetXSize()) / 4500;
            uint16_t y = (((int) data[i]) * BSP_LCD_GetXSize()) / 4500;

            BSP_LCD_DrawLine(y_prev, (i-1)*step, y, i*step);
        }
    }
}

void plot_pds(float spectrum[], size_t length, uint32_t foreground,
		      uint32_t background) {
    plot_drawCoordSystem();
    BSP_LCD_SetTextColor(foreground);
    {
      	size_t step = BSP_LCD_GetYSize() / length;
        for (size_t i = 1; i < length; ++i) {
          uint16_t y_prev = max(spectrum[i-1], -140.0f) + 140.0f;
          uint16_t y = max(spectrum[i], -140.0f) + 140.0f;
          BSP_LCD_DrawLine(y_prev, (i-1)*step, y, i*step);
        }
    }
}

