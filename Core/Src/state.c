/*
 * state.c
 *
 */

#include "state.h"
#include "plot.h"

/* Private variables ---------------------------------------------------------*/
enum AppState state;
uint8_t g_foundTrigger = 0;

void setupState(void)
{
	state = TimeNoTriggerState;
}

bool isTimeDomain(void)
{
	return state & 0b000001;
}

bool isFrequencyDomain(void)
{
	return state & 0b000010;
}

bool isTriggerEnabled(void)
{
	return state & 0b000100;
}

bool isTriggerDisabled(void)
{
	return state & 0b001000;
}

bool isTriggerRising(void)
{
	return state & 0b010000;
}

bool isTriggerFalling(void)
{
	return state & 0b100000;
}

void setTimeDomain(void)
{
	state = (state | 0b000001) & ~0b000010;
}

void setFrequencyDomain(void)
{
	state = (state | 0b000010) & ~0b000001;
}

void setTriggerOn(void)
{
	state = (state | 0b000100) & ~0b001000;
}

void setTriggerOff(void)
{
	state = (state | 0b001000) & ~0b000100;
}

void setTriggerRising(void)
{
	state = (state | 0b010000) & ~0b100000;
}

void setTriggerFalling(void)
{
	state = (state | 0b100000) & ~0b010000;
}

void updateState(enum ClickAction action)
{
	enum AppState prevState = state;
	switch (action)
	{
	case DisplayTime:
		setTimeDomain();
		break;
	case DisplayPDS:
		setFrequencyDomain();
		g_foundTrigger = 0;
		break;
	case TriggerOn:
		if (isTimeDomain())
		{
			setTriggerOn();
		}
		break;
	case TriggerOff:
		if (isTimeDomain())
		{
			setTriggerOff();
		    g_foundTrigger = 0;
		}
		break;
	case TLevelRise:
		if (isTimeDomain() && isTriggerEnabled())
		{
			setTriggerRising();
		    g_foundTrigger = 0;
		}
		break;
	case TLevelFall:
		if (isTimeDomain() && isTriggerEnabled())
		{
			setTriggerFalling();
		    g_foundTrigger = 0;
		}
		break;
	default:
		break;
	}

	if (prevState != state)
	{
		plot_drawCoordSystem();
	}
}
