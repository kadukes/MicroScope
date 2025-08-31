/*
 * state.h
 *
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>
#include <stdbool.h>

#include "touch.h"

/* Exported types ------------------------------------------------------------*/
enum AppState
{
	PDSState                = 0b011010,
	TimeNoTriggerState      = 0b011001,
	TimeTriggerRisingState  = 0b010101,
	TimeTriggerFallingState = 0b100101,
};

extern uint8_t g_foundTrigger;

bool isTimeDomain(void);

bool isFrequencyDomain(void);

bool isTriggerEnabled(void);

bool isTriggerDisabled(void);

bool isTriggerRising(void);

bool isTriggerFalling(void);

void updateState(enum ClickAction action);


#endif /* INC_STATE_H_ */
