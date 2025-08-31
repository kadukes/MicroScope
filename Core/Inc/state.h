/*
 * state.h
 *
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>

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

void setupState(void);

uint8_t isTimeDomain(void);

uint8_t isFrequencyDomain(void);

uint8_t isTriggerEnabled(void);

uint8_t isTriggerDisabled(void);

uint8_t isTriggerRising(void);

uint8_t isTriggerFalling(void);

void updateState(enum ClickAction action);


#endif /* INC_STATE_H_ */
