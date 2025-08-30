/*
 * tasks.h
 *
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

/* Exported types ------------------------------------------------------------*/
enum ClickAction
{
	DisplayTime = (1 << 1),
	DisplayPDS  = (1 << 2),
	TriggerOn   = (1 << 3),
	TriggerOff  = (1 << 4),
	TLevelRise  = (1 << 5),
	TLevelFall  = (1 << 6),
	Invalid     = 0x00,
};

enum AppState
{
	PDSState                = 0b011010,
	TimeNoTriggerState      = 0b011001,
	TimeTriggerRisingState  = 0b010101,
	TimeTriggerFallingState = 0b100101,
};

extern enum AppState g_state;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * Setup all necessary tasks and other RTOS infrastructure
 */
void setupTasks();

/**
 * Handle a touch event
 */
void handleTouchInterrupt();

/* Private defines -----------------------------------------------------------*/
#define PDS_LENGTH 32
#define TIME_DOMAIN_LENGTH 64
#define TRIGGER_LEVEL 2048

#endif /* INC_TASKS_H_ */
