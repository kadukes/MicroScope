/*
 * touch.h
 *
 */

#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

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

/* Exported functions prototypes ---------------------------------------------*/
/**
 * Setup all necessary tasks for touch controller interaction
 */
void setupTouch();

/**
 * Handle a touch event
 */
void handleTouchInterrupt();


#endif /* INC_TOUCH_H_ */
