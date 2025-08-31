/*
 * tasks.h
 *
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

/* Exported functions prototypes ---------------------------------------------*/
/**
 * Setup all necessary tasks and other RTOS infrastructure
 */
void setupTasks();


/* Private defines -----------------------------------------------------------*/
#define PDS_LENGTH 32
#define TIME_DOMAIN_LENGTH 64
#define TRIGGER_LEVEL 2048

#endif /* INC_TASKS_H_ */
