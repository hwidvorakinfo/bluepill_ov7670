#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include "stm32f10x.h"

#define SCH_MAX_TASKS		30
#define SCHEDULERPERIOD		1000

extern void Scheduler_Dispatch_Tasks(void);
extern void Scheduler_Dispatch_Task_with_index(uint8_t index);
extern unsigned char Scheduler_Add_Task(void (*pFunction)(void), const unsigned int DELAY, const unsigned int PERIOD);
extern unsigned char Scheduler_Delete_Task(const unsigned char TASK_INDEX);
extern void Scheduler_init(void);
extern void Scheduler_start(void);
extern void Run_scheduler(void);
extern void Scheduler_Refresh_task(const unsigned char TASK_INDEX);

// Total memory per task is 7 bytes
typedef struct
{
	void (*pTask)(void);						// Pointer to the task (must be a 'void (void)' function)
	uint16_t Delay;								// Delay (ticks) until the function will (next) be run
	uint16_t Period;							// Interval (ticks) between subsequent runs.
	uint8_t RunMe;								// Incremented (by scheduler) when task is due to execute
} sTask;


#endif
