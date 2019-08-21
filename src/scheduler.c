#include "scheduler.h"
#include "errorcodes.h"

static sTask SCH_tasks_G[SCH_MAX_TASKS];					// The array of tasks
static uint8_t Error_code_G = 0;
//static uint16_t Error_tick_count_G;					// Keeps track of time since last error was recorded (see below)
//static uint8_t Last_error_code_G;					// The code of the last error (reset after ~1 minute)

void Scheduler_init(void)
{
	unsigned char i;
	for (i = 0; i < SCH_MAX_TASKS; i++)
	{
		Scheduler_Delete_Task(i);						// vymaz vsechny tasky
	}
	Error_code_G = 0;									// vynuluj chybovy registr
	
	// nastaveni systemoveho casovace
	if (SysTick_Config(SystemCoreClock / SCHEDULERPERIOD))
	{
		while (1);
	}
}
void Scheduler_start(void)
{
	// povoleni preruseni
	__enable_irq();
}
void Scheduler_Dispatch_Tasks(void)
{
	unsigned char Index;

	// Dispatches (runs) the next task (if one is ready)
	for (Index = 0; Index < SCH_MAX_TASKS; Index++)
	{
		if (SCH_tasks_G[Index].RunMe > 0)
		{
			SCH_tasks_G[Index].RunMe -= 1;   			// Reset / reduce RunMe flag

			(*SCH_tasks_G[Index].pTask)();  			// Run the task

			// Periodic tasks will automatically run again
			// - if this is a 'one shot' task, remove it from the array
			if (SCH_tasks_G[Index].Period == 0)
			{
				__disable_irq();
				Scheduler_Delete_Task(Index);
				__enable_irq();
			}
		}
	}
}

void Scheduler_Dispatch_Task_with_index(uint8_t index)
{
	// Dispatches (runs) the next task (if one is ready)
	if (SCH_tasks_G[index].RunMe > 0)
	{
		SCH_tasks_G[index].RunMe -= 1;   			// Reset / reduce RunMe flag
		(*SCH_tasks_G[index].pTask)();  			// Run the task

		// Periodic tasks will automatically run again
		// - if this is a 'one shot' task, remove it from the array
		if (SCH_tasks_G[index].Period == 0)
		{
			Scheduler_Delete_Task(index);
		}
	}
}

unsigned char Scheduler_Add_Task(void (*pFunction)(void), const unsigned int DELAY, const unsigned int PERIOD)
{
	unsigned char Index = 0;
	
	// First find a gap in the array (if there is one)
	while ((SCH_tasks_G[Index].pTask != 0) && (Index < SCH_MAX_TASKS))
	{
		Index++;
	}
	
	// Have we reached the end of the list?
	if (Index == SCH_MAX_TASKS)
	{
		// Task list is full
		//
		// Set the global error variable
		Error_code_G = ERROR_SCH_TOO_MANY_TASKS;

		// Also return an error code
		return SCH_MAX_TASKS;
	}
	
	// If we're here, there is a space in the task array
	SCH_tasks_G[Index].pTask  = pFunction;
	
	SCH_tasks_G[Index].Delay  = DELAY;
	SCH_tasks_G[Index].Period = PERIOD;

	SCH_tasks_G[Index].RunMe  = 0;

	return Index; // return position of task (to allow later deletion)
}
unsigned char Scheduler_Delete_Task(const unsigned char TASK_INDEX)
{
	unsigned char Return_code;

	if (SCH_tasks_G[TASK_INDEX].pTask == 0)
	{
		// No task at this location...
		//
		// Set the global error variable
		Error_code_G = ERROR_SCH_CANNOT_DELETE_TASK;

		// ...also return an error code
		Return_code = RETURN_ERROR;
	}
	else
	{
		Return_code = RETURN_NORMAL;
	}
	
	SCH_tasks_G[TASK_INDEX].pTask   = 0x0000;
	SCH_tasks_G[TASK_INDEX].Delay   = 0;
	SCH_tasks_G[TASK_INDEX].Period  = 0;
	SCH_tasks_G[TASK_INDEX].RunMe   = 0;

	return Return_code;       // return status
}

void Run_scheduler(void)
{
	uint8_t index;

	for (index = 0; index < SCH_MAX_TASKS; index++)
	{
		// Check if there is a task at this location
		if (SCH_tasks_G[index].pTask)
		{
			if (SCH_tasks_G[index].Delay == 0)
			{
				// The task is due to run
				SCH_tasks_G[index].RunMe += 1; // Inc. the 'RunMe' flag
				if (SCH_tasks_G[index].Period)
				{
					// Schedule regular tasks to run again
					SCH_tasks_G[index].Delay = SCH_tasks_G[index].Period;
				}
			}
			else
			{
				// Not yet ready to run: just decrement the delay
				SCH_tasks_G[index].Delay -= 1;
			}
		}
	}
}

void Scheduler_Refresh_task(const unsigned char TASK_INDEX)
{
	// if there is a task with this TASK_INDEX ID
	if (SCH_tasks_G[TASK_INDEX].pTask)
	{
		// Schedule task to run again
		SCH_tasks_G[TASK_INDEX].Delay = SCH_tasks_G[TASK_INDEX].Period;
		SCH_tasks_G[TASK_INDEX].RunMe = 0;
	}
}

