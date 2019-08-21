/*
 * leds.c
 *
 *  Created on: Dec 28, 2014
 *      Author: daymoon
 */

#include "leds.h"

void leds_config(void)
{
	// zalozeni ulohy blikani zelene led
	if(Scheduler_Add_Task(LED_service, 0, LED_SERVICE_PERIOD) == SCH_MAX_TASKS)
	{
		// chyba pri zalozeni service
	}
}
