/*
 * delay.c
 *
 *  Created on: Mar 8, 2014
 *      Author: daymoon
 */

#include "delay.h"

static uint8_t delay_finished;
static uint8_t Get_Delay_finished(void);

void delay_ms(uint16_t time)
{
	uint32_t time_calc;

	// aktivni cekani, preruseni povoleno, bezne tasky jsou zpracovavany dale
	Set_Delay_finished(DELAY_STARTED);

	time_calc = (SCHEDULERPERIOD * time)/1000;
	if(Scheduler_Add_Task(Delay_service, (uint16_t)time_calc, 0) == SCH_MAX_TASKS)
	{
		// chyba pri zalozeni service
	}
	while (Get_Delay_finished() != DELAY_FINISHED)	// cekej na dokonceni casovaciho tasku
	{
		Scheduler_Dispatch_Tasks();					// dotazuj nas vytvoreny task
	}
}

void Set_Delay_finished(uint8_t value)
{
	delay_finished = value;
}

static uint8_t Get_Delay_finished(void)
{
	return delay_finished;
}

