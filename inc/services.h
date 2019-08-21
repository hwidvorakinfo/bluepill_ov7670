/*
 * services.h
 *
 *  Created on: Dec 28, 2014
 *      Author: daymoon
 */

#ifndef INCLUDES_SERVICES_H_
#define INCLUDES_SERVICES_H_

#include "stm32f10x.h"
#include "delay.h"
#include "leds.h"
#include "scheduler.h"
#include "application.h"
//#include "usart.h"
//#include "commands.h"

#define MILISEKUND 	/1000

// periody jsou v milisekundach, neboli zakladni periode SysTick casovace
#define LED_SERVICE_PERIOD				(SCHEDULERPERIOD * 500 MILISEKUND)

#define FREE			0
#define STARTED			127
#define REQUESTED		64


// sluzby
void LED_service(void);
void Delay_service(void);

#endif /* INCLUDES_SERVICES_H_ */
