/*
 * defs.h
 *
 *  Created on: Jan 1, 2015
 *      Author: daymoon
 */

#ifndef INCLUDES_DEFS_H_
#define INCLUDES_DEFS_H_

#define RESET 	0
#define SET 	!RESET
#define DISABLE 0
#define ENABLE 	!DISABLE
#define FALSE	0
#define TRUE	!FALSE
#define OK		RESET
#define NOK		!OK
#define LOW		0
#define HIGH	!LOW
#define OFF		0
#define ON		!OFF
#define VAR1	1
#define VAR2	2
#define VAR3	3

typedef void (*svcall_t)(void*);


#endif /* INCLUDES_DEFS_H_ */
