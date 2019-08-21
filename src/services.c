/*
 * services.c
 *
 *  Created on: Dec 28, 2014
 *      Author: daymoon
 */

#include "services.h"

/* private prototypes */

void LED_service(void)
{
	uint16_t led_port;

	led_port = GPIO_ReadOutputData(LED_PORT);
	if (led_port & LED_PIN)
	{
		GPIO_ResetBits(LED_PORT, LED_PIN);				// zelena LED on
	}
	else
	{
		GPIO_SetBits(LED_PORT, LED_PIN);					// zelena LED off
	}
}

void Delay_service(void)
{
	Set_Delay_finished(DELAY_FINISHED);
}

// obsluzna sluzba zavolana z IRQ rutiny UART1
void Command_service(void)
{
	// dekodovani a zpracovani zpravy z UART
	commands_process();

	// uvolneni Rx bufferu
	usart_release_Rx_buffer();
}

// obsluzna sluzba heartbeat zprav
void Heartbeat_service(void)
{
//#define ALENABLED
#ifdef ALENABLED

	uint8_t text[] = "AL";

	// posli zpravu
	usart_send_text((uint8_t *)&text);				// odesli zpravu AL
	usart_newline();
#endif
}
