/*
 * services.c
 *
 *  Created on: Dec 28, 2014
 *      Author: daymoon
 */

#include "services.h"
#include "ili9341.h"

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


void camera_service(void)
{
	camera_service_handler();
}

void button_service(void)
{
	if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN) == FALSE)
	{
		// tlacitko je porad stisknuto
		if ( (camera_get_detection_status() == TRUE) || (camera_get_detected_status() == TRUE) )
		{
			// detekce bezi nebo bylo zdetekovano, timto stiskem ji deaktivuj
			camera_set_detection_status(FALSE);
			camera_set_detected_status(FALSE);

			ili_display_clear(ILI9341_BLACK);
		}
		else
		{
			// detekce nebezi, aktivuj ji a uloz snimek
			camera_set_detection_status(TRUE);
			// uloz vzorek obrazku
			//camera_store_sample();
		}
	}
}
