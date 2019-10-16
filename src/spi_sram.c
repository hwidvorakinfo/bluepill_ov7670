/*
 * spi_sram.c
 *
 *  Created on: 9. 10. 2019
 *      Author: Petr Dvorak
 */

#include "spi_sram.h"
#include "spi.h"
#include "delay.h"

void spi_sram_CS_low(void);
void spi_sram_CS_high(void);
void spi_sram_readid(void);
void spi_sram_reset(void);
void spi_sram_wait_if_busy(void);

void spi_sram_test(void)
{
	uint8_t pole[10];
	uint8_t *p_uint8;
	volatile uint8_t i;

	// naplneni pole
	for (i = 0; i < 10; i++)
	{
		pole[i] = 0xF0 + i;
	}
	p_uint8 = &pole[0];

	//delay_ms(1);

	spi_sram_write(0, p_uint8, 5);

	// vymazani pole
	for (i = 0; i < 10; i++)
	{
		pole[i] = 0;
	}
	p_uint8 = &pole[0];

	delay_ms(1);

	spi_sram_read(0, p_uint8, 5);

	// vymazani pole
	for (i = 0; i < 10; i++)
	{
		pole[i] = 0;
	}
}

void spi_sram_init(void)
{
	spi_sram_reset();
	spi_sram_readid();

	spi_sram_test();
}

void spi_sram_reset(void)
{
	spi_sram_CS_low();
	SPI_Write8(RESETENCMD);
	spi_sram_wait_if_busy();
	spi_sram_CS_high();

	spi_sram_CS_low();
	SPI_Write8(RESETCMD);
	spi_sram_wait_if_busy();
	spi_sram_CS_high();
}


void spi_sram_readid(void)
{
	volatile uint8_t reg;

	spi_sram_CS_low();
	SPI_Write8(READIDCMD);

	reg = SPI_Read8Write8(DUMMYCMD);
	reg = SPI_Read8Write8(DUMMYCMD);
	reg = SPI_Read8Write8(DUMMYCMD);

	reg = SPI_Read8Write8(DUMMYCMD);	// MF ID
	reg = SPI_Read8Write8(DUMMYCMD);	// KGD
	reg = SPI_Read8Write8(DUMMYCMD);	// dalsi byte

	/*
	reg = SPI_Read8Write8(DUMMYCMD);	// EID0
	reg = SPI_Read8Write8(DUMMYCMD);	// EID1
	reg = SPI_Read8Write8(DUMMYCMD);	// EID2
	reg = SPI_Read8Write8(DUMMYCMD);	// EID3
	reg = SPI_Read8Write8(DUMMYCMD);	// EID4
	reg = SPI_Read8Write8(DUMMYCMD);	// EID5
	*/

	spi_sram_wait_if_busy();
	spi_sram_CS_high();
}

void spi_sram_read(uint32_t addr, uint8_t *buffer, uint32_t bytes)
{
	spi_sram_CS_low();
	SPI_Read8Write8(READCMD);					// zapis prikazu pro cteni

	SPI_Write8((addr >> 16) & 0xFF);	// zapis adresy MSB
	SPI_Write8((addr >> 8) & 0xFF);
	SPI_Write8(addr & 0xFF);			// zapis adresy LSB

	while (bytes--)
	{
		//*buffer++ = SPI_Read8Write8(DUMMYCMD);		// cteni jednotlivych bytu
		*buffer++ = SPI_Read8();		// cteni jednotlivych bytu
	}
	//spi_sram_wait_if_busy();
	spi_sram_CS_high();
}

void spi_sram_write(uint32_t addr, uint8_t *buffer, uint32_t bytes)
{
	spi_sram_CS_low();
	SPI_Read8Write8(WRITECMD);				// zapis prikazu pro zapis

	SPI_Write8((addr >> 16) & 0xFF);	// zapis adresy MSB
	SPI_Write8((addr >> 8) & 0xFF);
	SPI_Write8(addr & 0xFF);			// zapis adresy LSB

	while (bytes--)
	{
		//SPI_Read8Write8(*buffer++); 			// zapis jednotlivych bytu
		SPI_Write8(*buffer++); 			// zapis jednotlivych bytu
		//spi_sram_wait_if_busy();
	}
	spi_sram_wait_if_busy();
	spi_sram_CS_high();
}

void spi_sram_CS_low(void)
{
	GPIO_ResetBits(SRAM_CS_PORT, SRAM_CS_PIN);			// CS low
}

void spi_sram_CS_high(void)
{
	GPIO_SetBits(SRAM_CS_PORT, SRAM_CS_PIN);			// CS high
}

void spi_sram_wait_if_busy(void)
{
	while ((LCD_SPI->SR & SPI_I2S_FLAG_BSY))
	{}
}
