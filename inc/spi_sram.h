/*
 * spi_sram.h
 *
 *  Created on: 9. 10. 2019
 *      Author: Petr Dvorak
 */

#ifndef SPI_SRAM_H_
#define SPI_SRAM_H_

#include "stm32f10x.h"
#include "defs.h"
#include "stm32f10x_conf.h"
#include "application.h"

#define READCMD			0x03
#define FASTREADCMD		0x0B
#define FASTREAD4CMD	0xEB
#define WRITECMD		0x02
#define WRITE4CMD		0x38
#define ENTER4MODECMD	0x35
#define EXIT4MODECMD	0xF5
#define RESETENCMD		0x66
#define RESETCMD		0x99
#define BURSTMODECMD	0xC0
#define READIDCMD		0x9F
#define DUMMYCMD		0x00

void spi_sram_init(void);
void spi_sram_read(uint32_t addr, uint8_t *buffer, uint32_t bytes);
void spi_sram_write(uint32_t addr, uint8_t *buffer, uint32_t bytes);

#endif /* SPI_SRAM_H_ */
