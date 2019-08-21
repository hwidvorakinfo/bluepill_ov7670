/*
 * spi.h
 *
 *  Created on: 16. 7. 2019
 *      Author: daymoon
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f10x.h"
#include "defs.h"

void spi_init(void);
void SPI_Write8(uint8_t Data);
void SPI_Write16(uint16_t Data);
uint8_t SPI_Read8Write8(uint8_t writedat);
uint8_t SPI_Read8(void);

#endif /* SPI_H_ */
