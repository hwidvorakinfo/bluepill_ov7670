/*
 * spi.c
 *
 *  Created on: 16. 7. 2019
 *      Author: daymoon
 */

#include "spi.h"
#include "application.h"
#include "ili9341.h"

void spi_init(void)
{
#ifdef SPI
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

	// hodiny pro piny a periferii
	RCC_APB2PeriphClockCmd(LCD_MOSI_CLK | LCD_MISO_CLK | LCD_SCK_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(LCD_SPI_CLK, ENABLE);

	// piny SPI SCK, MOSI, MISO
	GPIO_InitStructure.GPIO_Pin = LCD_SCK_PIN | LCD_MOSI_PIN | LCD_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LCD_SCK_PORT, &GPIO_InitStructure);

	// Alternativni funkce
	//GPIO_PinAFConfig(LCD_SCK_PORT, LCD_SCK_SOURCE, LCD_SCK_AF);
	//GPIO_PinAFConfig(LCD_MOSI_PORT, LCD_MOSI_SOURCE, LCD_MOSI_AF);
	//GPIO_PinAFConfig(LCD_MISO_PORT, LCD_MISO_SOURCE, LCD_MISO_AF);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(LCD_SPI);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	// APB1=36MHz, optimalni je frekvence kolem 4MHz pro dlouhe vedeni
	// 36000kHz/8=4.25MHz
	// 36000kHz/16=2.25MHz
	// 36000kHz/4=9MHz
	// 36000kHz/2=18MHz
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //ILISPIPRESCALER;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(LCD_SPI, &SPI_InitStructure);
	SPI_Cmd(LCD_SPI, ENABLE); /* LCD_SPI enable */

	/* Drain SPI2 */
	while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET)
	{}
	SPI_I2S_ReceiveData(LCD_SPI);
#endif
}


void SPI_Write8(uint8_t Data)
{
  /* Wait until the transmit buffer is empty */
  //while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) != SET) {}
  while (!(LCD_SPI->SR & SPI_I2S_FLAG_TXE))
  {}

  /* Send the byte */
  *(__IO uint8_t *)((uint32_t)LCD_SPI + 0x0C) = Data;			//vykostene SPI_SendData8(LCD_SPI, Data);

  //while (!(LCD_SPI->SR & SPI_I2S_FLAG_TXE))
  //{}

  /* Wait until the transmit buffer is empty */
  //while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) != SET) {}
}

inline void SPI_Write16(uint16_t Data)
{
	// zde nepouzite prohozeni bytu Data = (((Data & 0x00FF) << 8) | ((Data & 0xFF00) >> 8));

	SPI_Write8((Data & 0xFF00) >> 8);
	SPI_Write8(Data & 0x00FF);
}

uint8_t SPI_Read8Write8(uint8_t writedat)
{
	uint16_t i = 0;

    /* Loop while DR register in not empty */
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET);
    /* Send byte through the SPI peripheral */
    SPI_I2S_SendData(LCD_SPI, writedat);

    /* Wait to receive a byte */
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    i = SPI_I2S_ReceiveData(LCD_SPI) ;

    /* Return the byte read from the SPI bus */
    return i;
}

uint8_t SPI_Read8(void)
{
	uint16_t i = 0;

	SPI_I2S_SendData(LCD_SPI, 0x00);

    /* Wait to receive a byte */
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    i = SPI_I2S_ReceiveData(LCD_SPI) ;

    /* Return the byte read from the SPI bus */
    return i;
}


