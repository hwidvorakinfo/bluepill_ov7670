/*
 * camera.c
 *
 *  Created on: 7. 8. 2019
 *      Author: Petr Dvorak
 */

#include "camera.h"
#include "delay.h"
#include "ili9341.h"
#include "spi.h"
#include "i2c.h"

uint8_t rowdata[ROWDATASIZE];
static uint8_t cam_i2c_address;

void camera_config(void)
{
	// nastaveni portu pro hodiny, i2c, sync signaly a data
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(CAMERACLOCKS, ENABLE);						// povoleni hodin pro porty

	// nastaveni vstupu data 0-3
	GPIO_InitStructure.GPIO_Pin = DATA0PIN | DATA1PIN | DATA2PIN | DATA3PIN | DATA4PIN | DATA5PIN | DATA6PIN | DATA7PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DATA07PORT, &GPIO_InitStructure);

	// nastaveni vstupu VSYNC
	GPIO_InitStructure.GPIO_Pin = VSYNCPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(VSYNCPORT, &GPIO_InitStructure);

	// nastaveni vstupu HSYNC
	GPIO_InitStructure.GPIO_Pin = HSYNCPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HSYNCPORT, &GPIO_InitStructure);

	// nastaveni vstupu PCLK
	GPIO_InitStructure.GPIO_Pin = PCLKPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PCLKPORT, &GPIO_InitStructure);

	// nastaveni debug portu
	RCC_AHBPeriphClockCmd(DEBUG_CLOCKS, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DEBUG_PORT, DEBUG_PIN);

	cam_i2c_address = OV7670_I2C_ADDRESS;

	volatile uint8_t reg;

	// test cteni adresy kamery
	if (camera_GetStatus() == CAMERA_OK)
	{
		const uint8_t cameraready[] = "* camera OV7670 ready...";
		//usart_send_text((uint8_t *)&cameraready);
		//usart_newline();

		// kontrolni cteni registru vyrobce a pokusny zapis do registru
		//reg = camera_ReadReg(REG_PID);
		//reg = camera_ReadReg(REG_VER);
		//reg = camera_ReadReg(REG_MIDH);
		//reg = camera_ReadReg(REG_MIDL);

		//reg = camera_ReadReg(REG_HSTART);
		//camera_WriteReg(REG_HSTART, 0x12);
		//reg = camera_ReadReg(REG_HSTART);
	}

	// inicializace
	ov7670_init();
	ov7670_setRes(QVGA);
	ov7670_setColorSpace(RGB565);
	camera_WriteReg(REG_CLKRC, 1);	// podil by nemel byt delitel frekvence, zde 1

	//
	// priprava DMA pro SPI
	//
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(LCDDMACLK, ENABLE);
	DMA_DeInit(LCDSPI_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rowdata;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = ROWDATASIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(LCDSPI_CHANNEL, &DMA_InitStructure);

	//
	// displej priprava zapisu
	//
	// Set column range.
	send_instruction(ILI9341_CASET);
	send_data16(0x0000);					// start column
	send_data16((uint16_t)(319));			// end column
	// Set row range.
	send_instruction(ILI9341_PASET);
	send_data16(0x0000);					// start line
	send_data16((uint16_t)(239));			// end line

	// Set 'write to RAM'
	send_instruction(ILI9341_RAMWR);

	CS_low();
	DC_data(); 											// DC = H

	__disable_irq();

	while (1)
	{
		// cekej na VSYNC
		while (!(GPIOA->IDR & GPIO_Pin_9))
		{}
		while ((GPIOA->IDR & GPIO_Pin_9))
		{}

		uint32_t x, y;
		uint8_t *p_8bit;

		y = 240;
		while (y--)
		{
			// cekej na HREF
			while ((GPIOA->IDR & GPIO_Pin_8))
			{}
			while (!(GPIOA->IDR & GPIO_Pin_8))
			{}

			p_8bit = rowdata;

			//DEBUG_PORT->ODR |= DEBUG_PIN;
			x = 320;		// priprava zapisu do radkove pameti
			while(x--)
			{
				// cekej na PCLK
				while ((GPIOA->IDR & GPIO_Pin_10))
				{}
				while (!(GPIOA->IDR & GPIO_Pin_10))
				{}
				*(p_8bit++) = GPIOA->IDR;

				// cekej na dalsi PCLK
				while ((GPIOA->IDR & GPIO_Pin_10))
				{}
				while (!(GPIOA->IDR & GPIO_Pin_10))
				{}
				*(p_8bit++) = GPIOA->IDR;

			}
			//DEBUG_PORT->ODR &= ~DEBUG_PIN;

			// odeslani dat v blank intervalu
			LCDSPI_CHANNEL->CMAR = (uint32_t)rowdata;
			LCDSPI_CHANNEL->CNDTR = ROWDATASIZE;
			LCDSPI_CHANNEL->CCR |= DMA_CCR1_EN;				// DMA1CH5 enable
			LCD_SPI->CR2 |= SPI_I2S_DMAReq_Tx;				// SPI2 DMA TX enable
			while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
			DMA_ClearFlag(DMA1_FLAG_TC5);
			DMA_ClearFlag(DMA1_IT_GL5 | DMA1_IT_TC5 | DMA1_IT_HT5 | DMA1_IT_TE5);
			while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY));
			LCDSPI_CHANNEL->CCR &= (uint16_t)(~DMA_CCR1_EN);// DMA1CH5 enable
			LCD_SPI->CR2 &= (uint16_t)~SPI_I2S_DMAReq_Tx;	// SPI2 DMA TX disable
			/*
			for (i = 0; i < 640; i++)
			{
				SPI_Write8(rowdata[i]);
			}
			*/
		}
	}
	CS_high();
}

// funkce pouze poslani adresy ciloveho zarizeni a overeni, ze odpovida pomoci ACK
uint8_t camera_GetStatus(void)
{
  uint32_t I2C_TimeOut = CAMERA_LONG_TIMEOUT/50;

  return CAMERA_OK;
}

// upravena funkce pro cteni registru z kamery, je tam poslani adresy, cislo registru, generovani STOP, cekani 1 ms, dalsi poslani adresy pro cteni, a jiz cteni dat, na konci opet cekani 1 ms
// standardni cteni je bez generovani STOP uprostred a cekani 1 ms
uint8_t camera_ReadReg(uint8_t RegName)
{
  uint8_t BufferRX[1] ={0};
  uint16_t tmp = 0;
  uint32_t DataNum = 0;
  uint16_t CameraTimeout;

	I2C_start(CAM_I2C, cam_i2c_address<<1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(CAM_I2C, RegName); 										// write byte to the slave
	I2C_stop(CAM_I2C); 											  		// stop the transmission

	delay_ms(1);

	I2C_start(CAM_I2C, cam_i2c_address<<1, I2C_Direction_Receiver);		// start a transmission in Master receiver mode
	BufferRX[0] = I2C_read_nack(CAM_I2C); 								// read one byte and do not request another byte

	delay_ms(1);

	/* return a Reg value */
	return (uint8_t)BufferRX[0];}

// upravena funkce pro zapis registru kamery, na konci je pridano cekani 1 ms kvuli kamere, jinak jde o standardni zapis 1 byte
void camera_WriteReg(uint8_t RegName, uint8_t RegValue)
{
	uint32_t DataNum = 0;
	uint8_t BufferTX[1] ={0};
	uint16_t CameraTimeout;

	BufferTX[0] = (uint8_t)(RegValue);

	I2C_start(CAM_I2C, cam_i2c_address<<1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(CAM_I2C, RegName); 										// write addr to the slave
	I2C_write(CAM_I2C, RegValue); 										// write byte to the slave
	I2C_stop(CAM_I2C); 											  		// stop the transmission

	delay_ms(1);
}

void camera_spidma_transfer(void)
{
	LCDSPI_CHANNEL->CMAR = (uint32_t)rowdata;
	LCDSPI_CHANNEL->CNDTR = ROWDATASIZE;
	LCDSPI_CHANNEL->CCR |= DMA_CCR1_EN;				// DMA1CH5 enable
	LCD_SPI->CR2 |= SPI_I2S_DMAReq_Tx;				// SPI2 DMA TX enable
	while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
	DMA_ClearFlag(DMA1_FLAG_TC5);
	DMA_ClearFlag(DMA1_IT_GL5 | DMA1_IT_TC5 | DMA1_IT_HT5 | DMA1_IT_TE5);
	while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY));

	LCDSPI_CHANNEL->CCR &= (uint16_t)(~DMA_CCR1_EN);// DMA1CH5 enable
	LCD_SPI->CR2 &= (uint16_t)~SPI_I2S_DMAReq_Tx;	// SPI2 DMA TX disable

	// DMA test
	/*uint16_t index = 0;
	while (index < ROWDATASIZE)
	{
		rowdata[index++] = 0xF8;
		rowdata[index++] = 0x1F;
	}
	while(1)
	{
		LCDSPI_CHANNEL->CMAR = (uint32_t)rowdata;
		LCDSPI_CHANNEL->CNDTR = ROWDATASIZE;
		DMA_Cmd(LCDSPI_CHANNEL, ENABLE);
		SPI_I2S_DMACmd(LCD_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

		while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
		DMA_ClearFlag(DMA1_FLAG_TC5);
		DMA_ClearFlag(DMA1_IT_GL5 | DMA1_IT_TC5 | DMA1_IT_HT5 | DMA1_IT_TE5);
		while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY));
		LCDSPI_CHANNEL->CCR &= (uint16_t)(~DMA_CCR1_EN);
		LCD_SPI->CR2 &= (uint16_t)~SPI_I2S_DMAReq_Tx;
	}*/
}

