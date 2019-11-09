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
#include "misc.h"

uint8_t rowdata[ROWDATASIZE];
static uint8_t cam_i2c_address;

static volatile uint8_t bitdata[20][120];
static volatile uint8_t bitdata_sample[20][120];
static volatile uint8_t detection_enable = FALSE;
static volatile uint8_t detected = FALSE;


void camera_config(void)
{
	// nastaveni portu pro hodiny, i2c, sync signaly a data
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(CAMERACLOCKS, ENABLE);						// povoleni hodin pro porty

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
	RCC_APB2PeriphClockCmd(DEBUG_CLOCKS, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DEBUG_PORT, DEBUG_PIN);

	// nastaveni preruseni tlacitka
	camera_button_config();

#ifdef OV7670
	cam_i2c_address = OV7670_I2C_ADDRESS;
#else
#ifdef OV7720
	cam_i2c_address = OV7720_I2C_ADDRESS;
#endif
#endif

	volatile uint8_t reg;

	// test cteni adresy kamery
	if (camera_GetStatus() == CAMERA_OK)
	{
		const uint8_t cameraready[] = "* camera OV7720 ready...";
		//usart_send_text((uint8_t *)&cameraready);
		//usart_newline();

		// kontrolni cteni registru vyrobce a pokusny zapis do registru
		//reg = camera_ReadReg(PID);
		//reg = camera_ReadReg(VER);
		//reg = camera_ReadReg(MIDH);
		//reg = camera_ReadReg(MIDL);

		//reg = camera_ReadReg(HSTART);
		//camera_WriteReg(HSTART, 0x12);
		//reg = camera_ReadReg(HSTART);
	}

	// inicializace
#ifdef OV7670
	ov7670_init();

	// barevny rezim odkomentovat
	/*
	ov7670_setRes(OV7670_QVGA);
	ov7670_setColorSpace(OV7670_RGB565);
	camera_WriteReg(REG_CLKRC, 1);	// podil by nemel byt delitel frekvence, zde 1
	*/

	// prahovani
	ov7670_setRes(OV7670_QQVGA);
	ov7670_setColorSpace(OV7670_YUV422);
	camera_WriteReg(REG_CLKRC, 1);	// podil by nemel byt delitel frekvence, zde 1

#else
#ifdef OV7720
	ov7720_init();
	ov7720_setRes(OV7720_QVGA);
	ov7720_setColorSpace(OV7720_RGB565);
	//ov7720_colorbarpattern();

	// prahovani
	//ov7720_setColorSpace(OV7720_YUV422);
	//ov7720_set_clk_prescaler(0x28);

#endif
#endif

	// zalozeni ulohy cteni obrazu
	//if(Scheduler_Add_Task(camera_service, 0, (SCHEDULERPERIOD * 300 MILISEKUND)) == SCH_MAX_TASKS)
	{
		// chyba pri zalozeni service
	}
	camera_get_image(TRUE);			// ziskej obrazky ve smycce

	//camera_get_threshold(TRUE, 32);		// ziskej prahovane obrazky ve smycce
}

void camera_service_handler(void)
{
	static volatile uint8_t threshold = 32;
	camera_get_threshold(FALSE, threshold);	// ziskej prahovane obrazky
}

void camera_get_image(uint8_t loop)
{
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

		if (loop == FALSE)
		{
			break;
		}
	}
	CS_high();
	__enable_irq();

}

void camera_get_threshold(uint8_t loop, uint8_t threshold)
{
	__disable_irq();

	while (1)
	{
		// cekej na VSYNC
		while (!(GPIOA->IDR & GPIO_Pin_9))
		{}
		while ((GPIOA->IDR & GPIO_Pin_9))
		{}

		uint16_t x, y;
		uint8_t *p_8bit;
		volatile uint8_t data;

		//DEBUG_PORT->ODR |= DEBUG_PIN;

		y = 120;
		while (y--)
		{
			// cekej na HREF
			while ((GPIOA->IDR & GPIO_Pin_8))
			{}
			while (!(GPIOA->IDR & GPIO_Pin_8))
			{}

			x = 160;		// priprava zapisu do radkove pameti
			while(x--)
			{
				// cekej na PCLK
				while ((GPIOA->IDR & GPIO_Pin_10))
				{}
				while (!(GPIOA->IDR & GPIO_Pin_10))
				{}
				data = GPIOA->IDR;				// Y slozka

				// prahovani
				if (data > threshold)
				{
					bitdata[x/8][y] |= (0x01 << (x % 8));
				}
				else
				{
					bitdata[x/8][y] &= ~(0x01 << (x % 8));
				}

				// cekej na dalsi PCLK
				while ((GPIOA->IDR & GPIO_Pin_10))
				{}
				while (!(GPIOA->IDR & GPIO_Pin_10))
				{}
				// vynechani cteni U nebo V slozky
			}
		}
		//DEBUG_PORT->ODR &= ~DEBUG_PIN;

		camera_detection(detection_enable);					// zjisti pohyb

		if (loop == FALSE)
		{
			break;
		}
		else
		{
			// vykresli obrazek
			//camera_show_bitdata();
		}
	}
	// zobraz data
	//camera_show_bitdata();

	__enable_irq();
}

void camera_show_bitdata(void)
{
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

	//while (1)
	{
		uint32_t x, y;
		uint8_t *p_8bit;
		volatile uint8_t data;

		// bitdata
		y = 120;
		while (y--)
		{
			p_8bit = rowdata;

			//DEBUG_PORT->ODR |= DEBUG_PIN;
			x = 160;		// priprava zapisu do radkove pameti
			while(x--)
			{
				// zobrazeni bitovych dat
				if (bitdata[x/8][y] & (0x01 << (x%8)))
				{
					data = 0xFF;
				}
				else
				{
					data = 0x00;
				}

				*(p_8bit++) = data;			// prahovana Y slozka
				*(p_8bit++) = data;			// prahovana hodnota, misto U nebo V slozky
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
		}

		// bitdata_sample
		y = 120;
		while (y--)
		{
			p_8bit = rowdata;

			//DEBUG_PORT->ODR |= DEBUG_PIN;
			x = 160;		// priprava zapisu do radkove pameti
			while(x--)
			{
				// zobrazeni bitovych dat
				if (bitdata_sample[x/8][y] & (0x01 << (x%8)))
				{
					data = 0xFF;
				}
				else
				{
					data = 0x00;
				}

				*(p_8bit++) = data;			// prahovana Y slozka
				*(p_8bit++) = data;			// prahovana hodnota, misto U nebo V slozky
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
		}

	}
	CS_high();
	__enable_irq();
}

void camera_button_config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	// nastaveni citlivosti na vyber sensoru signalem LCD_CS
	// Enable AFIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Connect EXTI12 Line EXTI9_5_IRQn
	GPIO_EXTILineConfig(BUTTON_PORTSOURCE, BUTTON_PINSOURCE);

	// Configure BUTTON_EXTI line
	EXTI_InitStructure.EXTI_Line = BUTTON_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set BUTTON_EXTI Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void camera_store_sample(void)
{
	uint16_t x, y;

	for (x = 0; x < 20; x++)
	{
		for (y = 0; y < 120; y++)
		{
			bitdata_sample[x][y] = bitdata[x][y];
		}
	}
}

void camera_detection(uint8_t enable)
{
	static volatile uint16_t diff = 0;
	uint16_t x, y, b;

	if (enable)
	{
		for (x = 0; x < 20; x++)
		{
			for (y = 0; y < 120; y++)
			{
				// kazdy bit je nutno porovnat
				for (b = 0; b < 7; b++)
				{
					if ( (bitdata_sample[x][y] & (0x01 << b)) != (bitdata[x][y] & (0x01 << b)) )
					{
						diff++;
					}
				}
			}
		}
	}

	// vypis pocet rozdilu na displej
	uint8_t diff_text[5];
	uint16_t temp = diff;
	diff_text[0] = temp / 1000 + '0';
	temp %= 1000;
	diff_text[1] = temp / 100 + '0';
	temp %= 100;
	diff_text[2] = temp / 10 + '0';
	temp %= 10;
	diff_text[3] = temp + '0';
	diff_text[4] = 0;

	ili_text_print(180, 100, diff_text, ILI9341_RED, FALSE, 80);

	if (diff > CAMERA_DET_LIMIT)
	{
		// detekovan pohyb
		camera_set_detected_status(TRUE);

		// zakaz dalsi detekci
		camera_set_detection_status(FALSE);

		// rozsvit LED
		camera_set_detected_led(TRUE);

		// vykresli obrazek
		camera_show_bitdata();
	}

	diff = 0;
}

uint8_t camera_get_detection_status(void)
{
	return detection_enable;
}

uint8_t camera_get_detected_status(void)
{
	return detected;
}

void camera_set_detected_status(uint8_t status)
{
	detected = status;
}


void camera_set_detection_status(uint8_t status)
{
	detection_enable = status;

	if (status == TRUE)
	{
		// rozsvit
		camera_set_run_led(TRUE);
		camera_set_detected_led(FALSE);
	}
	else
	{
		// zhasni
		camera_set_run_led(FALSE);
		camera_set_detected_led(FALSE);
	}
}

void camera_set_detected_led(uint8_t status)
{
	if (status == TRUE)
	{
		GPIO_ResetBits(LEDDET_PORT, LEDDET_PIN);				// detected LED on
	}
	else
	{
		GPIO_SetBits(LEDDET_PORT, LEDDET_PIN);					// detected LED off
	}
}

void camera_set_run_led(uint8_t status)
{
	if (status == TRUE)
	{
		GPIO_ResetBits(LEDRUN_PORT, LEDRUN_PIN);				// run LED on
	}
	else
	{
		GPIO_SetBits(LEDRUN_PORT, LEDRUN_PIN);					// run LED off
	}
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

void wrSensorRegs8_8(const struct regval_list reglist[])
{
	volatile uint8_t reg_addr;
	volatile uint8_t reg_val;

	const struct regval_list *next = reglist;

	reg_addr = next->reg_num;
	reg_val = next->value;

	while((reg_addr != 0xff) && (reg_val != 0xff))
	{
		camera_WriteReg(reg_addr, reg_val);
		next++;
		reg_addr = next->reg_num;
		reg_val = next->value;
	}
}
