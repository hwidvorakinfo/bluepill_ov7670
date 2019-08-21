/*
 * camera.h
 *
 *  Created on: 7. 8. 2019
 *      Author: Petr Dvorak
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "stm32f10x.h"
#include "defs.h"
#include "ov7670.h"
#include "application.h"

typedef enum
{
  CAMERA_OK = 0,
  CAMERA_FAIL
}Camera_Status_TypDef;

#define CAMERACLOCKS	DATA_CLOCKS | SYNC_CLOCKS | PCLK_CLOCKS | XCLK_CLOCKS

#define DATA_CLOCKS		OUT_PORT_A_CLK
#define DATA07PORT		GPIOA
#define DATA0PIN		GPIO_Pin_0
#define DATA1PIN		GPIO_Pin_1
#define DATA2PIN		GPIO_Pin_2
#define DATA3PIN		GPIO_Pin_3
#define DATA4PIN		GPIO_Pin_4
#define DATA5PIN		GPIO_Pin_5
#define DATA6PIN		GPIO_Pin_6
#define DATA7PIN		GPIO_Pin_7

#define SYNC_CLOCKS		OUT_PORT_A_CLK | OUT_PORT_B_CLK
#define VSYNCPORT		GPIOB
#define VSYNCPIN		GPIO_Pin_7
#define HSYNCPORT		GPIOA
#define HSYNCPIN		GPIO_Pin_4

#define PCLK_CLOCKS		OUT_PORT_A_CLK
#define PCLKPORT		GPIOA
#define PCLKPIN			GPIO_Pin_6

#define XCLK_PORT		GPIOA
#define XCLK_CLOCKS		OUT_PORT_A_CLK
#define XCLK_PIN		GPIO_Pin_11
#define XCLK_SOURCE		GPIO_PinSource11

#define DEBUG_PORT		GPIOA
#define DEBUG_CLOCKS	OUT_PORT_A_CLK
#define DEBUG_PIN		GPIO_Pin_12

#define SYSTEMCORECLOCK	72000000
#define XCLKFREQ		12000000
#define PWM_DUTY		5000
#define PWM_MAX			10000

#define XCLK_TIM		TIM1
#define XCLK_TIM_CLOCKS	RCC_APB2Periph_TIM1
#define XCLK_TIM_PRESC	0

#define CAM_I2C			I2C2
#define CAM_I2C_CLK		RCC_APB1Periph_I2C2
#define I2C_PIN_CLOCKS	OUT_PORT_B_CLK

#define CAM_SCL_PIN		GPIO_Pin_10
#define CAM_SCL_PORT	GPIOB
#define CAM_SCL_CLK		OUT_PORT_B_CLK
#define CAM_SCL_SOURCE	GPIO_PinSource10
#define CAM_SCL_AF		GPIO_AF_1

#define CAM_SDA_PIN		GPIO_Pin_11
#define CAM_SDA_PORT	GPIOB
#define CAM_SDA_CLK		OUT_PORT_B_CLK
#define CAM_SDA_SOURCE	GPIO_PinSource11
#define CAM_SDA_AF		GPIO_AF_1

#define CAM_I2C_TIMING	0xB0420F13

#define ROWDATASIZE		320*2
#define LCDDMACLK		RCC_AHBPeriph_DMA1
#define LCDSPI_CHANNEL	DMA1_Channel5
#define SPI_DR_ADDRESS	(uint32_t)&LCD_SPI->DR

#define CAMERA_LONG_TIMEOUT	50000


void camera_config(void);
uint8_t camera_GetStatus(void);
uint8_t camera_ReadReg(uint8_t RegName);
void camera_WriteReg(uint8_t RegName, uint8_t RegValue);
void camera_spidma_transfer(void);

#endif /* CAMERA_H_ */
