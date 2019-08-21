/*
 * i2c.c
 *
 *  Created on: 14. 8. 2019
 *      Author: Petr Dvorak
 */

#include "i2c.h"
#include "application.h"
#include "camera.h"

void i2c_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	// Turn on peripherals
	RCC_APB1PeriphClockCmd(CAM_I2C_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(CAM_SCL_CLK | CAM_SDA_CLK, ENABLE);

	// Configure I2C SCL and SDA pins.
	GPIO_InitStructure.GPIO_Pin = CAM_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(CAM_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CAM_SDA_PIN;
	GPIO_Init(CAM_SDA_PORT, &GPIO_InitStructure);

	// Configure INT pin.
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_PinAFConfig(CAM_SCL_PORT, CAM_SCL_SOURCE, CAM_SCL_AF);
	//GPIO_PinAFConfig(CAM_SDA_PORT, CAM_SDA_SOURCE, CAM_SDA_AF);


	// Reset I2C
	RCC_APB1PeriphResetCmd(CAM_I2C_CLK, ENABLE);
	RCC_APB1PeriphResetCmd(CAM_I2C_CLK, DISABLE);

	// Configure I2C
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x10;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(CAM_I2C, &I2C_InitStructure);
	I2C_Cmd(CAM_I2C, ENABLE);
}

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_WriteReg(uint8_t addr, uint8_t byte)
{
	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);

	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(I2C1, addr); 										  	// write addr to the slave
	I2C_write(I2C1, byte); 										  	// write byte to the slave
	I2C_stop(I2C1); 											  	// stop the transmission
}

uint8_t I2C_ReadReg(uint8_t reg)
{
	uint8_t read;

	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);

	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(I2C1, reg); 								  			// write one byte to the slave
	I2C_stop(I2C1); 											  	// stop the transmission

	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);		// start a transmission in Master receiver mode
	read = I2C_read_nack(I2C1); 									// read one byte and do not request another byte
	return read;
}
