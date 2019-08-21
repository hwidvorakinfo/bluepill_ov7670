/*
 * i2c.h
 *
 *  Created on: 14. 8. 2019
 *      Author: Petr Dvorak
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f10x.h"
#include "defs.h"
#include "stm32f10x_conf.h"

#define SLAVE_ADDRESS	0x70
#define I2C_SPEED 		100000

void i2c_init(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void I2C_WriteReg(uint8_t addr, uint8_t byte);
uint8_t I2C_ReadReg(uint8_t reg);

#endif /* I2C_H_ */
