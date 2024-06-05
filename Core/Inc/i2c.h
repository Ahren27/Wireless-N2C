/*
 * i2c.h
 *
 *  Created on: Jun 1, 2024
 *      Author: ahren
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#define N2C_ADDR 0X52

void I2C_init1();
void I2C_GPIO_Init1(void);
void N2C_Config1();
void N2C_Read1(uint8_t *measurments);

void I2C_init2();
void I2C_GPIO_Init2(void);
void N2C_Config2();
void N2C_Read2(uint8_t *measurments);

uint8_t N2C_data(uint8_t step);

#endif /* INC_I2C_H_ */
