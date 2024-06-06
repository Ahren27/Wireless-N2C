/*
 * i2c.c
 *
 *  Created on: Jun 1, 2024
 *      Author: ahren
 */

#include "main.h"
#include "I2C.h"
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

void I2C_GPIO_Init1(void) {
	// Configure GPIOB for I2C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C1EN);

	// Set Pins 8 and 9 to AF mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

	// Set Pins 8 and 9 to open drain
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);

	// Set pints 8 and 9 to high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	GPIOB->OTYPER |= (GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED9_1);

	// Set AFRL Table
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL8_Pos) | 4 << GPIO_AFRH_AFSEL9_Pos);
}

void I2C_GPIO_Init2(void) {
	// Configure GPIOC for I2C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);
	RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C3EN);

	// Set Pins 0 and 1 to AF mode
	GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOC->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);

	// Set Pins 0 and 1 to open drain
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);
	GPIOC->OTYPER |= (GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);

	// Set pints 0 and 1 to high speed
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
	GPIOC->OTYPER |= (GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1);

	// Set AFRL Table
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);
	GPIOC->AFR[0] |=
			((4 << GPIO_AFRL_AFSEL0_Pos) | 4 << GPIO_AFRL_AFSEL1_Pos);
}

void I2C_init1() {
	I2C1->CR1 &= ~I2C_CR1_PE;

	I2C1->CR1 &= ~I2C_CR1_ANFOFF;
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;

	I2C1->TIMINGR = 0X0000004;

	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_init2() {
	I2C3->CR1 &= ~I2C_CR1_PE;

	I2C3->CR1 &= ~I2C_CR1_ANFOFF;
	I2C3->CR1 &= ~I2C_CR1_NOSTRETCH;

	I2C3->TIMINGR = 0X0000004;

	I2C3->CR1 |= I2C_CR1_PE;
}

/******************************* NUNCHUCK #1 ********************************/

void N2C_Config1() {
	uint8_t data = 0;
	uint8_t step = 1;
	uint8_t byteSender = 0;

	for (byteSender = 0; byteSender <= 1; byteSender++) {
		I2C_init1();

		/* Clear and then set AUTOEND bit to 1 */
		I2C1->CR2 &= ~(I2C_CR2_AUTOEND);
		I2C1->CR2 |= (I2C_CR2_AUTOEND);

		/* NBYTES = 2 (Amount of Data Needed to Be Sent) */
		/* SADD = Slave Address for Nunchuck */
		I2C1->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);
		I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos);

		/* Write Mode */
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;

		/* Send Slave Address (0x52) */
		I2C1->CR2 |= (N2C_ADDR << 1);

		/* Set START bit to 1 */
		I2C1->CR2 |= I2C_CR2_START;

		/* Wait for ACK */
		while (!(I2C1->ISR & I2C_ISR_TXIS))
			;

		/* Get Data */
		data = N2C_data(step);

		/* Send Data */
		I2C1->TXDR = data;

		/* Wait for ACK */
		while (!(I2C1->ISR & I2C_ISR_TXIS))
			;

		/* Increment step (To send 2nd Byte) */
		step++;

		/* Get Data */
		data = N2C_data(step);

		/* Send Data */
		I2C1->TXDR = data;

		/* Wait for STOPF */
		while (!(I2C1->ISR & I2C_ISR_STOPF))
			;

		/* Increment step (To send 2nd data when byteSender is 0) */
		step++;

		/* Delay for 1ms */
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void N2C_Read1(uint8_t *measurments) {
	uint8_t step = 5;
	uint8_t data = 0;

	///// Write Reading Byte /////

	I2C_init1();

	I2C1->CR2 = 0;

	/* Set AUTOEND bit to 1 */
	I2C1->CR2 |= I2C_CR2_AUTOEND;

	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C1->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 1 */
	I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to write */
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;

	/* Send Address */
	I2C1->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C1->CR2 |= I2C_CR2_START;

	/* Wait for ACK */
	while (!(I2C1->ISR & I2C_ISR_TXIS))
		;

	/* Get Data */
	data = N2C_data(step);

	/* Send Data */
	I2C1->TXDR |= data;

	/* Wait for STOPF */
	while (!(I2C1->ISR & I2C_ISR_STOPF))
		;

	/* Delay for 1ms */
	vTaskDelay(pdMS_TO_TICKS(1));

	///// Read Measurements /////

	I2C1->CR2 &= ~I2C_CR2_AUTOEND;
	/* Set AUTOEND bit to 1 */
	//I2C1 -> CR2 |= I2C_CR2_AUTOEND;
	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C1->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 6 */
	I2C1->CR2 |= (6 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to read */
	I2C1->CR2 |= I2C_CR2_RD_WRN;

	/* Send Address */
	I2C1->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C1->CR2 |= I2C_CR2_START;

	for (uint8_t i = 0; i < 6; i++) {
		/* Wait until byte is received */
		while (!(I2C1->ISR & I2C_ISR_RXNE))
			;

		/* Receive byte */
		measurments[i] = I2C1->RXDR;
	}

	I2C1->CR2 |= (I2C_CR2_STOP);

	//HAL_Delay(1);
	vTaskDelay(pdMS_TO_TICKS(1));
}

/******************************* NUNCHUCK #2 ********************************/

void N2C_Config2() {
	uint8_t data = 0;
	uint8_t step = 1;
	uint8_t byteSender = 0;

	for (byteSender = 0; byteSender <= 1; byteSender++) {
		I2C_init2();

		/* Clear and then set AUTOEND bit to 1 */
		I2C3->CR2 &= ~(I2C_CR2_AUTOEND);
		I2C3->CR2 |= (I2C_CR2_AUTOEND);

		/* NBYTES = 2 (Amount of Data Needed to Be Sent) */
		/* SADD = Slave Address for Nunchuck */
		I2C3->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);
		I2C3->CR2 |= (2 << I2C_CR2_NBYTES_Pos);

		/* Write Mode */
		I2C3->CR2 &= ~I2C_CR2_RD_WRN;

		/* Send Slave Address (0x52) */
		I2C3->CR2 |= (N2C_ADDR << 1);

		/* Set START bit to 1 */
		I2C3->CR2 |= I2C_CR2_START;

		/* Wait for ACK */
		while (!(I2C3->ISR & I2C_ISR_TXIS))
			;

		/* Get Data */
		data = N2C_data(step);

		/* Send Data */
		I2C3->TXDR = data;

		/* Wait for ACK */
		while (!(I2C3->ISR & I2C_ISR_TXIS))
			;

		/* Increment step (To send 2nd Byte) */
		step++;

		/* Get Data */
		data = N2C_data(step);

		/* Send Data */
		I2C3->TXDR = data;

		/* Wait for STOPF */
		while (!(I2C3->ISR & I2C_ISR_STOPF))
			;

		/* Increment step (To send 2nd data when byteSender is 0) */
		step++;

		/* Delay for 1ms */
		//HAL_Delay(1);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void N2C_Read2(uint8_t *measurments) {
	uint8_t step = 5;
	uint8_t data = 0;

	///// Write Reading Byte /////

	I2C_init2();

	I2C3->CR2 = 0;

	/* Set AUTOEND bit to 1 */
	I2C3->CR2 |= I2C_CR2_AUTOEND;

	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C3->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 1 */
	I2C3->CR2 |= (1 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to write */
	I2C3->CR2 &= ~I2C_CR2_RD_WRN;

	/* Send Address */
	I2C3->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C3->CR2 |= I2C_CR2_START;

	/* Wait for ACK */
	while (!(I2C3->ISR & I2C_ISR_TXIS))
		;

	/* Get Data */
	data = N2C_data(step);

	/* Send Data */
	I2C3->TXDR |= data;

	/* Wait for STOPF */
	while (!(I2C3->ISR & I2C_ISR_STOPF))
		;

	/* Delay for 1ms */
	//HAL_Delay(1);
	vTaskDelay(pdMS_TO_TICKS(1));

	///// Read Measurements /////

	I2C3->CR2 &= ~I2C_CR2_AUTOEND;
	/* Set AUTOEND bit to 1 */
	//I2C3 -> CR2 |= I2C_CR2_AUTOEND;
	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C3->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 6 */
	I2C3->CR2 |= (6 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to read */
	I2C3->CR2 |= I2C_CR2_RD_WRN;

	/* Send Address */
	I2C3->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C3->CR2 |= I2C_CR2_START;

	for (uint8_t i = 0; i < 6; i++) {
		/* Wait until byte is received */
		while (!(I2C3->ISR & I2C_ISR_RXNE))
			;

		/* Receive byte */
		measurments[i] = I2C3->RXDR;
	}

	I2C3->CR2 |= (I2C_CR2_STOP);

	//HAL_Delay(1);
	vTaskDelay(pdMS_TO_TICKS(1));
}

/* Returns the appropriate byte to be sent over I2C */
uint8_t N2C_data(uint8_t step) {
	uint8_t data;

	switch (step) {
	case 1:
		data = 0xF0;
		break;
	case 2:
		data = 0x55;
		break;
	case 3:
		data = 0xFB;
		break;
	case 4:
	case 5:
		data = 0x00;
		break;
	}

	return data;
}
