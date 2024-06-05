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

SemaphoreHandle_t txCompleteSemaphore, stopCompleteSemaphore,
		rxCompleteSemaphore, txCompleteSemaphore2, stopCompleteSemaphore2,
		rxCompleteSemaphore2;

void createI2CSemaphores(void) {
	txCompleteSemaphore = xSemaphoreCreateBinary();
	if (txCompleteSemaphore == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	stopCompleteSemaphore = xSemaphoreCreateBinary();
	if (stopCompleteSemaphore == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	rxCompleteSemaphore = xSemaphoreCreateBinary();
	if (rxCompleteSemaphore == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	txCompleteSemaphore2 = xSemaphoreCreateBinary();
	if (txCompleteSemaphore2 == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	stopCompleteSemaphore2 = xSemaphoreCreateBinary();
	if (stopCompleteSemaphore2 == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	rxCompleteSemaphore2 = xSemaphoreCreateBinary();
	if (rxCompleteSemaphore2 == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed
}

void I2C_GPIO_Init1(void) {
	// Configure GPIOB for I2C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C1EN);

	// Set Pins 6 and 7 to AF mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);

	// Set Pins 6 and 7 to open drain
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

	// Set pints 6 and 7 to high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
	GPIOB->OTYPER |= (GPIO_OSPEEDR_OSPEED6_1 | GPIO_OSPEEDR_OSPEED7_1);

	// Set AFRL Table
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	GPIOB->AFR[0] |= ((4 << GPIO_AFRL_AFSEL6_Pos) | 4 << GPIO_AFRL_AFSEL7_Pos);
}

void I2C_GPIO_Init2(void) {
	// Configure GPIOB for I2C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C2EN);

	// Set Pins 10 and 11 to AF mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
	GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

	// Set Pins 10 and 11 to open drain
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);

	// Set pints 10 and 11 to high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11);
	GPIOB->OTYPER |= (GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED11_1);

	// Set AFRL Table
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
	GPIOB->AFR[1] |=
			((4 << GPIO_AFRH_AFSEL10_Pos) | 4 << GPIO_AFRH_AFSEL11_Pos);
}

void I2C_init1() {
	I2C1->CR1 &= ~I2C_CR1_PE;

	I2C1->CR1 &= ~I2C_CR1_ANFOFF;
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;

	I2C1->TIMINGR = 0X0000004;

	// Enable I2C interrupts
	I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXNE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;

	NVIC_SetPriority(I2C1_EV_IRQn, 5);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_init2() {
	I2C2->CR1 &= ~I2C_CR1_PE;

	I2C2->CR1 &= ~I2C_CR1_ANFOFF;
	I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;

	I2C2->TIMINGR = 0X0000004;

	// Enable I2C interrupts
	I2C2->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXNE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;

	NVIC_SetPriority(I2C2_EV_IRQn, 5);
	NVIC_EnableIRQ(I2C2_EV_IRQn);

	I2C2->CR1 |= I2C_CR1_PE;
}

/******************************* NUNCHUCK #1 ********************************/

void N2C_Config1() {
	uint8_t data = 0;
	uint8_t step = 1;
	uint8_t byteSender = 0;

	for (byteSender = 0; byteSender <= 1; byteSender++) {

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

		/* Wait for TXIS */
		if (xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			/* Get Data */
			data = N2C_data(step);

			/* Send Data */
			I2C1->TXDR = data;

			/* Wait for TXIS */
			if (xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
				/* Increment step (To send 2nd Byte) */
				step++;

				/* Get Data */
				data = N2C_data(step);

				/* Send Data */
				I2C1->TXDR = data;

				/* Wait for STOPF */
				if (xSemaphoreTake(stopCompleteSemaphore,
						portMAX_DELAY) == pdTRUE) {
					/* Increment step (To send 2nd data when byteSender is 0) */
					step++;
				}
			}
		}
	}
}

void N2C_Read1(uint8_t *measurments) {
	uint8_t step = 5;
	uint8_t data = 0;

	///// Write Reading Byte /////

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

	/* Wait for TXIS */
	if (xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
		/* Get Data */
		data = N2C_data(step);

		/* Send Data */
		I2C1->TXDR |= data;

		/* Wait for STOPF */
		if (xSemaphoreTake(stopCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
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
				/* Wait for RXNE */
				if (xSemaphoreTake(rxCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
					/* Receive byte */
					measurments[i] = I2C1->RXDR;
				}
			}

			I2C1->CR2 |= (I2C_CR2_STOP);
		}
	}
}

/******************************* NUNCHUCK #2 ********************************/

void N2C_Config2() {
	uint8_t data = 0;
	uint8_t step = 1;
	uint8_t byteSender = 0;

	for (byteSender = 0; byteSender <= 1; byteSender++) {
		/* Clear and then set AUTOEND bit to 1 */
		I2C2->CR2 &= ~(I2C_CR2_AUTOEND);
		I2C2->CR2 |= (I2C_CR2_AUTOEND);

		/* NBYTES = 2 (Amount of Data Needed to Be Sent) */
		/* SADD = Slave Address for Nunchuck */
		I2C2->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);
		I2C2->CR2 |= (2 << I2C_CR2_NBYTES_Pos);

		/* Write Mode */
		I2C2->CR2 &= ~I2C_CR2_RD_WRN;

		/* Send Slave Address (0x52) */
		I2C2->CR2 |= (N2C_ADDR << 1);

		/* Set START bit to 1 */
		I2C2->CR2 |= I2C_CR2_START;

		/* Wait for TXIS */
		if (xSemaphoreTake(txCompleteSemaphore2, portMAX_DELAY) == pdTRUE) {
			/* Get Data */
			data = N2C_data(step);

			/* Send Data */
			I2C2->TXDR = data;

			/* Wait for TXIS */
			if (xSemaphoreTake(txCompleteSemaphore2, portMAX_DELAY) == pdTRUE) {
				/* Increment step (To send 2nd Byte) */
				step++;

				/* Get Data */
				data = N2C_data(step);

				/* Send Data */
				I2C2->TXDR = data;

				/* Wait for STOPF */
				if (xSemaphoreTake(stopCompleteSemaphore2,
						portMAX_DELAY) == pdTRUE) {
					/* Increment step (To send 2nd data when byteSender is 0) */
					step++;
				}
			}
		}
	}
}

void N2C_Read2(uint8_t *measurments) {
	uint8_t step = 5;
	uint8_t data = 0;

	///// Write Reading Byte /////

	I2C2->CR2 = 0;

	/* Set AUTOEND bit to 1 */
	I2C2->CR2 |= I2C_CR2_AUTOEND;

	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C2->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 1 */
	I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to write */
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;

	/* Send Address */
	I2C2->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C2->CR2 |= I2C_CR2_START;

	/* Wait for ACK */
	while (!(I2C2->ISR & I2C_ISR_TXIS))
		;

	/* Get Data */
	data = N2C_data(step);

	/* Send Data */
	I2C2->TXDR |= data;

	/* Wait for STOPF */
	while (!(I2C2->ISR & I2C_ISR_STOPF))
		;

//	/* Delay for 1ms */
//	HAL_Delay(1);

///// Read Measurements /////

	I2C2->CR2 &= ~I2C_CR2_AUTOEND;
	/* Set AUTOEND bit to 1 */
	//I2C2 -> CR2 |= I2C_CR2_AUTOEND;
	/* NBYTES = (Amount of Data Needed to Be Sent) */
	/* SADD = Slave Address for Nunchuck */
	I2C2->CR2 &= ~((I2C_CR2_NBYTES_Msk) | I2C_CR2_SADD_Msk);

	/* NBYTES = 6 */
	I2C2->CR2 |= (6 << I2C_CR2_NBYTES_Pos);

	/* Set RD_WRN bit to read */
	I2C2->CR2 |= I2C_CR2_RD_WRN;

	/* Send Address */
	I2C2->CR2 |= (N2C_ADDR << 1);

	/* Set START bit to 1 */
	I2C2->CR2 |= I2C_CR2_START;

	for (uint8_t i = 0; i < 6; i++) {
		/* Wait until byte is received */
		while (!(I2C2->ISR & I2C_ISR_RXNE))
			;

		/* Receive byte */
		measurments[i] = I2C2->RXDR;
	}

	I2C2->CR2 |= (I2C_CR2_STOP);

//	HAL_Delay(1);
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

void I2C1_EV_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (I2C1->ISR & I2C_ISR_TXIS) {
		// Handle TXIS event (ready to transmit next byte)
		// Give the semaphore
		xSemaphoreGiveFromISR(txCompleteSemaphore, &xHigherPriorityTaskWoken);
	}

	if (I2C1->ISR & I2C_ISR_RXNE) {
		xSemaphoreGiveFromISR(rxCompleteSemaphore, &xHigherPriorityTaskWoken);
	}

	if (I2C1->ISR & I2C_ISR_STOPF) {
		// Handle STOPF event (stop condition detected)
		I2C1->ICR = I2C_ICR_STOPCF;  // Clear the STOPF flag
		// Give the semaphore
		xSemaphoreGiveFromISR(stopCompleteSemaphore, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void I2C2_EV_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (I2C2->ISR & I2C_ISR_TXIS) {
		// Handle TXIS event (ready to transmit next byte)
		// Give the semaphore
		xSemaphoreGiveFromISR(txCompleteSemaphore2, &xHigherPriorityTaskWoken);
	}

	if (I2C2->ISR & I2C_ISR_RXNE) {
		xSemaphoreGiveFromISR(rxCompleteSemaphore2, &xHigherPriorityTaskWoken);
	}

	if (I2C2->ISR & I2C_ISR_STOPF) {
		// Handle STOPF event (stop condition detected)
		I2C2->ICR = I2C_ICR_STOPCF;  // Clear the STOPF flag
		// Give the semaphore
		xSemaphoreGiveFromISR(stopCompleteSemaphore2,
				&xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

