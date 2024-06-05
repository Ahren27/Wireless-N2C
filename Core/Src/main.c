/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "i2c.h"
#include "uart.h"
//#include "spsgrf.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TaskHandle_t task1Handler, task2Handler, task3Handler, task4Handler;
SemaphoreHandle_t sendDataSema, configMutex;

// Raw data from I2C Nunchuck 1 (PB6, PB7) */
uint8_t measurments1[6];

// Raw data from I2C Nunchuck 2 (PB10, PB11) */
uint8_t measurments2[6];

// Global Data Variables
volatile uint8_t stick_x1 = 0;
volatile uint8_t stick_y1 = 0;

volatile uint16_t acc_x1 = 0;
volatile uint16_t acc_y1 = 0;
volatile uint16_t acc_z1 = 0;

volatile uint8_t button_c1 = 0;
volatile uint8_t button_z1 = 0;

volatile uint8_t stick_x2 = 0;
volatile uint8_t stick_y2 = 0;

volatile uint16_t acc_x2 = 0;
volatile uint16_t acc_y2 = 0;
volatile uint16_t acc_z2 = 0;

volatile uint8_t button_c2 = 0;
volatile uint8_t button_z2 = 0;

// Output Strings
char sx_str1[13];
char sy_str1[13];
char ax_str1[13];
char ay_str1[13];
char az_str1[13];
char bc_str1[13];
char bz_str1[13];

// Output Strings
char sx_str2[13];
char sy_str2[13];
char ax_str2[13];
char ay_str2[13];
char az_str2[13];
char bc_str2[13];
char bz_str2[13];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void split_data(void);
void print_data(void);

/* Task function prototypes --------------------------------------------------*/
void Task1(void *argument);
void Task2(void *argument);
	BaseType_t retVal; // used for checking task creation

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Set up RTOS */

	/* Create the tasks */
	retVal = xTaskCreate(Task1, "Set Up Program", configMINIMAL_STACK_SIZE * 2,
	NULL, osPriorityHigh, &task1Handler);
	if (retVal != pdPASS) {
		while (1)
			;
	} // check if task creation failed
//
//	retVal = xTaskCreate(Task2, "Send Data", configMINIMAL_STACK_SIZE * 2,
//	NULL, osPriorityNormal, &task2Handler);
//	if (retVal != pdPASS) {
//		while (1)
//			;
//	} // check if task creation failed

	sendDataSema = xSemaphoreCreateBinary();
	if (sendDataSema == NULL) {
		while (1)
			;
	} // check if binary semaphore creation failed

	configMutex = xSemaphoreCreateMutex();
	if (configMutex == NULL) {
		while (1)
			;
	}

	//createI2CSemaphores();

	/* Start scheduler */
	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	while (1)
		;
}

/* Define Tasks --------------------------------------------------------------*/

/* Configure Device */
void Task1(void *argument) {
	// Infinite Loop
	for (;;) {

		// Take the mutex before configuring I2C
		if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
			// Configure Peripherals
			UART_Init();
			I2C_GPIO_Init1();
			I2C_GPIO_Init2();
			I2C_init1();
			I2C_init2();
			N2C_Config1();
			N2C_Config2();

			// Release the mutex after transmission
			xSemaphoreGive(configMutex);
		}

		retVal = xTaskCreate(Task2, "Send Data", configMINIMAL_STACK_SIZE * 2,
		NULL, osPriorityNormal, &task2Handler);
		if (retVal != pdPASS) {
			while (1)
				;
		} // check if task creation failed


		// Task not needed for rest of program duration, so delete it
		vTaskDelete(NULL);
	}
}

/* Send Data */
void Task2(void *argument) {
	// Infinite Loop
	for (;;) {
		// Get measurements for Nunchuck #2
		N2C_Read2(measurments2);

		// Get measurements for Nunchuck #1
		N2C_Read1(measurments1);

		// Decode Raw Bytes Measurement into Global Variables
		split_data();

		// Uploads data to serial port
		print_data();

		// Wait for 5ms
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

// Decode Raw Bytes Measurement into Global Variables
void split_data() {
// x and y axis from first 2 bytes
	stick_x1 = measurments1[0];
	stick_y1 = measurments1[1];

// x and y axis from first 2 bytes
	stick_x2 = measurments2[0];
	stick_y2 = measurments2[1];

// Might want to Reset accelerometer values
	acc_x1 = 0;
	acc_y1 = 0;
	acc_z1 = 0;

// Might want to Reset accelerometer values
	acc_x2 = 0;
	acc_y2 = 0;
	acc_z2 = 0;

// higher 8 bits of accelerometer from next 3 bytes
	acc_x1 |= (measurments1[2] << 2);
	acc_y1 |= (measurments1[3] << 2);
	acc_z1 |= (measurments1[4] << 2);

// higher 8 bits of accelerometer from next 3 bytes
	acc_x2 |= (measurments2[2] << 2);
	acc_y2 |= (measurments2[3] << 2);
	acc_z2 |= (measurments2[4] << 2);

// lower 2 bits from last byte
	acc_x1 |= (((1 << 2) - 1) & (measurments1[5] >> 3));
	acc_y1 |= (((1 << 2) - 1) & (measurments1[5] >> 5));
	acc_z1 |= (((1 << 2) - 1) & (measurments1[5] >> 7));

// lower 2 bits from last byte
	acc_x2 |= (((1 << 2) - 1) & (measurments2[5] >> 3));
	acc_y2 |= (((1 << 2) - 1) & (measurments2[5] >> 5));
	acc_z2 |= (((1 << 2) - 1) & (measurments2[5] >> 7));

// Buttons are last 2 bits of last byte
	button_c1 = (1 & ~(measurments1[5] >> 1));
	button_z1 = (1 & ~(measurments1[5] >> 0));

// Buttons are last 2 bits of last byte
	button_c2 = (1 & ~(measurments2[5] >> 1));
	button_z2 = (1 & ~(measurments2[5] >> 0));
}

// Uploads data to serial port
void print_data() {
// Convert Data to Strings
	sprintf(sx_str1, "StickX1:%i", stick_x1);
	sprintf(sy_str1, "StickY1:%i", stick_y1);

	sprintf(ax_str1, "AccX1:%i", acc_x1);
	sprintf(ay_str1, "AccY1:%i", acc_y1);
	sprintf(az_str1, "AccZ1:%i", acc_z1);

	sprintf(bc_str1, "ButtonC1:%i", button_c1);
	sprintf(bz_str1, "ButtonZ1:%i", button_z1);

// Convert Data to Strings
	sprintf(sx_str2, "StickX2:%i", stick_x2);
	sprintf(sy_str2, "StickY2:%i", stick_y2);

	sprintf(ax_str2, "AccX2:%i", acc_x2);
	sprintf(ay_str2, "AccY2:%i", acc_y2);
	sprintf(az_str2, "AccZ2:%i", acc_z2);

	sprintf(bc_str2, "ButtonC2:%i", button_c2);
	sprintf(bz_str2, "ButtonZ2:%i", button_z2);

// Print strings to terminal
	UART_PrintLn(sx_str1);
	UART_PrintLn(sy_str1);
	UART_PrintLn(ax_str1);
	UART_PrintLn(ay_str1);
	UART_PrintLn(az_str1);
	UART_PrintLn(bc_str1);
	UART_PrintLn(bz_str1);

// Print strings to terminal
	UART_PrintLn(sx_str2);
	UART_PrintLn(sy_str2);
	UART_PrintLn(ax_str2);
	UART_PrintLn(ay_str2);
	UART_PrintLn(az_str2);
	UART_PrintLn(bc_str2);
	UART_PrintLn(bz_str2);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
