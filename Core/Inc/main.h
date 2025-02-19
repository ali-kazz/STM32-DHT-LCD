/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  // HAL Library for STM32F4
#include <stdio.h>          // Standard I/O for sprintf()
#include "i2c-lcd.h"        // LCD header
#include "dht.h"            // DHT Sensor header

/* Private defines -----------------------------------------------------------*/
#define DHT_GPIO_Port GPIOB  // Define DHT sensor GPIO port
#define DHT_Pin GPIO_PIN_5   // Define DHT sensor GPIO pin

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Peripheral Initialization */
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM2_Init(void);

/* Utility Functions */
void Display_Temperature_Humidity(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
