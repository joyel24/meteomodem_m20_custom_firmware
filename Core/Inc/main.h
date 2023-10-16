/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t serialTXbuffer[2000];
extern void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
extern UART_HandleTypeDef huart1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define RF_FINAL_STAGE_EN_Pin GPIO_PIN_12
#define RF_FINAL_STAGE_EN_GPIO_Port GPIOB
#define ADF7012_TxDATA_Pin GPIO_PIN_13
#define ADF7012_TxDATA_GPIO_Port GPIOB
#define GPS_Heater_LDO_EN_Pin GPIO_PIN_14
#define GPS_Heater_LDO_EN_GPIO_Port GPIOB
#define PLL_UnkownIC_LDO_EN_Pin GPIO_PIN_15
#define PLL_UnkownIC_LDO_EN_GPIO_Port GPIOB
#define ADF7012_CLK_Pin GPIO_PIN_7
#define ADF7012_CLK_GPIO_Port GPIOC
#define ADF7012_DATA_Pin GPIO_PIN_8
#define ADF7012_DATA_GPIO_Port GPIOC
#define ADF7012_LE_Pin GPIO_PIN_9
#define ADF7012_LE_GPIO_Port GPIOC
#define MASTER_CLK_OUT_PLL_Pin GPIO_PIN_8
#define MASTER_CLK_OUT_PLL_GPIO_Port GPIOA
#define DC_BOOST_EN_Pin GPIO_PIN_12
#define DC_BOOST_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/*
#define Pin4_Pin GPIO_PIN_0
#define Pin4_GPIO_Port GPIOA
#define Pin13____Pin GPIO_PIN_3
#define Pin13____GPIO_Port GPIOA
#define Pin13_____Pin GPIO_PIN_1
#define Pin13_____GPIO_Port GPIOB
#define Pin9_Pin GPIO_PIN_2
#define Pin9_GPIO_Port GPIOB
#define Pin7_Pin GPIO_PIN_10
#define Pin7_GPIO_Port GPIOB
#define Pin6_Pin GPIO_PIN_11
#define Pin6_GPIO_Port GPIOB
#define TXDATA_Pin GPIO_PIN_13
#define TXDATA_GPIO_Port GPIOB
#define GPS_Supply_Pin GPIO_PIN_14
#define GPS_Supply_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_7
#define CLK_GPIO_Port GPIOC
#define DATA_Pin GPIO_PIN_8
#define DATA_GPIO_Port GPIOC
#define LE_Pin GPIO_PIN_9
#define LE_GPIO_Port GPIOC
#define OSC2_Pin GPIO_PIN_8
#define OSC2_GPIO_Port GPIOA
#define Main_Supply_Pin GPIO_PIN_12
#define Main_Supply_GPIO_Port GPIOA
#define GPS_Data_Pin GPIO_PIN_11
#define GPS_Data_GPIO_Port GPIOC
*/
extern uint8_t activeMode;
extern uint8_t maxActiveMode;
extern uint16_t	TIM21_increment;
extern uint16_t	TIM21_value;
extern TIM_HandleTypeDef htim21;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
