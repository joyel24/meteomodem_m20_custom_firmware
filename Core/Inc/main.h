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
#define PLL_UnkownIC_LDO_EN_Pin GPIO_PIN_15
#define PLL_UnkownIC_LDO_EN_GPIO_Port GPIOB
#define ADF7012_CLK_Pin GPIO_PIN_7
#define ADF7012_CLK_GPIO_Port GPIOC
#define ADF7012_DATA_Pin GPIO_PIN_8
#define ADF7012_DATA_GPIO_Port GPIOC
#define ADF7012_LE_Pin GPIO_PIN_9
#define ADF7012_LE_GPIO_Port GPIOC
#define DC_BOOST_EN_Pin GPIO_PIN_12
#define DC_BOOST_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
