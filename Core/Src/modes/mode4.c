/*
 * mode4.c
 *
 *  Created on: Oct 16, 2023
 *      Author: joel
 */
#include "modes.h"
#include "main.h"

void mode4(){
	printstr("mode4()");

	HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET); //Disable PLL &
	HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_RESET);
	printstr("PLL & RF Stage : OFF");

	for (int i=0; i<8; i++){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(40);
	}
	HAL_Delay(500);
	for (int i=0; i<4; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
		HAL_Delay(500);
	}
	HAL_Delay(1000);

	//activeMode++;
}
