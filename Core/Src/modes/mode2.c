/*
 * mode2.c
 *
 *  Created on: Oct 16, 2023
 *      Author: joel
 */
#include "modes.h"
#include "main.h"

void mode2(){
	for (int i=0; i<8; i++){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(40);
	}
	HAL_Delay(500);
	for (int i=0; i<2; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
		HAL_Delay(500);
	}
	//HAL_GPIO_WritePin(DC_BOOST_EN_GPIO_Port, DC_BOOST_EN_Pin, GPIO_PIN_RESET); //STOP DC Boost
	HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET); //STOP PLL &
	HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_RESET); //STOP RF Stage

	uint8_t cnt=0;
	while (activeMode==2) {
		HAL_Delay(2500);
		cnt++;
		if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) ==0 ) {
			HAL_GPIO_WritePin(DC_BOOST_EN_GPIO_Port, DC_BOOST_EN_Pin, GPIO_PIN_SET);
			activeMode++;
		}
		if (cnt==5) {
			HAL_GPIO_WritePin(DC_BOOST_EN_GPIO_Port, DC_BOOST_EN_Pin, GPIO_PIN_RESET); //STOP DC Boost
		}
	}
}
