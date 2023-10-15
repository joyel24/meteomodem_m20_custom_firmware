/*
 * mode0.c
 *
 *  Created on: Oct 15, 2023
 *      Author: joel
 */
#include "modes.h"
#include "main.h"
#include "stm32l0xx_hal.h"

void mode0(){
	HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	sprintf(serialTXbuffer,"mode0()\nTxDATA:\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	HAL_Delay(100);
	//HAL_UART_Transmit_IT(&huart1, serialTXbuffer, sizeof (serialTXbuffer));
	while (activeMode == 0){


		//while ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin == 0) ) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
			clearbuffer();
			sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			HAL_Delay(500);
		//}

		//HAL_GPIO_WritePin(GPS_Heater_LDO_EN_GPIO_Port, GPS_Heater_LDO_EN_Pin, GPIO_PIN_RESET); //Disable GPS & Heater

		if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0 ) {
			sprintf(serialTXbuffer,"\nswitch mode request\n");
			HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			clearbuffer();
			HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET); //Disable PLL &
			HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_RESET);
			printstr("PLL & RF Stage : OFF");
			activeMode++;
		}
	}
}
