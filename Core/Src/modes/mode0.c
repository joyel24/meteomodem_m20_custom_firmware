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
	sprintf(serialTXbuffer,"mode0\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, 10, sizeof (serialTXbuffer));
	HAL_Delay(50);
			//HAL_UART_Transmit_IT(&huart1, serialTXbuffer, sizeof (serialTXbuffer));
	while (activeMode == 0){


		//while ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin == 0) ) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
			//serialTXbuffer[0:2000]="";
			//serialTXbuffer[0] = '\0';
			for (int i=0; i < sizeof(serialTXbuffer); i++){
				serialTXbuffer[i]='\0';
			}
			//free(serialTXbuffer);
			sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			HAL_UART_Transmit(&huart1, serialTXbuffer, 10, sizeof (serialTXbuffer));
			HAL_Delay(500);
		//}

		//HAL_GPIO_WritePin(GPS_Heater_LDO_EN_GPIO_Port, GPS_Heater_LDO_EN_Pin, GPIO_PIN_RESET); //Disable GPS & Heater
		//HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET); //Disable PLL &

		if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0 ) {
			activeMode++;
			sprintf(serialTXbuffer,"mode1\n");
			HAL_UART_Transmit(&huart1, serialTXbuffer, 10, sizeof (serialTXbuffer));
		}
	}
}
