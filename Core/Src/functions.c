/*
 * functions.c
 *
 *  Created on: Oct 15, 2023
 *      Author: joel
 */

#include "functions.h"
#include "stm32l0xx_hal.h"
#include "main.h"


void printstr(char test[]){
	sprintf(serialTXbuffer,"%s \n", test);
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();
}


void TxDATA_test(char bits[]){
	//if (stop==0){
	sprintf(serialTXbuffer,"TxDATA_test() begin\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();

	uint8_t activeModeTemp = activeMode;


	while (activeModeTemp == activeMode) {
		sprintf(serialTXbuffer,"TxDATA_test() while begin\n");
		HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
		clearbuffer();

		__HAL_TIM_SET_COUNTER(&htim21, 0);
		TIM21_increment=0;

		uint32_t test = ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) );
		//uint32_t test2 = bitNumber*(8000000/1200);
		sprintf(serialTXbuffer,"%u \n", test);
		HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
		clearbuffer();
		for (int16_t bitNumber=0; bitNumber < 1120; bitNumber++){
			TIM21_value = __HAL_TIM_GET_COUNTER(&htim21);

			//sprintf(serialTXbuffer,"%u\n %u\n", TIM21_value, TIM21_increment);
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			//clearbuffer();

			if (bits[bitNumber] == '0'){
				while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/1200) ){}
				HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

				//HAL_UART_Transmit(&huart1, UartBuffOut, strlen(UartBuffOut), 1000);
				//clearbuffer();
				//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
				//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
				//HAL_Delay(1);
				}
			else if(bits[bitNumber] == '1'){
				//HAL_UART_Transmit(&huart1, UartBuffOut, strlen(UartBuffOut), 1000);
				while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/1200) ){}
				HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

				//clearbuffer();
				//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
				//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
				//HAL_Delay(1);

			}
			//bitNumber++;

		}
		sprintf(serialTXbuffer,"TxDATA_test() while end\n");
		HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
		clearbuffer();
		HAL_Delay(2500);

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

	sprintf(serialTXbuffer,"TxDATA_test() end\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();
}

void myspi(uint32_t data)
{
	//uint32_t delay = 0;
	HAL_GPIO_WritePin(ADF7012_LE_GPIO_Port, ADF7012_LE_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF7012_DATA_GPIO_Port, ADF7012_DATA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF7012_CLK_GPIO_Port, ADF7012_CLK_Pin, GPIO_PIN_RESET);
	//HAL_Delay(delay);
	__HAL_RCC_GPIOA_CLK_ENABLE();
	for (int i = 0; i < 32; i++) {
		HAL_GPIO_WritePin(ADF7012_CLK_GPIO_Port, ADF7012_CLK_Pin, GPIO_PIN_RESET);
		if (data & 0b10000000000000000000000000000000)
		{
			HAL_GPIO_WritePin(ADF7012_DATA_GPIO_Port, ADF7012_DATA_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(ADF7012_DATA_GPIO_Port, ADF7012_DATA_Pin, GPIO_PIN_RESET);
		}
		//HAL_Delay(delay);
		HAL_GPIO_WritePin(ADF7012_CLK_GPIO_Port, ADF7012_CLK_Pin, GPIO_PIN_SET);
		data = data << 1;
	}
	HAL_GPIO_WritePin(ADF7012_LE_GPIO_Port, ADF7012_LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF7012_LE_GPIO_Port, ADF7012_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF7012_CLK_GPIO_Port, ADF7012_CLK_Pin, GPIO_PIN_RESET);

}
