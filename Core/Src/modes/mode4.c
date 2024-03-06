
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

	HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_SET); //Enable PLL &
		HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_SET); //Enable RF Stage
		printstr("PLL & RF Stage : ON");
		HAL_Delay(200);

	for (int i=0; i<8; i++){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(40);
	}
	HAL_Delay(500);
	/*
	for (int i=0; i<4; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
		HAL_Delay(500);
	}
	HAL_Delay(1000);
	*/
	//activeMode++;

	myspi(0b00000000000000000010000000100010);
	myspi(0b00000000011101000001100010101111);
	myspi(0b00000011110001000010000001001100);
	myspi(0b00000000000011011111000011001101);

	while (activeMode == 4 ){
		myspi(0b00000000000000000010000000100010);
		__HAL_TIM_SET_COUNTER(&htim21, 0);
		TIM21_increment=0;

		//uint32_t test = ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) );
		//uint32_t test2 = bitNumber*(8000000/1200);
		//sprintf(serialTXbuffer,"%u \n", test);
		//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
		//clearbuffer();
		//myspi(0b00000000000000000010100000100010);
		//HAL_Delay(500);

		//TIM21_value = __HAL_TIM_GET_COUNTER(&htim21);

		int16_t bitNumber=0;
		//int16_t TIM21_tone_increment=0;
		char bits[] = "1010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101001111100110100100001010111011000011110101000100111000001100101110111101010001001110000011001011101001011100110100110011000000011110001110010000100011110000000100111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111110011010010000101011101100001111010100010011100000110010111011110101000100111000001100101110";

		while (bitNumber<500){

			//while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/(1200)) ){
				if (bits[bitNumber] == '1'){
					while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/(1200)) ){
						uint16_t i = 0;
						while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/(2*1200)) ){
							HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						}
					}
					bitNumber++;
				}
				else if (bits[bitNumber] == '0'){
					while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/(1200)) ){
						while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/(2*2200)) ){
							HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						}
						//bitNumber++;
					}
					bitNumber++;
				}
			//}
			bitNumber++;

		}



		/*

		char bits[] = "1010101010111110000000000000001111010101010100000000000000000000000000111111111111111111111111111111111111111111111111110000000000000000000001010101011111111111111111111100000000000000000000000" ;


		for (int16_t bitNumber=0; bitNumber < strlen(bits); bitNumber++){
			TIM21_value = __HAL_TIM_GET_COUNTER(&htim21);

			//sprintf(serialTXbuffer,"%u\n %u\n", TIM21_value, TIM21_increment);
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			//clearbuffer();

			if (bits[bitNumber] == '0'){
				while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/200) ){}
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
				while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/200) ){}
				HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

				//clearbuffer();
				//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
				//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
				//HAL_Delay(1);

			}
			//myspi(0b00000000000000000010100000000010);
			//bitNumber++;
		}

		*/

		myspi(0b00000000000000000010100000000010);

		sprintf(serialTXbuffer,"TxDATA_test() while end\n");
		HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
		clearbuffer();
		HAL_Delay(5000);

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


