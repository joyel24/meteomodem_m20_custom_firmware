/*
 * mode1.c
 *
 *  Created on: Oct 16, 2023
 *      Author: joel
 */
#include "modes.h"
#include "main.h"

void mode1(){
	HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	sprintf(serialTXbuffer,"mode1()\nTxDATA:\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	HAL_Delay(100);
	//HAL_UART_Transmit_IT(&huart1, serialTXbuffer, sizeof (serialTXbuffer));

	for (int i=0; i<8; i++){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(40);
		}
	HAL_Delay(500);
	for (int i=0; i<1; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
		HAL_Delay(500);
	}


	HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_SET); //Enable PLL &
	HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_SET); //Enable RF Stage
	printstr("PLL & RF Stage : ON");
	HAL_Delay(200);

	//403,5 MHz
	myspi(0b00000011110001000010000001001100);/* Reg 0 R Register
	Output divider = devide 2 0b01
	VCO Adjustment = Max VCO Adj 0b11
	Clock out divider = 0b1000 = 16
	XOEB = 1 (XTAL Osc Off)
	Crystal doubler OFF
	4bit R div = 0b001 = 1
	11Bit Freq err corr 0b10011
	*/
	myspi(0b00000000000011001001110000000001);/*
	Prescaler = 0b0 = 4/5
	8Bits integer N = 0b110010
	12bits factional N = 0b011100000000

	*/
	myspi(0b00000000000000000101111111100010);/*
	Modulation register
	Index counter 0b00 16
	GFSK Mod 0b000 0
	Modulation deviation ????
	Power ampli 0b111111
	GOOK 0 = Gaussian OOK = Off
	Mod control 0b00 FSK
	*/
	myspi(0b00000000011101000001100010101111);/*
	PA BIAS 0b111 = 12uA
	VCO BIAS current 0b0100
	LD1 0b0 3 Cycles
	MUXOUT = 0b0011 = Regulator ready
	VCO Disable = 0b0 = VCO ON
	Bleed Down up 0b0 0b0 off off
	Charge pump = 0b10 1.5mA
	Data invert = 0b1 = inverted
	clkout enable = 0b0 = Off
	PA Enable = 0b1 PA on
	PLL Enable = 0b1 PLL On
	*/

	myspi(0b00000011110001000010000001001100);

	//myspi(setfreq(433920000, 8000000, 0));
	myspi(0b00000000000011011000111101011101);

	myspi(0b00000000000000000010100000100010);


	while (activeMode == 1){


		//while ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin == 0) ) {
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			//HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
			//clearbuffer();
			//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			//HAL_Delay(2);
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

		TxDATA_test2("1010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101001111100110100100001010111011000011110101000100111000001100101110111101010001001110000011001011101001011100110100110011000000011110001110010000100011110000000100111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111101010001001110000011001011101111010100010011100000110010111011110101000100111000001100101110111110011010010000101011101100001111010100010011100000110010111011110101000100111000001100101110");

	}
}






void TxDATA_AFSK_TEST(char bits[]){
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

		sprintf(serialTXbuffer,"%u \n", strlen(bits));
				HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
				clearbuffer();
		printstr(bits);

		for (int16_t bitNumber=0; bitNumber < 1120; bitNumber++){
			TIM21_value = __HAL_TIM_GET_COUNTER(&htim21);

			//sprintf(serialTXbuffer,"%u\n %u\n", TIM21_value, TIM21_increment);
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			//clearbuffer();

			if (bits[bitNumber] == '0'){
				while ( ( (TIM21_increment * 65535) + __HAL_TIM_GET_COUNTER(&htim21) ) < bitNumber*(8000000/1200) ){}
				HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

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
				//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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



/////////////////////////////
void TxDATA_test2(char bits[]){
	//if (stop==0){
	sprintf(serialTXbuffer,"TxDATA_test() begin\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();

	uint8_t activeModeTemp = activeMode;


	while (activeModeTemp == activeMode) {
		myspi(0b00000000000000000010100000100010);

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
		myspi(0b00000000000000000010100000100010);
		//HAL_Delay(500);
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
		myspi(0b00000000000000000010100000000010);

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
