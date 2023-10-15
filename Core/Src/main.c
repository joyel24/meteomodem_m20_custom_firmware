/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "modes.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t activeMode = 2;
uint8_t maxActiveMode = 5;
uint8_t serialTXbuffer[2000];

//extern void mode0();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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

uint32_t setfreq(float freq, float fPFD, uint8_t prescaler)
{
	float latch = freq / fPFD;
	uint32_t Nint = latch;
	latch = latch - Nint;
	uint32_t Nfrac = latch * 4096;
	uint32_t ret = 0;
	ret = ret | prescaler << 22;
	ret = ret | Nint << 14;
	ret = ret | Nfrac << 2;
	ret = ret | 0b00000001;
	return ret;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   HAL_UART_Transmit_IT(&huart1, serialTXbuffer, sizeof (serialTXbuffer));
}

void clearbuffer(){
	for (int i=0; i < sizeof(serialTXbuffer); i++){
		serialTXbuffer[i]='\0';
	}
}

void printstr(char test[]){
	sprintf(serialTXbuffer,"%s \n", test);
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();
}

void mode1(){
	HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	sprintf(serialTXbuffer,"mode1()\nTxDATA:\n");
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	HAL_Delay(100);
	//HAL_UART_Transmit_IT(&huart1, serialTXbuffer, sizeof (serialTXbuffer));
	while (activeMode == 1){


		//while ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin == 0) ) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_GPIO_TogglePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin);
			clearbuffer();
			sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			HAL_Delay(100);
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

void mode3(){
	sprintf(serialTXbuffer,"mode3()\nTxDATA:\n");
	HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	HAL_Delay(100);

	while (activeMode == 3){
		TxDATA_test("10101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101011011000000101011101001001111100100101111100000110001001011110101001011111000001100010010111101010100101000111010000000000000000111000000110101110100110100010111111010001010001111110111100110101101001000001110000000010000000100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010100101111100000110001001011110101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101101100000010101110100100111110010010111110000011000100101111010100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010101001010001110100000000000000001000011010011111111101101111011111110001001000011001001111001101100000000000010100000000111010101001011111000001100010010111101010010111110000011000100101111010100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010100101111100000110001001011110101001011111000001100010010111101010010111110000011000100101111010");
		HAL_Delay(1500);
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

void TxDATA_test(char bits[]){
	//if (stop==0){
	for (int16_t bitNumber=0; bitNumber < 1120; bitNumber++){
		if (bits[bitNumber] == '0'){
			HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_RESET);

			//HAL_UART_Transmit(&huart1, UartBuffOut, strlen(UartBuffOut), 1000);
			//clearbuffer();
			//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			HAL_Delay(1);
			}
		else if(bits[bitNumber] == '1'){
			//HAL_UART_Transmit(&huart1, UartBuffOut, strlen(UartBuffOut), 1000);
			HAL_GPIO_WritePin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin, GPIO_PIN_SET);
			//clearbuffer();
			//sprintf(serialTXbuffer,"%d", HAL_GPIO_ReadPin(ADF7012_TxDATA_GPIO_Port, ADF7012_TxDATA_Pin));
			//HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
			HAL_Delay(1);
		}
		//bitNumber++;
	}
}

	/*
		if (bitNumber >= 1120){
			HAL_GPIO_WritePin(PLL_UnkownIC_LDO_EN_GPIO_Port, PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET); //Disable PLL &
			HAL_GPIO_WritePin(RF_FINAL_STAGE_EN_GPIO_Port, RF_FINAL_STAGE_EN_Pin, GPIO_PIN_RESET);
			printstr("PLL & RF Stage : OFF");
	//		stop=1; bitNumber=0;}
	 *
	//}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(DC_BOOST_EN_GPIO_Port, DC_BOOST_EN_Pin, GPIO_PIN_SET); //Enable DC Boost

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	printstr("\n\nMain While");

	sprintf(serialTXbuffer,"activeMode=%d\n", activeMode);
	HAL_UART_Transmit(&huart1, serialTXbuffer, sizeof (serialTXbuffer), sizeof (serialTXbuffer));
	clearbuffer();

  	if (activeMode > maxActiveMode){activeMode=0;}

		switch (activeMode)
		{
			case 0:
				printstr("case 0");
				//HAL_GPIO_WritePin(GPS_Heater_LDO_EN_GPIO_Port, GPS_Heater_LDO_EN_Pin, GPIO_PIN_SET); //Enable GPS & Heater
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
				myspi(setfreq(446100000, 8000000, 0));
				myspi(0b00000000000000000010110011100010);

				mode0();
				break;

			case 1:

				printstr("case 1");
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
				myspi(setfreq(446100000, 8000000, 0));
				myspi(0b00000000000000000010110011100010);

				mode1();
				/*
				while (activeMode==1) {
					HAL_Delay(500);
					if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) ==0 ) { activeMode++; }
				}*/
				break;
			case 2:

				printstr("case 2 not yet programmed (off tx)");
				while (activeMode==2) {
					HAL_Delay(500);
					if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) ==0 ) { activeMode++; }
				}
				break;
			case 3:
				printstr("case 3");
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
				myspi(setfreq(446100000, 8000000, 0));
				myspi(0b00000000000000000010110011100010);

				mode3();
				break;
			case 4:
				printstr("case 4 > Switch to case 0");
				activeMode=0;
				break;
			default:
				break;
		}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|ADF7012_CLK_Pin|ADF7012_DATA_Pin|ADF7012_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_FINAL_STAGE_EN_Pin|ADF7012_TxDATA_Pin|GPS_Heater_LDO_EN_Pin|PLL_UnkownIC_LDO_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_BOOST_EN_GPIO_Port, DC_BOOST_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin ADF7012_CLK_Pin ADF7012_DATA_Pin ADF7012_LE_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ADF7012_CLK_Pin|ADF7012_DATA_Pin|ADF7012_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_FINAL_STAGE_EN_Pin ADF7012_TxDATA_Pin GPS_Heater_LDO_EN_Pin PLL_UnkownIC_LDO_EN_Pin */
  GPIO_InitStruct.Pin = RF_FINAL_STAGE_EN_Pin|ADF7012_TxDATA_Pin|GPS_Heater_LDO_EN_Pin|PLL_UnkownIC_LDO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MASTER_CLK_OUT_PLL_Pin */
  GPIO_InitStruct.Pin = MASTER_CLK_OUT_PLL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MASTER_CLK_OUT_PLL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_BOOST_EN_Pin */
  GPIO_InitStruct.Pin = DC_BOOST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DC_BOOST_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
