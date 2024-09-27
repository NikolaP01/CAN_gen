/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];

uint8_t lcd_thermistor = 1;
uint16_t msg_flag = 0x00;
int16_t adc1, adc2, adc3;
uint8_t led = 0x00;
uint8_t light_source = 0x01;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void ADC_Select_CH0(void);
void ADC_Select_CH1(void);
void ADC_Select_CH2(void);
void ADC_Select_CH3(void);
void ADC_Read(int16_t* pValue1, int16_t* pValue2, int16_t* pValue3, int16_t* pValue4);
void ADC_Read1(int16_t* pValue1, int16_t* pValue2, int16_t* pValue3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 12;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  //canfilterconfig.FilterIdHigh = 0x103<<5;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  //canfilterconfig.FilterMaskIdHigh = 0x1<<13;
  canfilterconfig.FilterMaskIdHigh = 0x0;
  canfilterconfig.FilterMaskIdLow = 0x0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(
		  &hcan,
          CAN_IT_TX_MAILBOX_EMPTY |
          CAN_IT_RX_FIFO0_MSG_PENDING |
          CAN_IT_RX_FIFO0_FULL |
          CAN_IT_RX_FIFO0_OVERRUN |
          CAN_IT_RX_FIFO1_MSG_PENDING |
          CAN_IT_RX_FIFO1_FULL |
          CAN_IT_RX_FIFO1_OVERRUN |
          CAN_IT_WAKEUP |
          CAN_IT_SLEEP_ACK |
          CAN_IT_ERROR_WARNING |
          CAN_IT_ERROR_PASSIVE |
          CAN_IT_BUSOFF |
          CAN_IT_LAST_ERROR_CODE |
          CAN_IT_ERROR
  );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //promenljive za kvazi-tajmere
  //sluze za izvrsavanje odredjenih delova koda svaki sekund bez prekidne rutine
  uint32_t now = 0, last_blink = 0, last_tx = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  now = HAL_GetTick();

	  	  if(led == 0x01) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  	  else if(led == 0x00) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

	  	  if(msg_flag == 500){
	  		  //zatrezene sve analogne vrednosti i stanje diode
	  		  TxHeader.DLC = 7;
	  		  TxHeader.ExtId = 0;
	  		  TxHeader.IDE = CAN_ID_STD;
	  		  TxHeader.RTR = CAN_RTR_DATA;
	  		  TxHeader.StdId = 0x400;
	  		  TxHeader.TransmitGlobalTime = DISABLE;
	  		  TxData[0] = adc1 >> 8;
	  		  TxData[1] = adc1 & 0xFF;
	  		  TxData[2] = adc2 >> 8;
	  		  TxData[3] = adc2 & 0xFF;
	  		  TxData[4] = adc3 >> 8;
	  		  TxData[5] = adc3 & 0xFF;
	  		  TxData[6] = led;
	  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		  {
	  		  	Error_Handler();
	  		  }
	  		  msg_flag = 0;
	  	  }
	  	  else if(msg_flag == 501){
	  		  //zatrazena vrednost potenciometra
	  		  TxHeader.DLC = 2;
	  		  TxHeader.ExtId = 0;
	  		  TxHeader.IDE = CAN_ID_STD;
	  		  TxHeader.RTR = CAN_RTR_DATA;
	  		  TxHeader.StdId = 0x401;
	  		  TxHeader.TransmitGlobalTime = DISABLE;
	  		  TxData[0] = adc1 >> 8;
	  		  TxData[1] = adc1 & 0xFF;
	  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		  {
	  		  	Error_Handler();
	  		  }
	  		  msg_flag = 0;
	  	  }
	  	  else if(msg_flag == 502){
	  		  //zatrazena vrednost prvog termistora
	  		  TxHeader.DLC = 2;
	  		  TxHeader.ExtId = 0;
	  		  TxHeader.IDE = CAN_ID_STD;
	  		  TxHeader.RTR = CAN_RTR_DATA;
	  		  TxHeader.StdId = 0x402;
	  		  TxHeader.TransmitGlobalTime = DISABLE;
	  		  TxData[0] = adc2 >> 8;
	  		  TxData[1] = adc2 & 0xFF;
	  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		  {
	  		  	Error_Handler();
	  		  }
	  		  msg_flag = 0;
	  	  }
	  	  else if(msg_flag == 503){
	  		  //zatrazena vrednost drugog termistora
	  		  TxHeader.DLC = 2;
	  		  TxHeader.ExtId = 0;
	  		  TxHeader.IDE = CAN_ID_STD;
	  		  TxHeader.RTR = CAN_RTR_DATA;
	  		  TxHeader.StdId = 0x403;
	  		  TxHeader.TransmitGlobalTime = DISABLE;
	  		  TxData[0] = adc3 >> 8;
	  		  TxData[1] = adc3 & 0xFF;
	  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		  {
	  		  	Error_Handler();
	  		  }
	  		  msg_flag = 0;
	  	  }
	  	  else if(msg_flag == 504){
	  		  //zatrazeno stanje diode
	  		  TxHeader.DLC = 1;
	  		  TxHeader.ExtId = 0;
	  		  TxHeader.IDE = CAN_ID_STD;
	  		  TxHeader.RTR = CAN_RTR_DATA;
	  		  TxHeader.StdId = 0x404;
	  		  TxHeader.TransmitGlobalTime = DISABLE;
	  		  TxData[0] = led;
	  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		  {
	  		  	Error_Handler();
	  		  }
	  		  msg_flag = 0;
	  	  }

	  	  if (now - last_tx >= 1000) {
	  		  //poruka koja se salje svaki sekund
	  		  //salje se vrednost potenciometra jednog od 2 termistora

	  		  //ako se izvor svetlosti podesi na termistore onda se salje
	  		  //vrednost oba termistora a potenciometar se izostavlja

	  		  ADC_Read1(&adc1, &adc2, &adc3);//proveri sve ADC vrednosti

	  		  if(msg_flag == 0){
	  			  TxHeader.DLC = 4;
	  			  TxHeader.ExtId = 0;
	  			  TxHeader.IDE = CAN_ID_STD;
	  			  TxHeader.RTR = CAN_RTR_DATA;
	  			  TxHeader.StdId = 0x605;
	  			  TxHeader.TransmitGlobalTime = DISABLE;
	  			  if(light_source == 0x01){
	  				  TxData[0] = adc1 >> 8; //posalji prvih 8 bita
	  				  TxData[1] = adc1 & 0xFF; //posalji zadnjih
	  				  if(lcd_thermistor == 1){
		  				  TxData[2] = adc2 >> 8; //posalji prvih 8 bita
		  				  TxData[3] = adc2 & 0xFF; //posalji zadnjih
		  			  }
		  			  else if(lcd_thermistor == 2){
		  				  TxData[2] = adc3 >> 8; //posalji prvih 8 bita
		  				  TxData[3] = adc3 & 0xFF; //posalji zadnjih
		  			  }
	  			  }
	  			  else{
	  				  TxData[0] = adc2 >> 8; //posalji prvih 8 bita
	  				  TxData[1] = adc2 & 0xFF; //posalji zadnjih
	  				  TxData[2] = adc3 >> 8;
	  				  TxData[3] = adc3 & 0xFF;
	  			  }
	  			  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  			  {
	  				  Error_Handler();
	  			  }
	  		  }

	  	      last_tx = now;
	  	  }

	  	  if (now - last_blink >= 500) {
	  		  last_blink = now;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_Select_CH0 (void)
{
	//odabir kanala 0 za AD konverziju
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH3 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//na osnovu pristiglog zahteva za podatke postavi se odgovarajuci flag
	//flag odredjuje koja poruka se salje u glavnoj petlji
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	if(RxHeader.StdId == 0x301){
    		//postavi izvor svetlosti na trazeni
    		light_source = RxData[0];
    	}
    	if(RxHeader.StdId == 0x302){
    		//postavi izvor svetlosti na trazeni
    		led = RxData[0];
    	}
    	else if(RxHeader.StdId == 0x500)
    	{
    		msg_flag = 500;
    	}
    	else if(RxHeader.StdId == 0x501)
    	{
    		msg_flag = 501;
    	}
    	else if(RxHeader.StdId == 0x502)
    	{
    		msg_flag = 502;
    	}
    	else if(RxHeader.StdId == 0x503)
    	{
    		msg_flag = 503;
    	}
    	else if(RxHeader.StdId == 0x504)
    	{
    		msg_flag = 504;
    	}
    	else if(RxHeader.StdId == 0x601){
    		lcd_thermistor = 1;
    	}
    	else if(RxHeader.StdId == 0x602){
    		lcd_thermistor = 2;
    	}
    }
}

void ADC_Read(int16_t* pValue1, int16_t* pValue2, int16_t* pValue3, int16_t* pValue4) {
	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		*pValue1 = HAL_ADC_GetValue(&hadc1); // Read first channel value
	    // Poll again to ensure the second channel conversion is complete
	    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
	    	*pValue2 = HAL_ADC_GetValue(&hadc1); // Read second channel value
	        // Poll again to ensure the third channel conversion is complete
	        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
	        	*pValue3 = HAL_ADC_GetValue(&hadc1); // Read second channel value
	            //Poll again to ensure the fourth channel conversion is complete
	            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
	            	*pValue4 = HAL_ADC_GetValue(&hadc1); // Read second channel value
	            }
	        }
	    }
	}
}

//funkcija za citanje svih analognih vrednosti
void ADC_Read1(int16_t* pValue1, int16_t* pValue2, int16_t* pValue3) {
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*pValue1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*pValue2 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*pValue3 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	*pValue1 = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
}
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
