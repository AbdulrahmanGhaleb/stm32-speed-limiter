/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

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

// GPIO definitions
#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_0
#define DAC1_CS_PORT   GPIOA
#define DAC1_CS_PIN    GPIO_PIN_0
#define DAC2_CS_PORT   GPIOA
#define DAC2_CS_PIN    GPIO_PIN_1
#define MUX_EN_PORT    GPIOC
#define MUX_EN_PIN     GPIO_PIN_9

// ADC Parameters
#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION  4095.0f

// Speed amd CAN related parameters
#define SPEED_CAN_ID   		  0x7E8
#define SPEED_THRESHOLD_KMH   30
#define SPEED_FADE_RANGE_KMH  20

//Pedal processing parameters
#define VoltageRestoreFactor 2.0f
#define preproccsing_percent_current_voltage_Max 1.0f
#define preproccsing_percent_current_voltage_Min 0.0f

//Scaling Factor limits
#define Scale_Factor_Max 1.0f
#define Scale_Factor_Min 0.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// CAN reception
CAN_RxHeaderTypeDef RxHeader;
uint8_t CANResponse[8];
volatile uint16_t CarSpeed = 0;

// UART buffer
char uartBuffer[100];

// ADC raw values
volatile uint16_t adc_val1;
volatile uint16_t adc_val2;

// Pedal channel limits
float PEDAL_CH1_VMIN = 0.5f; //THIS VALUE IS ASSUMED AND NEED TO BE CONFIRMED
float PEDAL_CH1_VMAX = 4.5f; //THIS VALUE IS ASSUMED AND NEED TO BE CONFIRMED
float PEDAL_CH2_VMIN = 0.5f; //THIS VALUE IS ASSUMED AND NEED TO BE CONFIRMED
float PEDAL_CH2_VMAX = 4.5f; //THIS VALUE IS ASSUMED AND NEED TO BE CONFIRMED
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Filter_Config(void);
void ProcessSpeedMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void UART_SendString(char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// CAN Messages filter
void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    // Configure filter for speed messages only
    sFilterConfig.FilterBank = 0;                           // Use filter bank 0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;       // Use ID + Mask mode
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;      // 32-bit filter
    sFilterConfig.FilterIdHigh = (SPEED_CAN_ID << 5);       //
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);          // Mask for exact ID match (standard 11-bit ID)
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;      // Assign to FIFO0
    sFilterConfig.FilterActivation = ENABLE;                // Enable the filter

    // Apply filter configuration
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

// OBD-II PID Speed Request
void RequestSpeed(){
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint32_t TxMailbox;

	TxHeader.StdId = 0x7DF;
	TxHeader.ExtId = 0x00;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }

}

// Speed extraction
void ProcessSpeedMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *CANResponse)
{
	if (rxHeader->StdId == SPEED_CAN_ID) {

		CarSpeed = CANResponse[3];

        snprintf(uartBuffer, sizeof(uartBuffer),
                "Message Received!\r\nID: 0x%03lX\r\nData: %02X %02X %02X %02X %02X %02X %02X %02X\r\n\r\n Speed:%02X\r\n",
                rxHeader->StdId,
				CANResponse[0], CANResponse[1], CANResponse[2], CANResponse[3],
				CANResponse[4], CANResponse[5], CANResponse[6], CANResponse[7],
				CANResponse[3]);

        UART_SendString(uartBuffer);
	}

}

//CAN Response upon interrupt callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CANResponse) == HAL_OK)
    {
        ProcessSpeedMessage(&RxHeader, CANResponse);
    }
}

// Write 12-bit value to DAC via SPI
void DAC_Write(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t value)
{
    value &= 0x0FFF;    // ensure 12-bit value
    uint16_t word = value | (0x0 << 12); // DAC121S101 expects data in lower 12 bits

    uint8_t data[2];
    data[0] = (word >> 8) & 0xFF;
    data[1] = word & 0xFF;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

// Convert ADC value to normalized pedal percentage
float ReadPedalPercent(uint16_t adc_value, float voltage_min, float voltage_max){

	float preproccsing_current_voltage_divided = ((float)adc_value / ADC_RESOLUTION)*ADC_REF_VOLTAGE;
	float preproccsing_current_voltage = preproccsing_current_voltage_divided * VoltageRestoreFactor;
	float preproccsing_percent_current_voltage = (preproccsing_current_voltage - voltage_min) / (voltage_max - voltage_min);

    if (preproccsing_percent_current_voltage <preproccsing_percent_current_voltage_Min) preproccsing_percent_current_voltage = preproccsing_percent_current_voltage_Min;
    if (preproccsing_percent_current_voltage > preproccsing_percent_current_voltage_Max) preproccsing_percent_current_voltage = preproccsing_percent_current_voltage_Max;

    return preproccsing_percent_current_voltage;
}


// Convert pedal percentage to DAC output code
uint16_t PercentToDACValue(float percent, float voltage_min, float voltage_max) {

    float afterprocessing_voltage = voltage_min + (percent * (voltage_max - voltage_min)); //Percentage to voltage
    uint16_t dac_value = (uint16_t)((afterprocessing_voltage / ADC_REF_VOLTAGE) * ADC_RESOLUTION); //DAC expects digital value

    if (dac_value > ADC_RESOLUTION ) dac_value = ADC_RESOLUTION ;
    return dac_value;
}

// Apply pedal attenuation based on vehicle speed
void AttenuationFunction(void){

	float percent1, percent2, output_percent1, output_percent2;
	percent1 = ReadPedalPercent(adc_val1, PEDAL_CH1_VMIN, PEDAL_CH1_VMAX); //Converts the raw value from adc to percentage
	percent2 = ReadPedalPercent(adc_val2, PEDAL_CH2_VMIN, PEDAL_CH2_VMAX); //Converts the raw value from adc to percentage

    if(CarSpeed >= SPEED_THRESHOLD_KMH) {

        float overspeed = CarSpeed - SPEED_THRESHOLD_KMH;
        float scale_factor = Scale_Factor_Max - (overspeed / (float)SPEED_FADE_RANGE_KMH);
        if (scale_factor < Scale_Factor_Min) scale_factor = Scale_Factor_Min;
        output_percent1 = percent1 * scale_factor;
        output_percent2 = percent2 * scale_factor;
    } else {

        output_percent1 = percent1;
        output_percent2 = percent2;
    }

    uint16_t dac_value_1 = PercentToDACValue(output_percent1, PEDAL_CH1_VMIN, PEDAL_CH1_VMAX); //Converts percentage back to analog
    uint16_t dac_value_2 = PercentToDACValue(output_percent2, PEDAL_CH2_VMIN, PEDAL_CH2_VMAX); //Converts percentage back to analog

    DAC_Write(DAC1_CS_PORT, DAC1_CS_PIN, dac_value_1);
    DAC_Write(DAC2_CS_PORT, DAC2_CS_PIN, dac_value_2);

}

void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
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
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(MUX_EN_PORT, MUX_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DAC1_CS_PORT, DAC1_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DAC2_CS_PORT, DAC2_CS_PIN, GPIO_PIN_SET);

  HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)&adc_val1, 1);
  HAL_ADC_Start_DMA(&hadc2,  (uint32_t*)&adc_val2, 1);

  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan1) != HAL_OK) //Starts CAN peripheral
  {
      Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) //Enable interrupts
  {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	RequestSpeed();
	HAL_Delay(100); //Sending too fast causes STM CAN to fault out.

	AttenuationFunction();
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
