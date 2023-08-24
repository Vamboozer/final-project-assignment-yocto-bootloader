/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "app_openbootloader.h"
#include "interfaces_conf.h"
#include "spi_interface.h"
#include "stm32l4xx_it.h"
#include "openbl_spi_cmd.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
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
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

extern TIM_HandleTypeDef htim3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void PRINTF(const char *format, ...);
void SetRFBoardToSafeState(void);
uint32_t mVTo12bitDigitalOutput(uint32_t milliVolt);
uint32_t mVTo12bitDigitalOutput_Halfx(uint32_t milliVolt);
void Spi2_UpdateRxDac(ExternDacChannel_e Channel, uint16_t DacValue);
void Spi3_UpdateTxDac(ExternDacChannel_e Channel, uint16_t DacValue);

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
  MX_DAC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize RF Board Hardware to Safe State
  SetRFBoardToSafeState();

  /* Initialize the OpenBootloader */
  OpenBootloader_Init();

  PRINTF("\033[2J"); // Clear the terminal
  PRINTF("\033[s\033[1;1H"); // Save cursor position, return to top left corner
  PRINTF("Running Bootloader Version %d.%d ...\n\r", ((OPENBL_SPI_VERSION >> 4) & 0x0F), (OPENBL_SPI_VERSION & 0x0F));

  // Alert SOM that the RFB is in a safe state and we're ready to bootload
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);

  while(OPENBL_SPI_ReadByte() != 0x5AU){} // Blocking call -> Wait for SOM to be ready

  SPI_SetSpiDetected(1U);
  OPENBL_SPI_SendByte(SYNC_BYTE); // Send synchronization byte
  OPENBL_SPI_SendAcknowledgeByte(ACK_BYTE); // Send acknowledgment
  OPENBL_InterfaceDetection();
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET); // Deassert busy flag until needed again
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    // Process the commands from the detected interface
    OPENBL_CommandProcess();
    /* USER CODE END WHILE */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPO_TP17_Pin|GPIO2_Pin|GPIO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPO_SBANDRX_Pin|SPI2_NSS_RxSynth_Pin|SPI2_NSS_RxDAC_Pin|GPO_RXPDEN_Pin
                          |GPO_RXLNASD_Pin|GPO_RSEL2n_Pin|GPO_RSEL3n_Pin|GPO_RSEL0n_Pin
                          |GPO_LPBK0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_IqDemod_GPIO_Port, SPI2_NSS_IqDemod_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPO_TP14_Pin|GPO_TP15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPO_DEN_Pin|GPO_RXLOEN_Pin|GPO_PHVRXEN_Pin|GPO_HEATON_Pin
                          |GPO_RSEL1n_Pin|GPO_LPBK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPO_MEN_Pin|GPO_PHVTXEN_Pin|GPO_TXPDEN_Pin|GPO_TSEL1n_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPO_SBANDTX_Pin|GPO_TSEL0n_Pin|GPO_TSEL2n_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_TxSynth_GPIO_Port, SPI3_NSS_TxSynth_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_AR_GPIO_Port, SPI3_NSS_AR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPO_TXLOEN_Pin|GPO_STXDEN_Pin|GPO_CTXDEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPO_TPX_Pin|GPO_TP16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPO_TP17_Pin GPIO2_Pin GPIO3_Pin */
  GPIO_InitStruct.Pin = GPO_TP17_Pin|GPIO2_Pin|GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : GPO_SBANDRX_Pin GPO_DEN_Pin GPO_RXLOEN_Pin SPI2_NSS_RxSynth_Pin
                           SPI2_NSS_RxDAC_Pin GPO_RXPDEN_Pin GPO_RXLNASD_Pin GPO_PHVRXEN_Pin
                           GPO_HEATON_Pin GPO_RSEL2n_Pin GPO_RSEL3n_Pin GPO_RSEL1n_Pin
                           GPO_RSEL0n_Pin GPO_LPBK1_Pin GPO_LPBK0_Pin */
  GPIO_InitStruct.Pin = GPO_SBANDRX_Pin|GPO_DEN_Pin|GPO_RXLOEN_Pin|SPI2_NSS_RxSynth_Pin
                          |SPI2_NSS_RxDAC_Pin|GPO_RXPDEN_Pin|GPO_RXLNASD_Pin|GPO_PHVRXEN_Pin
                          |GPO_HEATON_Pin|GPO_RSEL2n_Pin|GPO_RSEL3n_Pin|GPO_RSEL1n_Pin
                          |GPO_RSEL0n_Pin|GPO_LPBK1_Pin|GPO_LPBK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_IqDemod_Pin GPO_TP14_Pin GPO_TP15_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_IqDemod_Pin|GPO_TP14_Pin|GPO_TP15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_MPAEN_Pin */
  GPIO_InitStruct.Pin = GPI_MPAEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPI_MPAEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPO_MEN_Pin GPO_SBANDTX_Pin GPO_PHVTXEN_Pin GPO_TSEL0n_Pin
                           GPO_TXPDEN_Pin GPO_TSEL2n_Pin GPO_TSEL1n_Pin */
  GPIO_InitStruct.Pin = GPO_MEN_Pin|GPO_SBANDTX_Pin|GPO_PHVTXEN_Pin|GPO_TSEL0n_Pin
                          |GPO_TXPDEN_Pin|GPO_TSEL2n_Pin|GPO_TSEL1n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_NSS_TxDAC_Pin GPO_TXLOEN_Pin GPO_STXDEN_Pin GPO_CTXDEN_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_TxDAC_Pin|GPO_TXLOEN_Pin|GPO_STXDEN_Pin|GPO_CTXDEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_DO_NOT_USE_Pin */
  GPIO_InitStruct.Pin = GPI_DO_NOT_USE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPI_DO_NOT_USE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_NSS_TxSynth_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_TxSynth_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_TxSynth_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO0_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_NSS_AR_Pin GPO_TPX_Pin GPO_TP16_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_AR_Pin|GPO_TPX_Pin|GPO_TP16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C3SDA_EEPROM_Pin */
  GPIO_InitStruct.Pin = I2C3SDA_EEPROM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(I2C3SDA_EEPROM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DMDTMP_Pin P28VPA_Pin */
  GPIO_InitStruct.Pin = DMDTMP_Pin|P28VPA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TXRPWR_Pin CRTNSENSE_Pin */
  GPIO_InitStruct.Pin = TXRPWR_Pin|CRTNSENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void PRINTF(const char *format, ...){
	va_list ap;
	char bufferLocal[255]={0}; // print up to 254 characters per PRINTF

	va_start(ap, format);
	vsnprintf((char *)bufferLocal, 255, format, ap);
	va_end(ap);

	HAL_UART_Transmit(&huart1, (uint8_t *)bufferLocal, strlen(bufferLocal), 100);
}

void SetRFBoardToSafeState(void){
	char DeviceData[4];

	// Initialize and Enable RFuC DACs
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, mVTo12bitDigitalOutput(2000)); // initialize RVCA0 to 2V
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, mVTo12bitDigitalOutput(2000)); // initialize TVCA to 26mV
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // RVCA0
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // TVCA

	// Enable the External Rx DAC's internal 2.5V reference voltage
	DeviceData[0] = 0x08;
	DeviceData[1] = 0x00;
	DeviceData[2] = 0x00;
	DeviceData[3] = 0x01;
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_RESET); // SPI2 CS -> RxDAC
	HAL_SPI_Transmit(&hspi2, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_SET); // SPI2 CS -> RxDAC
	HAL_Delay(1); // Delay in milliseconds

	// Initialize External RxDAC Channels
	Spi2_UpdateRxDac(DAC_CHANNEL_A, mVTo12bitDigitalOutput_Halfx(5000)); // Init Channel A (TP18) to 5000mV
	//Spi2_UpdateRxDac(DAC_CHANNEL_B, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel B (TP22) to 0mV
	Spi2_UpdateRxDac(DAC_CHANNEL_C, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel C (TP19) to 0mV
	//Spi2_UpdateRxDac(DAC_CHANNEL_D, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel D (TP21) to 0mV
	Spi2_UpdateRxDac(DAC_CHANNEL_E, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel E (TP20) to 0mV
	//Spi2_UpdateRxDac(DAC_CHANNEL_F, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel F (RXTUNE) to 0mV
	Spi2_UpdateRxDac(DAC_CHANNEL_G, mVTo12bitDigitalOutput_Halfx(2000)); // Init Channel G (RVCA1) to 2000mV
	//Spi2_UpdateRxDac(DAC_CHANNEL_H, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel H (VTUNE) to 0mV

	// Enable the External Tx DAC's internal 2.5V reference voltage
	DeviceData[0] = 0x08;
	DeviceData[1] = 0x00;
	DeviceData[2] = 0x00;
	DeviceData[3] = 0x01;
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_RESET); // SPI3 CS -> TxDAC
	HAL_SPI_Transmit(&hspi3, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_SET); // SPI3 CS -> TxDAC
	HAL_Delay(1); // Delay in milliseconds

	// --- Initialize External TxDAC Channels ---
	Spi3_UpdateTxDac(DAC_CHANNEL_A, mVTo12bitDigitalOutput_Halfx(5000)); // Bit 12 (PBA6; RxDAC-A) to 5000mV
	//Spi3_UpdateTxDac(DAC_CHANNEL_B, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel B (MLIN) to 0mV
	Spi3_UpdateTxDac(DAC_CHANNEL_C, mVTo12bitDigitalOutput_Halfx(5000)); // Bit 11 (PBA5; RxDAC-C) to 5000mV
	//Spi3_UpdateTxDac(DAC_CHANNEL_D, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel D (PBA0) to 0mV
	Spi3_UpdateTxDac(DAC_CHANNEL_E, mVTo12bitDigitalOutput_Halfx(5000)); // Bit 10 (PBA4; RxDAC-E) to 5000mV
	//Spi3_UpdateTxDac(DAC_CHANNEL_F, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel F (PBA1) to 0mV
	//Spi3_UpdateTxDac(DAC_CHANNEL_G, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel G (PBA3) to 0mV
	//Spi3_UpdateTxDac(DAC_CHANNEL_H, mVTo12bitDigitalOutput_Halfx(0)); // Init Channel H (PBA2) to 0mV
}

// Convert millivolts to 12 bit digital DAC register value and return the result.
uint32_t mVTo12bitDigitalOutput(uint32_t milliVolt){
	uint32_t TwelveBitDigitalOutput;

  /*			    _				 _
   * 			   |   D			  |
   * 		Vout = | ----- x G x Vref | + Vos = (0.0006103515625 * D)
   * 			   |_ 2^N			 _|
   *
   *		Where:
   *			D (Digital Input) = 12bit value
   *			N (bits of resolution) = 12
   *			G (gain) = 1
   *			Vref (Reference Voltage) = 2.5V
   *			Vos (Offset Voltage) = 0V
   * */

	TwelveBitDigitalOutput = (uint32_t)(((float)milliVolt / 0.6103515625)+0.5);

	if(TwelveBitDigitalOutput < 0){
		TwelveBitDigitalOutput = 0; // Vout Min = 0V
	}
	else if(TwelveBitDigitalOutput > 4095){
		TwelveBitDigitalOutput = 4095; // Vout Max = 2.5V
	}

	return TwelveBitDigitalOutput;
}

// Convert millivolts to 12 bit digital DAC register value and return the result.
uint32_t mVTo12bitDigitalOutput_Halfx(uint32_t milliVolt){
	uint32_t TwelveBitDigitalOutput;

	TwelveBitDigitalOutput = (uint32_t)(((float)milliVolt / 1.2207)+0.5);

	if(TwelveBitDigitalOutput < 0){
		TwelveBitDigitalOutput = 0; // Vout Min = 0V
	}
	else if(TwelveBitDigitalOutput > 4095){
		TwelveBitDigitalOutput = 4095; // Vout Max = 2.5V
	}

	return TwelveBitDigitalOutput;
}

///----------------------------------------------------------------------------
/// Method Name: Spi2_UpdateRxDac
/// Description: Update External Rx DAC with provided 12 bit digital DAC
///				 register value.
///					@arg DAC_CHANNEL_A: DAC Channel A selected -> TP18
///					@arg DAC_CHANNEL_B: DAC Channel B selected -> TP22
///					@arg DAC_CHANNEL_C: DAC Channel C selected -> TP19
///					@arg DAC_CHANNEL_D: DAC Channel D selected -> TP21
///					@arg DAC_CHANNEL_E: DAC Channel E selected -> TP20
///					@arg DAC_CHANNEL_F: DAC Channel F selected -> RXTUNE
///					@arg DAC_CHANNEL_G: DAC Channel G selected -> RVCA1
///					@arg DAC_CHANNEL_H: DAC Channel H selected -> VTUNE
///----------------------------------------------------------------------------
void Spi2_UpdateRxDac(ExternDacChannel_e Channel, uint16_t DacValue){
	char DeviceData[4];

	if(DacValue < 0){
		DacValue = 0; // Vout Min = 0V
	}
	else if(DacValue > 4095){
		DacValue = 4095; // Vout Max = 5.0V
	}

	DeviceData[0] = 0x03; // Control Bits = Write & Update DAC Ch
	DeviceData[1] = (Channel << 4) | ((DacValue & 0b111100000000) >> 8); // Mix: MSB = DAC ADDRESS; LSB = D11-D8
	DeviceData[2] = (DacValue & 0b000011111111); // LSB D7-D0
	DeviceData[3] = 0x00; // Don't Cares
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_RESET); // SPI2 CS -> RxDAC
	HAL_SPI_Transmit(&hspi2, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_SET); // SPI2 CS -> RxDAC
	HAL_Delay(1); // Delay in milliseconds

	// Enable the External Rx DAC's internal 2.5V reference voltage
	DeviceData[0] = 0x08;
	DeviceData[1] = 0x00;
	DeviceData[2] = 0x00;
	DeviceData[3] = 0x01;
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_RESET); // SPI2 CS -> RxDAC
	HAL_SPI_Transmit(&hspi2, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI2_NSS_RxDAC_GPIO_Port, SPI2_NSS_RxDAC_Pin, GPIO_PIN_SET); // SPI2 CS -> RxDAC
	HAL_Delay(1); // Delay in milliseconds
}

///----------------------------------------------------------------------------
/// Method Name: Spi3_UpdateTxDac
/// Description: Update External Tx DAC with provided 12 bit digital DAC
///							 register value.
///					@arg DAC_CHANNEL_A: DAC Channel A selected -> PAB6
///					@arg DAC_CHANNEL_B: DAC Channel B selected -> MLIN
///					@arg DAC_CHANNEL_C: DAC Channel C selected -> PAB5
///					@arg DAC_CHANNEL_D: DAC Channel D selected -> PAB0
///					@arg DAC_CHANNEL_E: DAC Channel E selected -> PAB4
///					@arg DAC_CHANNEL_F: DAC Channel F selected -> PAB1
///					@arg DAC_CHANNEL_G: DAC Channel G selected -> PAB3
///					@arg DAC_CHANNEL_H: DAC Channel H selected -> PAB2
///----------------------------------------------------------------------------
void Spi3_UpdateTxDac(ExternDacChannel_e Channel, uint16_t DacValue){
	char DeviceData[4];

	if(DacValue < 0){
		DacValue = 0; // Vout Min = 0V
	}
	else if(DacValue > 4095){
		DacValue = 4095; // Vout Max = 2.5V
	}

	DeviceData[0] = 0x03; // Control Bits = Write & Update DAC Ch
	DeviceData[1] = (Channel << 4) | ((DacValue & 0b111100000000) >> 8); // Mix: MSB = DAC ADDRESS; LSB = D11-D8
	DeviceData[2] = (DacValue & 0b000011111111); // LSB D7-D0
	DeviceData[3] = 0x00; // Don't Cares
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_RESET); // SPI3 CS -> TxDAC
	HAL_SPI_Transmit(&hspi3, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_SET); // SPI3 CS -> TxDAC
	HAL_Delay(1); // Delay in milliseconds

	// Enable the External Tx DAC's internal 2.5V reference voltage
	DeviceData[0] = 0x08;
	DeviceData[1] = 0x00;
	DeviceData[2] = 0x00;
	DeviceData[3] = 0x01;
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_RESET); // SPI3 CS -> TxDAC
	HAL_SPI_Transmit(&hspi3, (uint8_t *)DeviceData, 4, 100);
	HAL_GPIO_WritePin(SPI3_NSS_TxDAC_GPIO_Port, SPI3_NSS_TxDAC_Pin, GPIO_PIN_SET); // SPI3 CS -> TxDAC
	HAL_Delay(1); // Delay in milliseconds
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
