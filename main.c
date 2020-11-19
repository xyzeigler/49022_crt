/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUF_LEN (150)
#define SPI_BUF_LEN (2060)	//target TX buffer size
#define SPI_PACKET_LEN (20)
#define SPI_PACKETS_PER_XFER (SPI_BUF_LEN/SPI_PACKET_LEN)
#define ADC_BUF_LEN 1024	//ADC-DMA target buffer size

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/*ADC VARIABLES*/
uint16_t adc_buf[ADC_BUF_LEN];

/*SPI BT VARIABLES*/
volatile static uint16_t g_command = 0x0000;//stores most recent rcv'd command
volatile static uint8_t g_spi_send_buf[SPI_BUF_LEN]  = {0};  //data to be sent
volatile static uint8_t g_spi_rec_buf[SPI_PACKET_LEN]= {0};  //buffer for received data
volatile static int g_spi_send_idx   = 0;	//current index in TX buffer
volatile static int g_spi_packet_num = 0;	//counts number of packets sent, out of 103

/*MOTOR CONTROLLER VARIABLES*/
/*	(motor to mcu pinouts)
*	ain1 to pin PC7
*	ain2 to pin PC8
*	bin1 to pin PC9
*	bin2 to pin PA8
*	fault_n pin is: went PC6 (not on pcb; need to add connection)
*	sleep_n pin is: pin PA9
*/
int ALast=0;
int cnt=0;			//position variable (0 to 200), each increment is a rotation of 1.8 degrees
int scnt=0;  		//step count (used to update motor inputs)
int istep=0;    	//initial step, only variable which does not reset
int motorindex=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
/* USER CODE BEGIN PFP */


static uint16_t Endian_Swap_16(uint8_t *);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);

static void motorcontroller(int *ALast, int *cnt, int *scnt, int *istep, int *index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Endian_Swap_16 for SPI
 * @brief Swap endian-ness of 2 bytes in an array
 * @param Array pointer to 2 contiguous bytes of data
 * @retval 16-bit integer containing byte order
 */
uint16_t Endian_Swap_16(uint8_t bytes[2]){
	union{
		uint8_t bytes[2];
		uint16_t swapped;
	}   swapper = { 0 };

	swapper.bytes[0] = bytes[1];
	swapper.bytes[1] = bytes[0];

	return swapper.swapped;
}

/* HAL_SPI_TxRxCpltCallback for SPI
 * @brief
 * @param
 * @retval
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI1){
		// 1. Update TX buffer array index vars so it knows
		// where to start sending data from during next Xmission
		g_spi_send_idx = (g_spi_send_idx + SPI_PACKET_LEN) % SPI_BUF_LEN;
		g_spi_packet_num = (g_spi_packet_num+1) % SPI_PACKETS_PER_XFER;
		// 2. Checks received command vs prev commands; changes global command var
		if(g_command!=Endian_Swap_16(g_spi_rec_buf)){
			g_command = Endian_Swap_16(g_spi_rec_buf);
		}
		// 3. Make sure the buffer is only sent in increments of 20 bytes at a time
		// (omitted UART check theory, can readd given serial output)
		// 4. Stops current interrupt routine, check for success.
		int ret = HAL_SPI_DMAStop(&hspi1);
		  //check if ret!=HAL_OK ??
		ret = HAL_SPI_TransmitReceive_DMA(&hspi1, &g_spi_send_buf[g_spi_send_idx], g_spi_rec_buf, SPI_PACKET_LEN);
		if (ret != HAL_OK){
			ret=HAL_SPI_Abort(&hspi1);
			return;
		}
	}
}


/* motorcontroller for Encoder, Motor Control
 * @brief ...
 * @param ...
 * @retval ...
 */
static void motorcontroller(int *ALast, int *cnt, int *scnt, int *istep, int *index)
{

	*scnt=1+*istep+4;

	/* Count Increment Code */
	int A=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	int B=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	int X=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

	if (*ALast == A){			//prev A == A, checking for 0->0 or 1->1
		HAL_Delay(.01);
	}
	else if (A && B && *ALast){	//position count increments when A has been high and B goes high
		*cnt=*cnt+1;
	}
	else if (X){				//index indicates when a full rotation (cnt>=200) occurs
		*index=1;
	}

	*ALast = A;		//set ALast to current A for next iteration to check


	/* While Loop Dedicated (Based on Psuedo-Code) */
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)||!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))//sleep_n or fault_n is low
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
	}

	//while cnt<200 or index<1 && fault_n is high
	else if((*cnt<200||*index<1)&&(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)))
	{
		if((*scnt)%4==1){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);		//Ain1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);	//Ain2
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);	//Bin1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);	//Bin2
		}
		else if((*scnt)%4==2){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);	//Ain1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);	//Ain2
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET);		//Bin1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);	//Bin2
		}
		else if((*scnt)%4==3){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);	//Ain1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET);		//Ain2
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);	//Bin1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);	//Bin2
		}
		else if((*scnt)%4==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);	//Ain1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);	//Ain2
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);	//Bin1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);		//Bin2
		}

	}

	else if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))  //end process when sleep_n < 0
	{
		*istep = ((*scnt)-1)%4;
	}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USB_OTG_HS_USB_Init();
  /* USER CODE BEGIN 2 */

  //setup for SPI interrupt with DMA
  int ret = HAL_SPI_TransmitReceive_DMA(&hspi1,&g_spi_send_buf[g_spi_send_idx], g_spi_rec_buf,SPI_PACKET_LEN);
  /*if (ret != HAL_OK){
	  //do WHATEVER ERROR HANDLING WE NEED HERE
	  //shut down MCU, reset, try again etc.
  }*/

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);  //starts ADC attached to DMA

  //simulate for PLL (need to flesh out) but connect LED to see if board is programmed through SWD
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	motorcontroller(&ALast, &cnt, &scnt, &istep, &motorindex);
	HAL_Delay(0.5);  //delay for 0.5ms so that motor can update properly

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC15 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 PF4 PF5
                           PF6 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//upon adc_buf full 1024 ADC samples (2048 bytes)
	//memcpy adc_buf to tx_buf[0:2048]
	memcpy(g_spi_send_buf,adc_buf,(uint8_t)2048);
	//memcpy(&g_spi_send_buf,&adc_buf,(uint8_t)2048);
	//set tx_buf[2049] to motor pos uint8_t
	g_spi_send_buf[2049]=cnt;
	//set tx_buf[2050] to battery low (GPIO pin read)
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)){  //pin logic high (indicates minimum VDD of 2.31 V)
		g_spi_send_buf[2050]=1;
	}
	else{
		g_spi_send_buf[2050]=0;					//pin logic low (indicates voltage 0.99V or less from VDD 3.3V)
	}

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
