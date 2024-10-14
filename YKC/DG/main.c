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
#include "uarti2cspi_driver.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "type.h"
#include "stm32f4xx_hal_tim.h"
#include "ring_buffer.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/////////////////////플래그//////////////////////////
static cx_bool_t _flag_transmit  		= CX_FALSE;


static cx_bool_t _flag_COM1_rx_done		= CX_FALSE;
static cx_bool_t _flag_COM2_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM3_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM4_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM5_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM6_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM7_rx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM8_rx_done	 	= CX_FALSE;


static cx_bool_t _flag_COM1_tx_done		= CX_FALSE;
static cx_bool_t _flag_COM2_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM3_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM4_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM5_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM6_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM7_tx_done	 	= CX_FALSE;
static cx_bool_t _flag_COM8_tx_done	 	= CX_FALSE;
/////////////////////플래그//////////////////////////




//////////////////링버퍼, 원형버퍼///////////////////////////

uart_hal_rx_type uart_hal_rx;



static uart_hal_rx_type           g_uart1_rxringbuf;
static cx_uint8_t         		  g_uart1_rxbuf[256];
static const cx_uint_t    		  g_uart1_rxbufsize = 256;

static uart_hal_rx_type           g_uart2_rxringbuf;
static cx_uint8_t        		  g_uart2_rxbuf[256];
static const cx_uint_t    		  g_uart2_rxbufsize = 256;

static uart_hal_rx_type           g_uart3_rxringbuf;
static cx_uint8_t         		  g_uart3_rxbuf[256];
static const cx_uint_t   		  g_uart3_rxbufsize = 256;

static uart_hal_rx_type           g_uart4_rxringbuf;
static cx_uint8_t         		  g_uart4_rxbuf[256];
static const cx_uint_t    		  g_uart4_rxbufsize = 256;

static uart_hal_rx_type           g_uart5_rxringbuf;
static cx_uint8_t	         	  g_uart5_rxbuf[256];
static const cx_uint_t    		  g_uart5_rxbufsize = 256;

static uart_hal_rx_type           g_uart6_rxringbuf;
static cx_uint8_t	         	  g_uart6_rxbuf[256];
static const cx_uint_t    		  g_uart6_rxbufsize = 256;

static uart_hal_rx_type           g_uart7_rxringbuf;
static cx_uint8_t	         	  g_uart7_rxbuf[256];
static const cx_uint_t    		  g_uart7_rxbufsize = 256;

static uart_hal_rx_type           g_uart8_rxringbuf;
static cx_uint8_t	         	  g_uart8_rxbuf[256];
static const cx_uint_t    		  g_uart8_rxbufsize = 256;


//////////////////링버퍼, 원형버퍼///////////////////////////




///////////////////////UART 수신////////////////////////
static cx_uint8_t COM1_data =0;
static cx_uint8_t COM2_data =0;
static cx_uint8_t COM3_data =0;
static cx_uint8_t COM4_data =0;
static cx_uint8_t COM5_data =0;
static cx_uint8_t COM6_data =0;
static cx_uint8_t COM7_data =0;
static cx_uint8_t COM8_data =0;

static cx_uint8_t _COM1_rx_data [150];
static cx_uint8_t _COM2_rx_data	[150];
static cx_uint8_t _COM3_rx_data	[150];
static cx_uint8_t _COM4_rx_data	[150];
static cx_uint8_t _COM5_rx_data	[150];
static cx_uint8_t _COM6_rx_data	[150];
static cx_uint8_t _COM7_rx_data	[150];
static cx_uint8_t _COM8_rx_data	[150];





static cx_uint8_t 		_DG_tx_buf[8][30];


static cx_uint8_t _data_[30] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0xA0, 0x03, 0x0D, 0x0A};


///////////////////////UART 수신////////////////////////


////////////////////////타이머////////////////////
static cx_uint_t _timer_count_0020_msec = 0;
static cx_uint_t _COM1_rx_timeout_count = 0;
static cx_uint_t _COM2_rx_timeout_count = 0;
static cx_uint_t _COM3_rx_timeout_count = 0;
static cx_uint_t _COM4_rx_timeout_count = 0;
static cx_uint_t _COM5_rx_timeout_count = 0;
static cx_uint_t _COM6_rx_timeout_count = 0;
static cx_uint_t _COM7_rx_timeout_count = 0;
static cx_uint_t _COM8_rx_timeout_count = 0;


static cx_uint_t _COM1_tx_timeout_count = 0;
static cx_uint_t _COM2_tx_timeout_count = 0;
static cx_uint_t _COM3_tx_timeout_count = 0;
static cx_uint_t _COM4_tx_timeout_count = 0;
static cx_uint_t _COM5_tx_timeout_count = 0;
static cx_uint_t _COM6_tx_timeout_count = 0;
static cx_uint_t _COM7_tx_timeout_count = 0;
static cx_uint_t _COM8_tx_timeout_count = 0;

////////////////////////타이머////////////////////


//for SPI to UART Module
const uint8_t _UARTI2CSPI_COMMUNICATION_I2C = 0x01;
const uint8_t _UARTI2CSPI_COMMUNICATION_SPI = 0x02;

float UARTI2CSPI_OSCILATOR_FREQ             = 1843200.0;
float UARTI2CSPI_PRESCALER_DEF              = 1.0;
const uint8_t UARTI2CSPI_ADDR               = 0x48;
const uint8_t UARTI2CSPI_RHR                = 0x00<<3;
const uint8_t UARTI2CSPI_THR                = 0x00<<3;
const uint8_t UARTI2CSPI_IER                = 0x01<<3;
const uint8_t UARTI2CSPI_FCR                = 0x02<<3;
const uint8_t UARTI2CSPI_IIR                = 0x02<<3;
const uint8_t UARTI2CSPI_LCR                = 0x03<<3;
const uint8_t UARTI2CSPI_MCR                = 0x04<<3;
const uint8_t UARTI2CSPI_LSR                = 0x05<<3;
const uint8_t UARTI2CSPI_MSR                = 0x06<<3;
const uint8_t UARTI2CSPI_SPR                = 0x07<<3;
const uint8_t UARTI2CSPI_TCR                = 0x06<<3;
const uint8_t UARTI2CSPI_TLR                = 0x07<<3;
const uint8_t UARTI2CSPI_TXLVL              = 0x08<<3;
const uint8_t UARTI2CSPI_RXLVL              = 0x09<<3;
const uint8_t UARTI2CSPI_EFCR               = 0x0F<<3;

/* Special register set LCR[7] = 1 */
const uint8_t UARTI2CSPI_DLL   = 0x00<<3;
const uint8_t UARTI2CSPI_DLH   = 0x01<<3;
/* Enhanced register set LCR   = 0xBF.*/
const uint8_t UARTI2CSPI_EFR   = 0x02<<3;
const uint8_t UARTI2CSPI_XON1  = 0x04<<3;
const uint8_t UARTI2CSPI_XON2  = 0x05<<3;
const uint8_t UARTI2CSPI_XOFF1 = 0x06<<3;
const uint8_t UARTI2CSPI_XOFF2 = 0x07<<3;

/* Bitfields Init advanced */
const uint8_t UARTI2CSPI_UART_5_BIT_DATA   = 0x00;
const uint8_t UARTI2CSPI_UART_6_BIT_DATA   = 0x01;
const uint8_t UARTI2CSPI_UART_7_BIT_DATA   = 0x02;
const uint8_t UARTI2CSPI_UART_8_BIT_DATA   = 0x03;
const uint8_t UARTI2CSPI_UART_NOPARITY     = 0x00;
const uint8_t UARTI2CSPI_UART_EVENPARITY   = 0x18;
const uint8_t UARTI2CSPI_UART_ODDPARITY    = 0x08;
const uint8_t UARTI2CSPI_UART_PARITY_ONE   = 0x38;
const uint8_t UARTI2CSPI_UART_PARITY_ZERO  = 0x28;
const uint8_t UARTI2CSPI_UART_ONE_STOPBIT  = 0x00;
const uint8_t UARTI2CSPI_UART_TWO_STOPBITS = 0x04;

/* Interrupt bits */
const uint8_t UARTI2CSPI_CTS_INT_EN                   = 0x80;
const uint8_t UARTI2CSPI_RTS_INT_EN                   = 0x40;
const uint8_t UARTI2CSPI_XOFF_INT_EN                  = 0x20;
const uint8_t UARTI2CSPI_SLEEP_INT_EN                 = 0x10;
const uint8_t UARTI2CSPI_MODEM_STATUS_INT_EN          = 0x08;
const uint8_t UARTI2CSPI_RECEIVE_LINE_STATUS_INT_EN   = 0x04;
const uint8_t UARTI2CSPI_THR_EMPTY_INT_EN             = 0x02;
const uint8_t UARTI2CSPI_RXD_INT_EN                   = 0x01;

const uint8_t _LINE_PRINT = 0x03;
const uint8_t _TEXT_PRINT = 0x01;
const uint8_t _CHAR_PRINT = 0x04;

cx_uint8_t str[]= "HFFFF";





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t _calcBaudRate(uint16_t baud_rate);
cx_uint8_t Transmit_checksum(unsigned char *data, size_t length);
void ascii_to_bcd(cx_uint8_t *asciidata, cx_uint8_t* bcd_values, cx_uint_t data_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t _calcBaudRate(uint16_t baud_rate)
{
    uint16_t calc;
    float tmp;
    tmp =  UARTI2CSPI_OSCILATOR_FREQ / UARTI2CSPI_PRESCALER_DEF;
    calc = (uint16_t)( tmp / ( (float)baud_rate * 16.0 ) );

    return calc;
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(SPI_RST_GPIO_Port, SPI_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin,   GPIO_PIN_SET);

  uarti2cspi_initAdvanced(19200,UARTI2CSPI_UART_8_BIT_DATA,UARTI2CSPI_UART_NOPARITY,UARTI2CSPI_UART_ONE_STOPBIT);

  HAL_TIM_Base_Start_IT(&htim3);

  initial_GPIO_LED();


  ringbuf_init (&g_uart1_rxringbuf, g_uart1_rxbuf, g_uart1_rxbufsize);
  ringbuf_init (&g_uart2_rxringbuf, g_uart2_rxbuf, g_uart2_rxbufsize);
  ringbuf_init (&g_uart3_rxringbuf, g_uart3_rxbuf, g_uart3_rxbufsize);
  ringbuf_init (&g_uart4_rxringbuf, g_uart4_rxbuf, g_uart4_rxbufsize);
  ringbuf_init (&g_uart5_rxringbuf, g_uart5_rxbuf, g_uart5_rxbufsize);
  ringbuf_init (&g_uart6_rxringbuf, g_uart6_rxbuf, g_uart6_rxbufsize);
  ringbuf_init (&g_uart7_rxringbuf, g_uart7_rxbuf, g_uart7_rxbufsize);
  ringbuf_init (&g_uart8_rxringbuf, g_uart8_rxbuf, g_uart8_rxbufsize);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);

  HAL_UART_Receive_IT(&huart1, &COM1_data, 1);
  HAL_UART_Receive_IT(&huart2, &COM2_data, 1);
  HAL_UART_Receive_IT(&huart3, &COM3_data, 1);
  HAL_UART_Receive_IT(&huart4, &COM4_data, 1);
  HAL_UART_Receive_IT(&huart5, &COM5_data, 1);
  HAL_UART_Receive_IT(&huart6, &COM6_data, 1);
  HAL_UART_Receive_IT(&huart7, &COM7_data, 1);
  HAL_UART_Receive_IT(&huart8, &COM8_data, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  uartbufcount1();
	  uartbufcount2();
	  uartbufcount3();
	  uartbufcount4();
	  uartbufcount5();
	  uartbufcount6();
	  uartbufcount7();
	  uartbufcount8();


	  if(_flag_transmit == CX_TRUE)
	  {
		  transmit();

		  _flag_transmit = CX_FALSE;

	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 18;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 9600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Watchdog_OUTPUT_GPIO_Port, Watchdog_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SPI_UART_RX_LED_Pin|SPI_UART_TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|SPI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, RX_1_RX_LED_Pin|RX_1_TX_LED_Pin|RX_2_RX_LED_Pin|RX_2_TX_LED_Pin
                          |RX_3_RX_LED_Pin|RX_3_TX_LED_Pin|RX_4_RX_LED_Pin|RX_4_TX_LED_Pin
                          |RX_5_RX_LED_Pin|RX_5_TX_LED_Pin|RX_6_RX_LED_Pin|RX_6_TX_LED_Pin
                          |RX_7_RX_LED_Pin|RX_7_TX_LED_Pin|RX_8_RX_LED_Pin|RX_8_TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCU_Status_1_Pin|MCU_Status_2_Pin|MCU_Status_3_Pin|MCU_Status_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Watchdog_INPUT_Pin */
  GPIO_InitStruct.Pin = Watchdog_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Watchdog_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Watchdog_OUTPUT_Pin */
  GPIO_InitStruct.Pin = Watchdog_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Watchdog_OUTPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_UART_RX_LED_Pin SPI_UART_TX_LED_Pin */
  GPIO_InitStruct.Pin = SPI_UART_RX_LED_Pin|SPI_UART_TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin SPI_RST_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|SPI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_1_RX_LED_Pin RX_1_TX_LED_Pin RX_2_RX_LED_Pin RX_2_TX_LED_Pin
                           RX_3_RX_LED_Pin RX_3_TX_LED_Pin RX_4_RX_LED_Pin RX_4_TX_LED_Pin
                           RX_5_RX_LED_Pin RX_5_TX_LED_Pin RX_6_RX_LED_Pin RX_6_TX_LED_Pin
                           RX_7_RX_LED_Pin RX_7_TX_LED_Pin RX_8_RX_LED_Pin RX_8_TX_LED_Pin */
  GPIO_InitStruct.Pin = RX_1_RX_LED_Pin|RX_1_TX_LED_Pin|RX_2_RX_LED_Pin|RX_2_TX_LED_Pin
                          |RX_3_RX_LED_Pin|RX_3_TX_LED_Pin|RX_4_RX_LED_Pin|RX_4_TX_LED_Pin
                          |RX_5_RX_LED_Pin|RX_5_TX_LED_Pin|RX_6_RX_LED_Pin|RX_6_TX_LED_Pin
                          |RX_7_RX_LED_Pin|RX_7_TX_LED_Pin|RX_8_RX_LED_Pin|RX_8_TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_Status_1_Pin MCU_Status_2_Pin MCU_Status_3_Pin MCU_Status_4_Pin */
  GPIO_InitStruct.Pin = MCU_Status_1_Pin|MCU_Status_2_Pin|MCU_Status_3_Pin|MCU_Status_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//////////LED 초기화//////////////////////////////////////////////////////////


void initial_GPIO_LED(void)
{
	HAL_GPIO_WritePin(RX_1_RX_LED_GPIO_Port, RX_1_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_2_RX_LED_GPIO_Port, RX_2_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_3_RX_LED_GPIO_Port, RX_3_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_4_RX_LED_GPIO_Port, RX_4_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_5_RX_LED_GPIO_Port, RX_5_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_6_RX_LED_GPIO_Port, RX_6_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_7_RX_LED_GPIO_Port, RX_7_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_8_RX_LED_GPIO_Port, RX_8_RX_LED_Pin, SET);

	HAL_GPIO_WritePin(RX_1_TX_LED_GPIO_Port, RX_1_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_2_TX_LED_GPIO_Port, RX_2_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_3_TX_LED_GPIO_Port, RX_3_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_4_TX_LED_GPIO_Port, RX_4_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_5_TX_LED_GPIO_Port, RX_5_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_6_TX_LED_GPIO_Port, RX_6_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_7_TX_LED_GPIO_Port, RX_7_TX_LED_Pin, SET);
	HAL_GPIO_WritePin(RX_8_TX_LED_GPIO_Port, RX_8_TX_LED_Pin, SET);


	HAL_GPIO_WritePin(SPI_UART_RX_LED_GPIO_Port, SPI_UART_RX_LED_Pin, SET);
	HAL_GPIO_WritePin(SPI_UART_TX_LED_GPIO_Port, SPI_UART_TX_LED_Pin, SET);

	HAL_Delay(30);


}





////////////////////////////타이머////////////////////////////////////



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

		if(htim->Instance == TIM3)
		{
        ////////////SPI 송신 타이머/////////////
			_timer_count_0020_msec++;
        ///////////////////////////////////


///////////////////UART 수신 타이머 카운트/////////////////////
			_COM1_rx_timeout_count++;
			_COM2_rx_timeout_count++;
			_COM3_rx_timeout_count++;
			_COM4_rx_timeout_count++;
			_COM5_rx_timeout_count++;
			_COM6_rx_timeout_count++;
			_COM7_rx_timeout_count++;
			_COM8_rx_timeout_count++;
//////////////////UART 수신 타이머 카운트/////////////////////




//////////////////////파싱 타이머 카운트/////////////////////
			_COM1_tx_timeout_count++;
			_COM2_tx_timeout_count++;
			_COM3_tx_timeout_count++;
			_COM4_tx_timeout_count++;
			_COM5_tx_timeout_count++;
			_COM6_tx_timeout_count++;
			_COM7_tx_timeout_count++;
			_COM8_tx_timeout_count++;
/////////////////파싱 타이머 카운트///////////////////////






			if(_timer_count_0020_msec>1000)
			{
				_flag_transmit = CX_TRUE;
				_timer_count_0020_msec=0;
			}



//////////////각 포트 타이머////////////////////////




///////////////////////////포트1////////////////////////////////////////////////////////



			//// RX_1_RX 카운트////
			if(_COM1_rx_timeout_count>2000) //2초간 대기 후 초기화
			{
				if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_1_RX_LED_GPIO_Port, RX_1_RX_LED_Pin, SET);

					_COM1_rx_timeout_count = 0;

				}

			}





///////////////////////////포트2////////////////////////////////////////////////////////

			if(_COM2_rx_timeout_count>2000)
			{
				if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_2_RX_LED_GPIO_Port, RX_2_RX_LED_Pin, SET);

					_COM2_rx_timeout_count = 0;

				}

			}



///////////////////////////포트3////////////////////////////////////////////////////////
			if(_COM3_rx_timeout_count>2000)
			{
				if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_3_RX_LED_GPIO_Port, RX_3_RX_LED_Pin, SET);

					_COM3_rx_timeout_count = 0;

				}


			}


///////////////////////////포트4////////////////////////////////////////////////////////
			if(_COM4_rx_timeout_count>2000)
			{
				if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_4_RX_LED_GPIO_Port, RX_4_RX_LED_Pin, SET);

					_COM4_rx_timeout_count = 0;

				}


			}



///////////////////////////포트5////////////////////////////////////////////////////////
			if(_COM5_rx_timeout_count>2000)
			{
				if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_5_RX_LED_GPIO_Port, RX_5_RX_LED_Pin, SET);

					_COM5_rx_timeout_count = 0;

				}


			}



///////////////////////////포트6////////////////////////////////////////////////////////
			if(_COM6_rx_timeout_count>2000)
			{
				if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_6_RX_LED_GPIO_Port, RX_6_RX_LED_Pin, SET);

					_COM6_rx_timeout_count = 0;

				}


			}

///////////////////////////포트7////////////////////////////////////////////////////////

			if(_COM7_rx_timeout_count>2000)
			{

				if(__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_7_RX_LED_GPIO_Port, RX_7_RX_LED_Pin, SET);

					_COM7_rx_timeout_count = 0;

				}

			}


///////////////////////////포트8////////////////////////////////////////////////////////
			if(_COM8_rx_timeout_count>2000)
			{

				if(__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE) == RESET)
				{

					HAL_GPIO_WritePin(RX_8_RX_LED_GPIO_Port, RX_8_RX_LED_Pin, SET);

					_COM8_rx_timeout_count = 0;

				}


			}


		}


}


////////////////////////수신 인터럽트///////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)
	{

		OnUart1_Recv(COM1_data);
		HAL_GPIO_WritePin(RX_1_RX_LED_GPIO_Port, RX_1_RX_LED_Pin, RESET);

		HAL_UART_Receive_IT(&huart1, &COM1_data, 1);


	}

	if(huart->Instance == USART2)
	{

		OnUart2_Recv(COM2_data);

		HAL_GPIO_WritePin(RX_2_RX_LED_GPIO_Port, RX_2_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart2, &COM2_data, 1);


	}

	if(huart->Instance == USART3)
	{

		OnUart3_Recv(COM3_data);



		HAL_GPIO_WritePin(RX_3_RX_LED_GPIO_Port, RX_3_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart3, &COM3_data, 1);

	}

	if(huart->Instance == UART4)
	{

		OnUart4_Recv(COM4_data);

		HAL_GPIO_WritePin(RX_4_RX_LED_GPIO_Port, RX_4_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart4, &COM4_data, 1);

	}

	if(huart->Instance == UART5)
	{


		OnUart5_Recv(COM5_data);


		HAL_GPIO_WritePin(RX_5_RX_LED_GPIO_Port, RX_5_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart5, &COM5_data, 1);


	}

	if(huart->Instance == USART6)
	{


		OnUart6_Recv(COM6_data);

		HAL_GPIO_WritePin(RX_6_RX_LED_GPIO_Port, RX_6_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart6, &COM6_data, 1);


	}

	if(huart->Instance == UART7)
	{


		OnUart7_Recv(COM7_data);


		HAL_GPIO_WritePin(RX_7_RX_LED_GPIO_Port, RX_7_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart7, &COM7_data, 1);


	}

	if(huart->Instance == UART8)
	{

		OnUart8_Recv(COM8_data);

		HAL_GPIO_WritePin(RX_8_RX_LED_GPIO_Port, RX_8_RX_LED_Pin, RESET);
		HAL_UART_Receive_IT(&huart8, &COM8_data, 1);

	}



}




//////////////////////////////////UART 수신 데이터 링버퍼 처리///////////////////////////////////////////



void OnUart1_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart1_rxringbuf, &ch, 1);
}
void OnUart2_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart2_rxringbuf, &ch, 1);
}
void OnUart3_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart3_rxringbuf, &ch, 1);
}
void OnUart4_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart4_rxringbuf, &ch, 1);
}
void OnUart5_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart5_rxringbuf, &ch, 1);
}
void OnUart6_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart6_rxringbuf, &ch, 1);
}
void OnUart7_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart7_rxringbuf, &ch, 1);
}
void OnUart8_Recv (cx_uint8_t ch)
{
	ringbuf_write (&g_uart8_rxringbuf, &ch, 1);
}




///////////////////////////////////////UART 수신 처리////////////////////////////////////////////////////////



void uartbufcount1(void) //원형 버퍼로부터 데이터를 받아 처리
{

	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart1_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart1_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if (i <= sizeof(buf))
	                {
	                    // 0x02부터 시작해서 50바이트 복사
	                    memcpy(_COM1_rx_data, &buf[i+1], 50);
	                    _flag_COM1_rx_done = CX_TRUE;
	                    uartbufverify1();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }

	                else
	                {
	                    i++;
	                }

	            }

	        }

	   }


}

void uartbufcount2(void)
{

	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart2_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart2_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if (i <= sizeof(buf))
	                {
	                    // 0x02부터 시작해서 50바이트 복사
	                    memcpy(_COM2_rx_data, &buf[i+1], 50);
	                    _flag_COM2_rx_done = CX_TRUE;
	                    uartbufverify2();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }

	                else
	                {
	                    i++;
	                }

	            }

	        }

	   }

}

void uartbufcount3(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart3_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart3_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if ( i <= sizeof(buf))
	                {
	                    // 0x02부터 시작해서 50바이트 복사
	                    memcpy(_COM3_rx_data, &buf[i+1], 50);
	                    _flag_COM3_rx_done = CX_TRUE;
	                    uartbufverify3();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }

	                else
	                {
	                    i++;
	                }

	            }

	        }

	   }



}


void uartbufcount4(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart4_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart4_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if ( i <= sizeof(buf))
	                {
	                    // 0x02부터 시작해서 50바이트 복사
	                    memcpy(_COM4_rx_data, &buf[i+1], 50);
	                    _flag_COM4_rx_done = CX_TRUE;
	                    uartbufverify4();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }
	                else
	                {
	                    i++;
	                }
	            }
	        }
	   }


}

void uartbufcount5(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart5_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart5_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if ( i <= sizeof(buf))
	                {
	                    // 0x02부터 시작해서 50바이트 복사
	                    memcpy(_COM5_rx_data, &buf[i+1], 50);
	                    _flag_COM5_rx_done = CX_TRUE;
	                    uartbufverify5();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }
	                else
	                {
	                    i++;
	                }
	            }
	        }
	   }

}

void uartbufcount6(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart6_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart6_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if ( i <= sizeof(buf))
	                {
	                    // 0x0a부터 시작해서 50바이트 복사
	                    memcpy(_COM6_rx_data, &buf[i+1], 50);
	                    _flag_COM6_rx_done = CX_TRUE;
	                    uartbufverify6();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }
	                else
	                {
	                    i++;
	                }
	            }
	        }
	   }

}


void uartbufcount7(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart7_rxringbuf) > 0)
	 {

	        if (ringbuf_read(&g_uart7_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
	        {
	            buf[i++] = temp_buf[0];

	            // 0x0a를 발견하면 처리
	            if (buf[i] == 0x0A)
	            {
	                if ( i <= sizeof(buf))
	                {
	                    // 0x0a부터 시작해서 46바이트 복사
	                    memcpy(_COM7_rx_data, &buf[i+1], 50);
	                    _flag_COM7_rx_done = CX_TRUE;
	                    uartbufverify7();
	                    memset(buf, 0x00, 255);
	                    i = 0;
	                }
	                else
	                {
	                    i++;
	                }
	            }
	        }
	   }


}


void uartbufcount8(void)
{
	static cx_uint8_t i=0;
	static cx_uint8_t temp_buf[1];
	static cx_uint8_t buf[255];



	 while (ringbuf_get_readable_space(&g_uart8_rxringbuf) > 0)
	 {

		if (ringbuf_read(&g_uart8_rxringbuf, temp_buf, 1) > 0) // 원형 버퍼에서 1바이트 읽어서 temp_buf에 저장
		{
			buf[i++] = temp_buf[0];


			if (buf[i] == 0x0A) // 0x0a를 발견하면 처리
			{
				if ( i <= sizeof(buf))
				{
					// 0x0a+1부터 시작해서 50바이트 복사
					memcpy(_COM8_rx_data, &buf[i+1], 50);
					_flag_COM8_rx_done = CX_TRUE;
					uartbufverify8();
					memset(buf, 0x00, 255);
					i = 0;
				}
				else
				{
					i++;
				}
			}

		 }

	   }


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////UART 패킷 처리/////////////////////////////////////////////////////////

void uartbufverify1(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM1_rx_data);


	if(CX_FALSE == _flag_COM1_rx_done)
	{

		return;
	}



	if(Packet_Verify(_COM1_rx_data, size, max, &offset))
	{
		_flag_COM1_tx_done = CX_TRUE;
		COM_Input_Parse1(_COM1_rx_data, size, 1);


	}
}

void uartbufverify2(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM2_rx_data);


	if(CX_FALSE == _flag_COM2_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM2_rx_data,size , max, &offset))
	{
		_flag_COM2_tx_done = CX_TRUE;
		COM_Input_Parse2(_COM2_rx_data, size, 2);

	}
}

void uartbufverify3(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM3_rx_data);


	if(CX_FALSE == _flag_COM3_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM3_rx_data, size, max, &offset))
	{
		_flag_COM3_tx_done = CX_TRUE;
		COM_Input_Parse3(_COM3_rx_data, size, 3);

	}

}

void uartbufverify4(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM4_rx_data);


	if(CX_FALSE == _flag_COM4_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM4_rx_data, size, max, &offset))
	{
		_flag_COM4_tx_done = CX_TRUE;
		COM_Input_Parse4(_COM4_rx_data, size, 4);
	}
}
void uartbufverify5(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM5_rx_data);


	if(CX_FALSE == _flag_COM5_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM5_rx_data, size, max, &offset))
	{
		_flag_COM5_tx_done = CX_TRUE;
		COM_Input_Parse5(_COM5_rx_data, size, 5);

	}
}
void uartbufverify6(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM6_rx_data);


	if(CX_FALSE == _flag_COM6_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM6_rx_data, size, max, &offset))
	{
		_flag_COM6_tx_done = CX_TRUE;
		COM_Input_Parse6(_COM6_rx_data, size, 6);

	}
}
void uartbufverify7(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM7_rx_data);


	if(CX_FALSE == _flag_COM7_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM7_rx_data, size, max, &offset))
	{
		_flag_COM7_tx_done = CX_TRUE;
		COM_Input_Parse7(_COM7_rx_data, size, 7);

	}
}

void uartbufverify8(void)
{

	const cx_uint32_t   max = 512;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = sizeof(_COM8_rx_data);


	if(CX_FALSE == _flag_COM8_rx_done)
	{
		return;
	}


	if(Packet_Verify(_COM8_rx_data, size, max, &offset))
	{
		_flag_COM8_tx_done = CX_TRUE;
		COM_Input_Parse8(_COM8_rx_data, size, 8);

	}
}
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////UART 수신 데이터 무결성 검사////////////////////////////////
cx_uint8_t fcs_rx(uint8_t* data, cx_uint16_t len)
{
	cx_uint16_t i;
	cx_uint8_t ch;


	ch = 0;
	for (i = 0; i < len; i++)
	{

		ch += data[i];
	}

	ch += 0x20;

	return ch;
}

cx_uint8_t fcs_conv(uint8_t data)
{
	uint8_t value;
	uint8_t ch;

	ch = data&0x0F;
	//value = data;
	if(data>9)
	{
		value = 'A'+(ch%10);
	}
	else
	{
		value = '0'+ch;
	}


	return value;
}



void Packet_Verify (cx_uint8_t* bufptr, cx_uint32_t bufsize, cx_uint32_t max, cx_uint32_t* packetsize)
{
		cx_uint32_t  offset;
		cx_uint32_t  result;

		cx_uint32_t  setnum;

		cx_uint8_t etx;

		cx_uint8_t chk;

		offset = 0;
		result = 0;

		static cx_uint8_t buf[100];

		static cx_uint32_t i;

		for(i = 0; i<bufsize; i++)
		{
			buf[i] = bufptr[i];

			if(buf[i] == 0x0A)
			{

				etx = buf[i];
				setnum = i;
				break;
			}
		}

		if (bufsize>max)
		{
			goto cleanup;
		}

		//etx = bufptr[bufsize-7];

		if (etx!=0x0A)
		{
			offset+=2;
			goto cleanup;
		}

		chk = fcs_rx(&bufptr[setnum-44], 40);

		if (bufptr[setnum-4]!=fcs_conv(chk>>4))
		{
			offset+=2;
			goto cleanup;
		}

		if (bufptr[setnum-3]!=fcs_conv(chk&0x0f))
		{
			offset+=2;
			goto cleanup;
		}


		offset = bufsize;
		result = 1;

		cleanup:
			if (packetsize)
			{
			*packetsize = offset;
			}

		return result;



}

/////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////데이터 BCD 변경 및 SPI 버퍼 저장/////////////////////////////////////
void COM_Input_Parse1(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{


	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;



	if (com_port == 1)  // COM1 포트만 처리
    {

		static cx_uint32_t i;
		static cx_uint32_t offset;

        for(i = 0; i<len; i++)
        {

        	parsebuf[i] = bufptr[i];

            if(parsebuf[i] == 0x0A)
            {
            	offset=i;
                break;
            }
        }

        if(CX_TRUE == _flag_COM1_tx_done)
        {
			HAL_GPIO_WritePin(RX_1_TX_LED_GPIO_Port, RX_1_TX_LED_Pin, RESET);

        }
        else
        {
        	HAL_GPIO_WritePin(RX_1_TX_LED_GPIO_Port, RX_1_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;


        memcpy(_DG_tx_buf[0],&parsebuf[offset-45], 23);

        _flag_COM1_tx_done = CX_FALSE;

        for(i = 0; i< 100; i++) // 초기화
        {
            parsebuf[i] = 0x00;
            val_savebuf[i]=0x00;

        }


    }

}

void COM_Input_Parse2(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;

	if (com_port == 2)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;


		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
        {
        	parsebuf[i] = bufptr[i];

            if(parsebuf[i] == 0x0A)
            {
            	offset=i;
                break;
            }
        }

        if(CX_TRUE == _flag_COM2_tx_done)
        {
			HAL_GPIO_WritePin(RX_2_TX_LED_GPIO_Port, RX_2_TX_LED_Pin, RESET);

        }
        else
        {
        	HAL_GPIO_WritePin(RX_2_TX_LED_GPIO_Port, RX_2_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;


		memcpy(_DG_tx_buf[1], &parsebuf[offset-45], 23);

		_flag_COM2_tx_done = CX_FALSE;

		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

void COM_Input_Parse3(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;

	if (com_port == 3)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;


		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}

        if(CX_TRUE == _flag_COM3_tx_done)
        {
    		HAL_GPIO_WritePin(RX_3_TX_LED_GPIO_Port, RX_3_TX_LED_Pin, RESET);

        }
        else
        {
        	HAL_GPIO_WritePin(RX_3_TX_LED_GPIO_Port, RX_3_TX_LED_Pin, SET);
        }
        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;


		memcpy(_DG_tx_buf[2], &parsebuf[offset-45], 23);

		_flag_COM3_tx_done = CX_FALSE;


		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}

}

void COM_Input_Parse4(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;



	if (com_port == 4)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;



		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}

        if(CX_TRUE == _flag_COM4_tx_done)
        {
        	HAL_GPIO_WritePin(RX_4_TX_LED_GPIO_Port, RX_4_TX_LED_Pin, RESET);
        }
        else
        {
        	HAL_GPIO_WritePin(RX_4_TX_LED_GPIO_Port, RX_4_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;

		memcpy(_DG_tx_buf[3], &parsebuf[offset-45], 23);

		_flag_COM4_tx_done = CX_FALSE;


		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

void COM_Input_Parse5(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;

	if (com_port == 5)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;


		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}

        if(CX_TRUE == _flag_COM5_tx_done)
        {
        	HAL_GPIO_WritePin(RX_5_TX_LED_GPIO_Port, RX_5_TX_LED_Pin, RESET);

        }
        else
        {
        	HAL_GPIO_WritePin(RX_5_TX_LED_GPIO_Port, RX_5_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;

		memcpy(_DG_tx_buf[4], &parsebuf[offset-45], 23);

		_flag_COM5_tx_done = CX_FALSE;

		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

void COM_Input_Parse6(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;


	if (com_port == 6)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;



		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}

        if(CX_TRUE == _flag_COM6_tx_done)
        {
        	HAL_GPIO_WritePin(RX_6_TX_LED_GPIO_Port, RX_6_TX_LED_Pin, RESET);
        }
        else
        {
        	HAL_GPIO_WritePin(RX_6_TX_LED_GPIO_Port, RX_6_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;


		memcpy(_DG_tx_buf[5], &parsebuf[offset-45], 23);

		_flag_COM6_tx_done = CX_FALSE;


		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

void COM_Input_Parse7(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;

	if (com_port == 7)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;


		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}

        if(CX_TRUE == _flag_COM7_tx_done)
        {
        	HAL_GPIO_WritePin(RX_7_TX_LED_GPIO_Port, RX_7_TX_LED_Pin, RESET);
        }
        else
        {
        	HAL_GPIO_WritePin(RX_7_TX_LED_GPIO_Port, RX_7_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;

		memcpy(_DG_tx_buf[6], &parsebuf[offset-45], 23);

		_flag_COM7_tx_done = CX_FALSE;

		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

void COM_Input_Parse8(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[100];
	static cx_uint8_t val_savebuf[50];

	static cx_uint8_t chk;
	chk =0;

	if (com_port == 8)
	{
		static cx_uint8_t i;
		static cx_uint32_t offset;


		for(i = 0; i<len; i++)  // parsebuf에 bufptr의 데이터를 복사
		{
			parsebuf[i] = bufptr[i];

			if(parsebuf[i] == 0x0A)
			{
				offset=i;
				break;
			}
		}


        if(CX_TRUE == _flag_COM8_tx_done)
        {
        	HAL_GPIO_WritePin(RX_8_TX_LED_GPIO_Port, RX_8_TX_LED_Pin, RESET);

        }
        else
        {
        	HAL_GPIO_WritePin(RX_8_TX_LED_GPIO_Port, RX_8_TX_LED_Pin, SET);
        }

        ascii_to_bcd(&parsebuf[offset-44], val_savebuf, 40);
        parsebuf[offset-44] = 0x01;
        memcpy(&parsebuf[offset-43],val_savebuf, 21);


        chk = Transmit_checksum(&parsebuf[offset-45], 22);
        parsebuf[offset-23] = chk;


		memcpy(_DG_tx_buf[7], &parsebuf[offset-45], 23);

		 _flag_COM8_tx_done = CX_FALSE;

		for(i=0; i<100; i++)
		{

			parsebuf[i]=0x00;
			val_savebuf[i]=0x00;

		}

	}
}

///////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////ascii to bcd 변형/////////////////////////////////////////////////

void ascii_to_bcd(cx_uint8_t *asciidata, cx_uint8_t* bcd_values, cx_uint_t data_size)
{

	cx_uint8_t buf[100];

	static cx_uint8_t j;

	for (cx_uint8_t i = 0; i < data_size; i++)
    {
		buf [i] = asciidata[i];

		if(buf[i] == 0x0A)
		{
			break;
		}


    }

	for(j=0; j<data_size; j++)
	{

        if (buf[j] >= '0' && buf[j] <= '9')
        {
        	cx_uint8_t high_nibble = 0;
        	cx_uint8_t low_nibble = 0;

        	high_nibble = (buf[j] - '0') & 0x0F;
        	low_nibble = (buf[j+ 1] - '0') & 0x0F;

        	bcd_values[j / 2] = (high_nibble << 4) | low_nibble;

            j++; // 다음 ASCII 문자로 이동
        }
        else
        {
        	bcd_values[j / 2] = 0; // ASCII가 숫자가 아닌 경우 0으로 설정
        }
	}

}
/////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////// SPI 송신 패킷 무결성 함수 ////////////////////////////////////////////////////////
cx_uint8_t Transmit_checksum(cx_uint8_t *data, cx_uint_t length)
{
	cx_uint8_t result = 0;
	cx_uint8_t idx;



    for (idx = 0; idx < length; ++idx)
    {
        result += (0x0F & data[idx]) + (data[idx] >> 4);
    }

    return result - 99;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////// UART to SPI 전송 ////////////////////////////////////////////////////////
void transmit(void)
{
	static cx_uint8_t chk;

	chk =0;

	chk = Transmit_checksum(_DG_tx_buf[0], 22);

	if(_DG_tx_buf[0][22]==chk    /*임시 설정*/ )
	{
		static cx_uint_t i;
		static cx_uint_t j;
		static cx_uint_t k;

		if(_DG_tx_buf[0][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[0][23] = 0x03;
		_DG_tx_buf[0][24] = 0x0d;
		_DG_tx_buf[0][25] = 0x0a;

		if(CX_FALSE == _flag_COM1_tx_done)
		{
			HAL_GPIO_WritePin(RX_1_TX_LED_GPIO_Port, RX_1_TX_LED_Pin, SET);
		}

		for( k= 0; k< sizeof(_COM1_rx_data); k++)
		{
			_COM1_rx_data[k]= 0x00;

		}




		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}


		for(  i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[0][i]);
			_DG_tx_buf[0][i] = 0x00;

		}


	}




	chk = Transmit_checksum(_DG_tx_buf[1], 22);

	if(_DG_tx_buf[1][22]== chk )
	{
		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;


		if(_DG_tx_buf[1][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[1][23] = 0x03;
		_DG_tx_buf[1][24] = 0x0d;
		_DG_tx_buf[1][25] = 0x0a;

		if(CX_FALSE == _flag_COM2_tx_done)
		{
			HAL_GPIO_WritePin(RX_2_TX_LED_GPIO_Port, RX_2_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM2_rx_data); k++)
		{
			_COM2_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}


		for(i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[1][i]);
			_DG_tx_buf[1][i] = 0x00;
		}


	}


	chk = Transmit_checksum(_DG_tx_buf[2], 22);
	if(_DG_tx_buf[2][22]==chk )
	{

		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[2][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[2][23] = 0x03;
		_DG_tx_buf[2][24] = 0x0d;
		_DG_tx_buf[2][25] = 0x0a;


		if(CX_FALSE == _flag_COM3_tx_done)
		{
			HAL_GPIO_WritePin(RX_3_TX_LED_GPIO_Port, RX_3_TX_LED_Pin, SET);
		}


		for( k= 0; k<sizeof(_COM3_rx_data); k++)
		{
			_COM3_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}

		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[2][i]);
			_DG_tx_buf[2][i] = 0x00;


		}

	}

	chk = Transmit_checksum(_DG_tx_buf[3], 22);
	if(_DG_tx_buf[3][22] == chk)
	{

		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[3][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[3][23] = 0x03;
		_DG_tx_buf[3][24] = 0x0d;
		_DG_tx_buf[3][25] = 0x0a;


		if(CX_FALSE == _flag_COM4_tx_done)
		{
			HAL_GPIO_WritePin(RX_4_TX_LED_GPIO_Port, RX_4_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM4_rx_data); k++)
		{
			_COM4_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}

		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[3][i]);
			_DG_tx_buf[3][i] = 0x00;

		}


	}
	chk = Transmit_checksum(_DG_tx_buf[4], 22);
	if(_DG_tx_buf[4][22] == chk)
	{

		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[4][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[4][23] = 0x03;
		_DG_tx_buf[4][24] = 0x0d;
		_DG_tx_buf[4][25] = 0x0a;


		if(CX_FALSE == _flag_COM5_tx_done)
		{
			HAL_GPIO_WritePin(RX_5_TX_LED_GPIO_Port, RX_5_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM5_rx_data); k++)
		{
			_COM5_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}

		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[4][i]);
			_DG_tx_buf[4][i] = 0x00;


		}


	}
	chk = Transmit_checksum(_DG_tx_buf[5], 22);
	if(_DG_tx_buf[5][22] == chk)
	{
		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[5][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[5][23] = 0x03;
		_DG_tx_buf[5][24] = 0x0d;
		_DG_tx_buf[5][25] = 0x0a;


		if(CX_FALSE == _flag_COM6_tx_done)
		{
			HAL_GPIO_WritePin(RX_6_TX_LED_GPIO_Port, RX_6_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM6_rx_data); k++)
		{
			_COM6_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}

		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[5][i]);
			_DG_tx_buf[5][i] = 0x00;

		}



	}
	chk = Transmit_checksum(_DG_tx_buf[6], 22);
	if(_DG_tx_buf[6][22] == chk)
	{
		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[6][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[6][23] = 0x03;
		_DG_tx_buf[6][24] = 0x0d;
		_DG_tx_buf[6][25] = 0x0a;


		if(CX_FALSE == _flag_COM7_tx_done)
		{
			HAL_GPIO_WritePin(RX_7_TX_LED_GPIO_Port, RX_7_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM7_rx_data); k++)
		{
			_COM7_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}

		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[6][i]);
			_DG_tx_buf[6][i] = 0x00;

		}

	}
	chk = Transmit_checksum(_DG_tx_buf[7], 22);
	if(_DG_tx_buf[7][22] == chk)
	{
		static cx_uint8_t i;
		static cx_uint8_t j;
		static cx_uint8_t k;

		if(_DG_tx_buf[7][22] != chk )
		{
			return 0;
		}

		_DG_tx_buf[7][23] = 0x03;
		_DG_tx_buf[7][24] = 0x0d;
		_DG_tx_buf[7][25] = 0x0a;


		if(CX_FALSE == _flag_COM8_tx_done)
		{
			HAL_GPIO_WritePin(RX_8_TX_LED_GPIO_Port, RX_8_TX_LED_Pin, SET);
		}


		for( k= 0; k< sizeof(_COM8_rx_data); k++)
		{
			_COM8_rx_data[k]= 0x00;

		}

		for(  j = 0; j< 26; j++)
		{
			uarti2cspi_uartWrite(_data_[j]);

		}
		for( i = 0; i< 26; i++)
		{
			uarti2cspi_uartWrite(_DG_tx_buf[7][i]);
			_DG_tx_buf[7][i] = 0x00;
		}

	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void uarti2cspi_initAdvanced( uint16_t baud_rate, uint8_t data_bits, uint8_t parity_mode, uint8_t stop_bits)
{
    uint8_t DLL_val;
    uint8_t DLH_val;
    uint16_t BRR_Reg_Speed;

    HAL_GPIO_WritePin(SPI_RST_GPIO_Port, SPI_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(SPI_RST_GPIO_Port, SPI_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    BRR_Reg_Speed = _calcBaudRate( baud_rate );

    DLH_val = (uint8_t)( ( BRR_Reg_Speed >> 8 ) & 0x00FF );
    DLL_val = (uint8_t)( BRR_Reg_Speed & 0x00FF );

    uarti2cspi_writeReg( UARTI2CSPI_LCR, 0x80    );                             //Enable Special register latch
    uarti2cspi_writeReg( UARTI2CSPI_DLL, DLL_val );                             //DLL
    uarti2cspi_writeReg( UARTI2CSPI_DLH, DLH_val );                             //DLH
    uarti2cspi_writeReg( UARTI2CSPI_LCR, 0xBF    );
    uarti2cspi_writeReg( UARTI2CSPI_EFR, 0       );                             //NO FLOW CONTROLL
    uarti2cspi_writeReg( UARTI2CSPI_LCR, data_bits | parity_mode | stop_bits ); //write UART configuration

//  uarti2cspi_writeReg( UARTI2CSPI_IER, 0x03 );                                //Enable RX,TX interrupt
	uarti2cspi_writeReg( UARTI2CSPI_IER, 0x02 );								//Enable TX interrupt
}

//------------------------------------------------------------------
void uarti2cspi_interruptEnable( uint8_t vect )
{
     uint8_t regTemp = 0u;
     uint8_t flowTmp;

     regTemp=uarti2cspi_readReg( UARTI2CSPI_LCR );     //preserv LCR state

     if( (vect & 0x80) || (vect & 0x40) || (vect & 0x20) || (vect & 0x10) )
     {
        /*
           Enhanced functionality bit must be set before CTS, RTS, XOFF and
           SLEEP mode interrupt enable bits are modified.
         */

        uarti2cspi_writeReg( UARTI2CSPI_LCR, 0xBF );
        flowTmp = uarti2cspi_readReg( UARTI2CSPI_EFR );
        uarti2cspi_writeReg( UARTI2CSPI_EFR, flowTmp | 0x10 );
     }
     else
     {
        uarti2cspi_writeReg( UARTI2CSPI_LCR, 0xBF );
        flowTmp = uarti2cspi_readReg( UARTI2CSPI_EFR );
        flowTmp &= ~0x10;
        uarti2cspi_writeReg( UARTI2CSPI_EFR, flowTmp );
     }

     uarti2cspi_writeReg( UARTI2CSPI_LCR, regTemp );
     uarti2cspi_writeReg( UARTI2CSPI_IER, vect    );

}
//------------------------------------------------------------------
void uarti2cspi_writeReg( uint8_t reg, uint8_t _data )
{
    uint8_t writeReg[ 2 ];

    writeReg[ 0 ] = reg;
    writeReg[ 1 ] = _data;


	if(reg == 0x00)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

		HAL_SPI_Transmit_IT(&hspi1,&writeReg[0], 2);

		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	else
	{

		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_IT(&hspi1,&writeReg[0] ,&writeReg[0],2);
//		HAL_SPI_Transmit_IT(&hspi1,&writeReg[0] ,2);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}

}

//------------------------------------------------------------------
uint8_t uarti2cspi_readReg( uint8_t reg )
{
    uint8_t writeReg[ 2 ];
    writeReg[0] = reg;


    writeReg[0] |= 0x80;

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1,&writeReg[0] ,&writeReg[0],2,30);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    return writeReg[ 1 ];

}
//------------------------------------------------------------------
void uarti2cspi_uartWrite( uint8_t wByte )
{
//	uarti2cspi_writeReg(UARTI2CSPI_FCR,0x05);
	uarti2cspi_writeReg(UARTI2CSPI_FCR,0x04);

    while(!(uarti2cspi_readReg(UARTI2CSPI_LSR) & 0x40));

    HAL_GPIO_TogglePin(SPI_UART_TX_LED_GPIO_Port, SPI_UART_TX_LED_Pin);
	uarti2cspi_writeReg(UARTI2CSPI_THR,wByte);

	while(HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port,SPI_IRQ_Pin));

}
//------------------------------------------------------------------
void uarti2cspi_uartWrite1( uint8_t wByte )
{

    uarti2cspi_writeReg(UARTI2CSPI_FCR,0x04);

    while(!(uarti2cspi_readReg(UARTI2CSPI_LSR) & 0x40));

    uarti2cspi_writeReg(UARTI2CSPI_THR,wByte);

}
//------------------------------------------------------------------
void uarti2cspi_uartText( uint8_t *wText )
{

     while(*wText)
     {
         uarti2cspi_uartWrite1( *wText++ );
		 HAL_Delay(0);
     }
}
//------------------------------------------------------------------
void uarti2cspi_serialWrite(uint8_t *strDat, uint8_t printMode)
{
    if(printMode & _TEXT_PRINT )
    {
        uarti2cspi_uartText( strDat );

        if(printMode & _LINE_PRINT)
        {
            uarti2cspi_uartText("\r\n");
        }
    }
    if(printMode == _CHAR_PRINT)
    {
         uarti2cspi_uartWrite1( *strDat );
    }
}

/////////////////////////////////////////////////////////////////////////////

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
