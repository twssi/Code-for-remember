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
#include "ctype.h"
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
/////////////////////?��?���?//////////////////////////
static cx_bool_t _flag_transmit  		= CX_FALSE;

/////////////////////?��?���?//////////////////////////


//////////////////링버?��, ?��?��버퍼///////////////////////////

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


//////////////////링버?��, ?��?��버퍼///////////////////////////




///////////////////////UART ?��?��////////////////////////
static cx_uint8_t COM1_data =0;
static cx_uint8_t COM2_data =0;
static cx_uint8_t COM3_data =0;
static cx_uint8_t COM4_data =0;
static cx_uint8_t COM5_data =0;
static cx_uint8_t COM6_data =0;
static cx_uint8_t COM7_data =0;
static cx_uint8_t COM8_data =0;


/*
static cx_uint8_t _COM1_rx_data [150];
static cx_uint8_t _COM2_rx_data	[150];
static cx_uint8_t _COM3_rx_data	[150];
static cx_uint8_t _COM4_rx_data	[150];
static cx_uint8_t _COM5_rx_data	[150];
static cx_uint8_t _COM6_rx_data	[150];
static cx_uint8_t _COM7_rx_data	[150];
static cx_uint8_t _COM8_rx_data	[150];
cx_uint8_t deparsebuf1[50];
cx_uint8_t deparsebuf2[50];
cx_uint8_t deparsebuf3[50];
cx_uint8_t deparsebuf4[50];
cx_uint8_t deparsebuf5[50];
cx_uint8_t deparsebuf6[50];
cx_uint8_t deparsebuf7[50];
cx_uint8_t deparsebuf8[50];

*/


static cx_uint8_t 		_DG_tx_buf[9][30];
static cx_uint8_t _data_[30] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0xA0, 0x03, 0x0D, 0x0A};


///////////////////////UART ?��?��////////////////////////


////////////////////////???���?////////////////////
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

////////////////////////???���?////////////////////


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
cx_uint32_t Packet_Verify(cx_uint8_t* bufptr, cx_uint32_t bufsize, cx_uint32_t max, cx_uint32_t* packetsize);

void ascii_convert_to_hex(cx_uint8_t *asciidata, cx_uint8_t* hex_values, cx_uint_t data_size);
void ascii_to_bcd(cx_uint8_t *asciidata, cx_uint8_t* bcd_values, cx_uint_t data_size);

void uartinput1(void);
void uartinput2(void);
void uartinput3(void);
void uartinput4(void);
void uartinput5(void);
void uartinput6(void);
void uartinput7(void);
void uartinput8(void);

void transmit_reset(void);
void initial_GPIO_LED(void);
void transmit(void);

void OnUart1_Recv (cx_uint8_t ch);
void OnUart2_Recv (cx_uint8_t ch);
void OnUart3_Recv (cx_uint8_t ch);
void OnUart4_Recv (cx_uint8_t ch);
void OnUart5_Recv (cx_uint8_t ch);
void OnUart6_Recv (cx_uint8_t ch);
void OnUart7_Recv (cx_uint8_t ch);
void OnUart8_Recv (cx_uint8_t ch);

void COM_Input_Parse1(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse2(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse3(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse4(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse5(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse6(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse7(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);
void COM_Input_Parse8(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port);

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

    /* USER CODE BEGIN 3 */

	uartinput1();
	uartinput2();
	uartinput3();
	uartinput4();
	uartinput5();
	uartinput6();
	uartinput7();
	uartinput8();

	transmit_reset();

	if(_flag_transmit == CX_TRUE)
	{
	  transmit();

	  _flag_transmit = CX_FALSE;

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
  RCC_OscInitStruct.PLL.PLLM = 4;
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


//////////LED 초기?��//////////////////////////////////////////////////////////


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





////////////////////////////???���?////////////////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

		if(htim->Instance == TIM3)
		{
        ////////////SPI ?��?�� ???���?/////////////
			_timer_count_0020_msec++;
        ///////////////////////////////////
			_COM1_rx_timeout_count++;
			_COM2_rx_timeout_count++;
			_COM3_rx_timeout_count++;
			_COM4_rx_timeout_count++;
			_COM5_rx_timeout_count++;
			_COM6_rx_timeout_count++;
			_COM7_rx_timeout_count++;
			_COM8_rx_timeout_count++;

			_COM1_tx_timeout_count++;
			_COM2_tx_timeout_count++;
			_COM3_tx_timeout_count++;
			_COM4_tx_timeout_count++;
			_COM5_tx_timeout_count++;
			_COM6_tx_timeout_count++;
			_COM7_tx_timeout_count++;
			_COM8_tx_timeout_count++;


			if(_COM1_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_1_RX_LED_GPIO_Port, RX_1_RX_LED_Pin,SET);

			}
			if(_COM2_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_2_RX_LED_GPIO_Port, RX_2_RX_LED_Pin,SET);

			}
			if(_COM3_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_3_RX_LED_GPIO_Port, RX_3_RX_LED_Pin,SET);
			}
			if(_COM4_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_4_RX_LED_GPIO_Port, RX_4_RX_LED_Pin,SET);

			}
			if(_COM5_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_5_RX_LED_GPIO_Port, RX_5_RX_LED_Pin,SET);

			}
			if(_COM6_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_6_RX_LED_GPIO_Port, RX_6_RX_LED_Pin,SET);

			}
			if(_COM7_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_7_RX_LED_GPIO_Port, RX_7_RX_LED_Pin,SET);

			}
			if(_COM8_rx_timeout_count>5)
			{
				HAL_GPIO_WritePin(RX_8_RX_LED_GPIO_Port, RX_8_RX_LED_Pin,SET);

			}



			if(_timer_count_0020_msec > 1000)
			{
				_flag_transmit = CX_TRUE;
				_timer_count_0020_msec=0;
			}


		}


}


////////////////////////?��?�� ?��?��?��?��///////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)
	{
		_COM1_rx_timeout_count = 0;
		_COM1_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_1_RX_LED_GPIO_Port, RX_1_RX_LED_Pin,RESET);

		OnUart1_Recv(COM1_data);
		HAL_UART_Receive_IT(&huart1, &COM1_data, 1);

	}

	if(huart->Instance == USART2)
	{
		_COM2_rx_timeout_count = 0;
		_COM2_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_2_RX_LED_GPIO_Port, RX_2_RX_LED_Pin,RESET);

		OnUart2_Recv(COM2_data);
		HAL_UART_Receive_IT(&huart2, &COM2_data, 1);


	}

	if(huart->Instance == USART3)
	{
		_COM3_rx_timeout_count = 0;
		_COM3_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_3_RX_LED_GPIO_Port, RX_3_RX_LED_Pin,RESET);

		OnUart3_Recv(COM3_data);
		HAL_UART_Receive_IT(&huart3, &COM3_data, 1);

	}

	if(huart->Instance == UART4)
	{
		_COM4_rx_timeout_count = 0;
		_COM4_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_4_RX_LED_GPIO_Port, RX_4_RX_LED_Pin,RESET);

		OnUart4_Recv(COM4_data);
		HAL_UART_Receive_IT(&huart4, &COM4_data, 1);

	}

	if(huart->Instance == UART5)
	{
		_COM5_rx_timeout_count = 0;
		_COM5_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_5_RX_LED_GPIO_Port, RX_5_RX_LED_Pin,RESET);

		OnUart5_Recv(COM5_data);
		HAL_UART_Receive_IT(&huart5, &COM5_data, 1);

	}

	if(huart->Instance == USART6)
	{

		_COM6_rx_timeout_count = 0;
		_COM6_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_6_RX_LED_GPIO_Port, RX_6_RX_LED_Pin,RESET);

		OnUart6_Recv(COM6_data);
		HAL_UART_Receive_IT(&huart6, &COM6_data, 1);

	}

	if(huart->Instance == UART7)
	{
		_COM7_rx_timeout_count = 0;
		_COM7_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_7_RX_LED_GPIO_Port, RX_7_RX_LED_Pin,RESET);

		OnUart7_Recv(COM7_data);
		HAL_UART_Receive_IT(&huart7, &COM7_data, 1);


	}

	if(huart->Instance == UART8)
	{
		_COM8_rx_timeout_count = 0;
		_COM8_tx_timeout_count =0;

		HAL_GPIO_WritePin(RX_8_RX_LED_GPIO_Port, RX_8_RX_LED_Pin,RESET);

		OnUart8_Recv(COM8_data);
		HAL_UART_Receive_IT(&huart8, &COM8_data, 1);

	}



}



//////////////////////////////////UART circular buf write///////////////////////////////////////////



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

//////////////////////////////////Receive packet verify//////////////////////////////////////////////////////////////////////////////////////
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


cx_uint32_t Packet_Verify(cx_uint8_t* bufptr, cx_uint32_t bufsize, cx_uint32_t max, cx_uint32_t* packetsize)
{
		cx_uint32_t  offset;
		cx_uint32_t  result;

		cx_uint8_t etx;
		cx_uint8_t stx;
		cx_uint8_t chk;

		offset = 0;
		result = 0;




		if (bufsize>max)
		{
			goto cleanup;
		}

		etx = bufptr[bufsize-1];

		stx = bufptr[offset];
		if (stx!=0x02)
		{
			offset+=1;
			goto cleanup;
		}

		if (etx!=0x0A)
		{
			offset+=1;
			goto cleanup;
		}

		chk = fcs_rx(&bufptr[bufsize-45], 40);

		if (bufptr[bufsize-5]!=fcs_conv(chk>>4))
		{
			offset+=1;
			goto cleanup;
		}

		if (bufptr[bufsize-4]!=fcs_conv(chk&0x0f))
		{
			offset+=1;
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////UART input process////////////////////////////////////////////////////////

void uartinput1(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart1_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart1_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse1(buf, max, 1);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart1_rxringbuf, buf, offset);


	}
}
void uartinput2(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size   = 0;
	static cx_uint_t  max    = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart2_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart2_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse2(buf, max, 2);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart2_rxringbuf, buf, offset);


	}
}

void uartinput3(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;


	while (ringbuf_get_readable_space(&g_uart3_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart3_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse3(buf, max, 3);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart3_rxringbuf, buf, offset);


	}
}

void uartinput4(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;



	while (ringbuf_get_readable_space(&g_uart4_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart4_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse4(buf, max, 4);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart4_rxringbuf, buf, offset);


	}
}


void uartinput5(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart5_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart5_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse5(buf, max, 5);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart5_rxringbuf, buf, offset);


	}
}
void uartinput6(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart6_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart6_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse6(buf, max, 6);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart6_rxringbuf, buf, offset);


	}
}

void uartinput7(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart7_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart7_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse7(buf, max, 7);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart7_rxringbuf, buf, offset);


	}
}

void uartinput8(void)
{

	static cx_uint8_t buf[46];
	static cx_uint8_t size =0;
	static cx_uint_t  max  = 46;
	static cx_uint32_t  offset =0;

	while (ringbuf_get_readable_space(&g_uart8_rxringbuf) >= 46)
	{
		size = ringbuf_peek(&g_uart8_rxringbuf, buf, max);

		if(Packet_Verify(buf, size, max, &offset))
		{
			COM_Input_Parse8(buf, max, 8);
		}

		else
		{
			if (0==offset)
			{
				break;
			}
		}
		ringbuf_read (&g_uart8_rxringbuf, buf, offset);


	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////UARTbuffer to SPIbuffer//////////////////////////////////////////////////////////////////
void COM_Input_Parse1(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{


	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 1)  // COM1 처리
    {

		memcpy(parsebuf, bufptr, 46);


        ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
        ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
        parsebuf[len-45] = 0x01;

        memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
        memcpy(&parsebuf[len-43],val_savebuf, 19);



        chk = Transmit_checksum(&parsebuf[len-46], 22);
        parsebuf[len-24] = chk;


        memcpy(&_DG_tx_buf[1][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00, 46);
        memset(val_savebuf, 0x00, 26);


    }

}

void COM_Input_Parse2(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 2)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x02;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		 chk = Transmit_checksum(&parsebuf[len-46], 22);
		 parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[2][1],&parsebuf[len-45], 22);


        memset(parsebuf, 0x00, 46);
        memset(val_savebuf, 0x00, 26);

	}
}

void COM_Input_Parse3(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 3)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x03;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		 chk = Transmit_checksum(&parsebuf[len-46], 22);
		 parsebuf[len-24] = chk;



		memcpy(&_DG_tx_buf[3][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00, sizeof(parsebuf));
        memset(val_savebuf, 0x00, sizeof(parsebuf));
	}

}

void COM_Input_Parse4(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;



	if (com_port == 4)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x04;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		chk = Transmit_checksum(&parsebuf[len-46], 22);
		parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[4][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00, 48);
        memset(val_savebuf, 0x00, 48);
	}
}

void COM_Input_Parse5(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 5)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x05;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		chk = Transmit_checksum(&parsebuf[len-46], 22);
		parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[5][1],&parsebuf[len-45], 22);


        memset(parsebuf, 0x00,48);
        memset(val_savebuf, 0x00, 48);
	}
}

void COM_Input_Parse6(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;


	if (com_port == 6)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x06;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		chk = Transmit_checksum(&parsebuf[len-46], 22);
		parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[6][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00, 48);
        memset(val_savebuf, 0x00, 48);
	}
}

void COM_Input_Parse7(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 7)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x07;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		chk = Transmit_checksum(&parsebuf[len-46], 22);
		parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[7][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00, 48);
        memset(val_savebuf, 0x00, 48);

	}
}

void COM_Input_Parse8(cx_uint8_t* bufptr, cx_uint_t len, cx_uint_t com_port)
{
	static cx_uint8_t parsebuf[46];
	static cx_uint8_t val_savebuf[26];
	static cx_uint8_t hax_val_savebuf[2];
	static cx_uint8_t chk;
	chk =0;

	if (com_port == 8)
	{
		memcpy(parsebuf, bufptr, 46);


		ascii_to_bcd(&parsebuf[len-43], val_savebuf, 38);
		ascii_convert_to_hex(&parsebuf[len-45], hax_val_savebuf, 2);
		parsebuf[len-45] = 0x08;

		memcpy(&parsebuf[len-44],hax_val_savebuf, 1);
		memcpy(&parsebuf[len-43],val_savebuf, 19);



		chk = Transmit_checksum(&parsebuf[len-46], 22);
		parsebuf[len-24] = chk;


		memcpy(&_DG_tx_buf[8][1],&parsebuf[len-45], 22);

        memset(parsebuf, 0x00,48);
        memset(val_savebuf, 0x00,48);

	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////ascii to hex ///////////////////////////////////////////////////////////////////////////////////////
void ascii_convert_to_hex(cx_uint8_t *asciidata, cx_uint8_t* hex_values, cx_uint_t data_size)
{

	cx_uint8_t high_nibble = 0;
	cx_uint8_t low_nibble = 0;

	for (cx_uint_t j = 0; j < data_size; j += 2) {
		// high nibble 계산
		if (asciidata[j] >= '0' && asciidata[j] <= '9') {
			high_nibble = asciidata[j] - '0'; // 숫자(0-9) 변환
		} else if (asciidata[j] >= 'A' && asciidata[j] <= 'F') {
			high_nibble = asciidata[j] - 'A' + 10; // 대문자(A-F) 변환
		} else if (asciidata[j] >= 'a' && asciidata[j] <= 'f') {
			high_nibble = asciidata[j] - 'a' + 10; // 소문자(a-f) 변환
		}

		// low nibble 계산
		if (asciidata[j + 1] >= '0' && asciidata[j + 1] <= '9') {
			low_nibble = asciidata[j + 1] - '0'; // 숫자(0-9) 변환
		} else if (asciidata[j + 1] >= 'A' && asciidata[j + 1] <= 'F') {
			low_nibble = asciidata[j + 1] - 'A' + 10; // 대문자(A-F) 변환
		} else if (asciidata[j + 1] >= 'a' && asciidata[j + 1] <= 'f') {
			low_nibble = asciidata[j + 1] - 'a' + 10; // 소문자(a-f) 변환
		}

		// 변환된 HEX 값을 배열에 저장
		hex_values[j / 2] = (high_nibble << 4) | low_nibble;
	}

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////ascii to bcd //////////////////////////////////////////////////////////////////////////////////////

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

            j++; // ?��?�� ASCII 문자�? ?��?��
        }
        else
        {
        	bcd_values[j / 2] = 0; // ASCII�? ?��?���? ?��?�� 경우 0?���? ?��?��
        }
	}

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////// SPI transmit checksum ////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void transmit_reset()
{


	if(_COM1_tx_timeout_count > 700)
	{
		_COM1_tx_timeout_count = 800;
		memset(_DG_tx_buf[1], 0x00,26);
	}
	if(_COM2_tx_timeout_count > 700)
	{
		_COM2_tx_timeout_count = 800;
		memset(_DG_tx_buf[2], 0x00,26);
	}
	if(_COM3_tx_timeout_count > 700)
	{
		_COM3_tx_timeout_count = 800;
		memset(_DG_tx_buf[3], 0x00,26);
	}
	if(_COM4_tx_timeout_count > 700)
	{
		_COM4_tx_timeout_count = 800;
		memset(_DG_tx_buf[4], 0x00,26);
	}
	if(_COM5_tx_timeout_count > 700)
	{
		_COM5_tx_timeout_count = 800;
		memset(_DG_tx_buf[5], 0x00,26);
	}
	if(_COM6_tx_timeout_count > 700)
	{
		_COM6_tx_timeout_count = 800;
		memset(_DG_tx_buf[6], 0x00,26);
	}
	if(_COM7_tx_timeout_count > 700)
	{
		_COM7_tx_timeout_count = 800;
		memset(_DG_tx_buf[7], 0x00,26);
	}
	if(_COM8_tx_timeout_count > 700)
	{
		_COM8_tx_timeout_count = 800;
		memset(_DG_tx_buf[8], 0x00,26);
	}

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////// UART to SPI send ////////////////////////////////////////////////////////
void transmit(void)
{
	static cx_uint8_t chk[9];
	static cx_uint_t i,j,k,l,m;
	cx_uint8_t sum =0;

	memcpy(_DG_tx_buf[0],_data_,26);


	for(k =1; k<9; k++)
	{
		_DG_tx_buf[k][0] =  0x02;
		_DG_tx_buf[k][23] = 0x03;
		_DG_tx_buf[k][24] = 0x0d;
		_DG_tx_buf[k][25] = 0x0a;
	}

	for(m =1; m<9; m++)
	{
		chk[m] = Transmit_checksum(_DG_tx_buf[m], 22);
	}

	for(l =0; l<26; l++)
	{
		uarti2cspi_uartWrite(_DG_tx_buf[0][l]);

	}


	for(i=1; i<9; i++)
	{

		 for (k = 2; k < 22; k++)
		    {
		        sum += _DG_tx_buf[i][k];

		    }

		if( _DG_tx_buf[i][22] == chk[i] && sum !=0x00 && _DG_tx_buf[i][3] != 0x00 )
		{

			for(j=0; j<26; j++)
			{
				uarti2cspi_uartWrite(_DG_tx_buf[i][j]);
			}

		}
		   memset(_DG_tx_buf[i], 0x00, 26);

	}


/*
*/
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
