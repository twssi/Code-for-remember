#define __STM32F10X_H__

#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_init.h"

#include "stm32f10x_bit_define.h"

#include "mcu.h"
#include "mcu_gpio_alias.h"

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
ADC_InitTypeDef ADC_InitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef    EXTI_InitStructure;
//DMA_InitTypeDef DMA_InitStructure;	//move to mcu_control_analog.c

/******************************************************************************************************/
/* RCC_initial	:	 Configures the system clocks.						      */												
/******************************************************************************************************/
void RCC_initial(void) 	//PLL configuration
{	
  /* Enable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

  /* This function fills a RCC_ClocksTypeDef structure with the current
     frequencies of different on chip clocks (for debug purpose) */
  RCC_GetClocksFreq(&RCC_ClockFreq);

  /* Enable Clock Security System(CSS) */
  RCC_ClockSecuritySystemCmd(ENABLE);
  
  /* Enable and configure RCC global IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = RCC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}

/******************************************************************************************************/
/* GPIO_port_initial	:								              */												
/******************************************************************************************************/
void GPIO_port_initial(void)
{
	/* Enable GPIOX clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//--------------------------------------------------------------------------
	// INPUT 
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	// Input setting GPIOA
	/* 0~4: ADC input,   */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                         
	
	//--------------------------------------------------------------------------
	// Input setting GPIOB
	/* 1: Track RELAY input, 9: External Watchdog Output, 12~15: TRANS 1~4 Input  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_9 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
	//--------------------------------------------------------------------------
	// Input setting GPIOC
	/* 0~3: HEXA SW 2, 4~5: 절체 계전기 1,2 입력, 7: Pulse freq input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7; /* GPIO_Pin_4 | GPIO_Pin_5 |*/ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    
	
	//--------------------------------------------------------------------------
	// Input setting GPIOD

	
	//--------------------------------------------------------------------------
	// Input setting GPIOE
	/* 0: External Watchdog input, 2~5: HEXA SW 1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);    
	
	//--------------------------------------------------------------------------
	// OUTPUT 
	//--------------------------------------------------------------------------
	// Output setting GPIOA

	//--------------------------------------------------------------------------
	// Output setting GPIOB
	/* 9: External Watchdog output  */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Output setting GPIOC
	/* 6: RUN/FAIL LED, 8: CLR1, 9: CLR2*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//--------------------------------------------------------------------------
	// Output setting GPIOD
	/* 9~16: FND 9~16 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Output setting GPIOE
	/* 7~14: FND 1~8 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	hw_gpio_initialize();
}


/******************************************************************************************************/
/* TIME2_initial	:								              */												
/******************************************************************************************************/
void TIME2_initial(void)
{
  /* TIM2 clock enable */   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 360-1;  //72Mhz/360=20Mhz
  TIM_TimeBaseStructure.TIM_Period = 10-1; // 20Mhz/10=->0.05ms timer

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


/******************************************************************************************************/
/* TIME3_initial_inputCapture	:								              */												
/******************************************************************************************************/
void TIME3_initial(void)
{
  /* TIM2 clock enable */   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* GPIO_Remap: selects the pin to remap.*/
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);	
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 2400-1; 
  TIM_TimeBaseStructure.TIM_Period = 65535-1; 

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  /*******************************************************************/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  /*******************************************************************/	
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE); 

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}



/******************************************************************************************************/
/* TIME4_initial	:								              */												
/******************************************************************************************************/
void TIME4_initial(void){
  /* TIM4 clock enable */   //max - 72mHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  //72Mhz/72=1Mhz
  TIM_TimeBaseStructure.TIM_Period = 500-1; // 1Mhz/500=->0.5ms timer

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  /* TIM enable counter */
  TIM_Cmd(TIM4, DISABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}

/******************************************************************************************************/
/* TIME5_initial	:								              */												
/******************************************************************************************************/
void TIME5_initial(void)
{
  /* TIM5 clock enable */   //max - 72mHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  
  TIM_TimeBaseStructure.TIM_Period = 100-1;	//100us

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 
  /* TIM enable counter */
  TIM_Cmd(TIM5, DISABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM5,TIM_IT_Update, ENABLE);

  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}


/******************************************************************************************************/
/* ADC_initial	:	ADC_ configuration   					      */												
/******************************************************************************************************/
void ADC_initial(void)
{
	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 5;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel 1 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_7Cycles5);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
	//---------------------------------------------------------------------------
    /* Enable DMA1 channel6 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//debug
static void USART1_initial(void)
{
     /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART1 configuration */
    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure the USART1 */
    USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_CTS , DISABLE);
	USART_ITConfig(USART1, USART_IT_LBD , DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE , DISABLE );
	USART_ITConfig(USART1, USART_IT_TC  , DISABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE );
	USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
	USART_ITConfig(USART1, USART_IT_PE  , DISABLE);
	USART_ITConfig(USART1, USART_IT_ERR , ENABLE );
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

	USART_Cmd(USART1, ENABLE);

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//rs-232 To Tx 2
static void USART2_initial(void)
{
     /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
	/* GPIO_Remap: selects the pin to remap.*/
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
	
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* USART2 configuration */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_CTS , DISABLE);
	USART_ITConfig(USART2, USART_IT_LBD , DISABLE);
	USART_ITConfig(USART2, USART_IT_TXE , DISABLE );
	USART_ITConfig(USART2, USART_IT_TC  , DISABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
	USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);
	USART_ITConfig(USART2, USART_IT_PE  , DISABLE);
	USART_ITConfig(USART2, USART_IT_ERR , ENABLE );

	USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
	
	USART_Cmd(USART2, ENABLE);

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//rs-232 To D.G
static void USART3_initial(void)
{
     /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
	/* GPIO_Remap: selects the pin to remap.*/
	
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* USART2 configuration */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure the USART2 */
    USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_CTS , DISABLE);
	USART_ITConfig(USART3, USART_IT_LBD , DISABLE);
	USART_ITConfig(USART3, USART_IT_TXE , DISABLE );
	USART_ITConfig(USART3, USART_IT_TC  , DISABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE );
	USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);
	USART_ITConfig(USART3, USART_IT_PE  , DISABLE);
	USART_ITConfig(USART3, USART_IT_ERR , ENABLE );

	USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);
	
	USART_Cmd(USART3, ENABLE);

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//rs-232 To Tx 1
static void USART4_initial(void)
{
     /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  
	
	/* GPIO_Remap: selects the pin to remap.*/
	
	
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


	/* USART2 configuration */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure the USART2 */
    USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_CTS , DISABLE);
	USART_ITConfig(UART4, USART_IT_LBD , DISABLE);
	USART_ITConfig(UART4, USART_IT_TXE , DISABLE );
	USART_ITConfig(UART4, USART_IT_TC  , DISABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE );
	USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
	USART_ITConfig(UART4, USART_IT_PE  , DISABLE);
	USART_ITConfig(UART4, USART_IT_ERR , ENABLE );

	USART_DMACmd(UART4, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(UART4, USART_DMAReq_Rx, DISABLE);
	
	USART_Cmd(UART4, ENABLE);

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel                   = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI_initial(void)
{
	 /* Enable GPIOD clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
	
	/* Configure PE15 as input floating (EXTI Line8,9,14,15) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
	 
	/* Enable the EXTI9_5 and EXTI15_10 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI Line8,9,14,15 to PD.08 / PD.09 / PD.14 /PD.15 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource15 );  
	
	/* Configure EXTI Line8,9,14,15 to generate an interrupt on falling edge */  
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

}

/******************************************************************************************************/
/* peripheral_initial	:		        						      */												
/******************************************************************************************************/
void peripheral_initial(void) 
{
	RCC_initial();				// PLL Initial
	
	GPIO_port_initial();		// gpio initial
	
	ADC_initial();			// ADC
    DMA_initial();          // DMA for adc
	
	USART1_initial();		//RS-232 debug
	USART2_initial();		//RS-232 to Tx 2계
	USART3_initial();		//RS-232 to D.G
	USART4_initial();		//RS-232 to Tx 1계
	
	TIME2_initial();		// timer2, current sense 
	TIME3_initial();		// timer3, Frequency	
	TIME4_initial();		// timer4, Run Timer
    TIME5_initial();
	
	EXTI_initial();
}
/******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/


