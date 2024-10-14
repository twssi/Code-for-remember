#define __STM32F10X_H__

#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_init.h"

#include "stm32f10x_bit_define.h"

#include "mcu.h"
#include "mcu_gpio_alias.h"

/* Private macro -------------------------------------------------------------*/
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
//I2C_InitTypeDef  I2C_InitStructure; 	

/* Private variables ---------------------------------------------------------*/
#define TX_CONTROL 		1u
#define TX_SUPERVISOR	2u

unsigned int _tx_type = 0u;
/******************************************************************************************************/
/* GPIO_port_initial_branch	:								              */												
/******************************************************************************************************/
unsigned int gpio_port_initial_branch(void)
{
	cx_uint_t read_pin6;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//--------------------------------------------------------------------------
	// Input setting GPIOB
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	read_pin6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
	
	if      (0u == read_pin6) _tx_type = TX_CONTROL;
	else if (1u == read_pin6) _tx_type = TX_SUPERVISOR;
	
	return _tx_type;
}

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
/* GPIO_port_initial for Tx Control	 */
/******************************************************************************************************/
void GPIO_port_initial_PWM(void)
{
	/* Enable GPIOX clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	//--------------------------------------------------------------------------
	// INPUT 
	//--------------------------------------------------------------------------
	// Input setting GPIOE
	/* 0: External Watchdog input   */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);    
	
	//--------------------------------------------------------------------------
	// OUTPUT 
	//--------------------------------------------------------------------------
	// Output setting GPIOB
	/* 9: External Watchdog output   */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Output setting GPIOC
	/* 6: External Clock output   */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}

/******************************************************************************************************/
/* TIME3_initial_PWM */												
/******************************************************************************************************/
void TIME3_initial_PWM(void)
{
  /* GPIO_Remap: selects the pin to remap.*/
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);	
	
  /* TIM3 clock enable */   //max - 72mHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 2400-1;     //3.0hz  
//  TIM_TimeBaseStructure.TIM_Prescaler = 2482-1; //2.9hz
//  TIM_TimeBaseStructure.TIM_Prescaler = 2322-1; //3.1hz
//  TIM_TimeBaseStructure.TIM_Prescaler = 2460-1; //2.92hz
//  TIM_TimeBaseStructure.TIM_Prescaler = 2335-1; //3.08hz
  TIM_TimeBaseStructure.TIM_Period = 10000-1;

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /*******************************************************************/
 
  TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState	= TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse			= 300-1;	//high 10ms
  TIM_OCInitStructure.TIM_OCPolarity	= TIM_OCPolarity_High;
  
  TIM_OC1Init(TIM3,&TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);
 
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
//  TIM_Cmd(TIM3, DISABLE);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
  
  /* TIM IT enable */
  TIM_ITConfig(TIM3,TIM_IT_Update , ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); 
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
	/* 0~4: ADC input, 8: DI-1  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
	//--------------------------------------------------------------------------
	// Input setting GPIOB
	/* 11: Read output Pulse Freq  */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//--------------------------------------------------------------------------
	// Input setting GPIOC
	/* 2: mode SW input, 5: tx 1/2계 input  6: Pulse clock pin, 7~9: DI-4~2   */    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Input setting GPIOD
	/* 12~15: DI-8~5   */    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);    
	//--------------------------------------------------------------------------
	// Input setting GPIOE
	/* 0: External Watchdog input , 3: MCU_Manual_Switch INPUT  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);    
	
	//--------------------------------------------------------------------------
	// OUTPUT 
	//--------------------------------------------------------------------------
	// Output setting GPIOA
	/* 6: CLR1, 7: CLR2  */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//--------------------------------------------------------------------------
	// Output setting GPIOB
	/* 1: MCU_SMPS_OUT_Control, 9: External Watchdog output, 10: Switchover_Control, 12~15: DO-8~5  */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Output setting GPIOC
	/* 1: RUN/FAIL LED   */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//--------------------------------------------------------------------------
	// Output setting GPIOD
	/* 0~7: FND9~16, 8~11: DO-4~1 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//--------------------------------------------------------------------------
	// Output setting GPIOE
	/* 7~14: FND1~8 */
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
  
  /* GPIO_Remap: selects the pin to remap.*/
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);	
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 2400-1;  
  TIM_TimeBaseStructure.TIM_Period = 65535-1; 

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
  /*******************************************************************/
//  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  /*******************************************************************/	
  
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE); 

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


/******************************************************************************************************/
/* TIME3_initial_inputCapture	:								              */												
/******************************************************************************************************/
void TIME3_initial(void)
{
  /* TIM3 clock enable */   
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
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
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
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); 

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}

/******************************************************************************************************/
/* TIME4_initial	:								              */												
/******************************************************************************************************/
void TIME4_initial(void)
{
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

  /* Enable the TIM4 global Interrupt */
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
  TIM_TimeBaseStructure.TIM_Prescaler = 1200-1;  
//  TIM_TimeBaseStructure.TIM_Period = 10-1;	//sampling 100
  TIM_TimeBaseStructure.TIM_Period = 5-1;		//sampling 200

  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 
  /* TIM enable counter */
  TIM_Cmd(TIM5, DISABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM5,TIM_IT_Update, ENABLE);

  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}

/******************************************************************************************************/
/* TIME6_initial	:								              */												
/******************************************************************************************************/
void TIME6_initial(void)
{
  /* TIM6 clock enable */   //max - 72mHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 360-1;  
  TIM_TimeBaseStructure.TIM_Period = 10-1; 
  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
 
  /* TIM enable counter */
  TIM_Cmd(TIM6, DISABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}


/******************************************************************************************************/
/* TIME6_initial	:								              */												
/******************************************************************************************************/
void TIME7_initial(void)
{
  /* TIM6 clock enable */   //max - 72mHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  
  TIM_TimeBaseStructure.TIM_Period = 50-1; //100us
  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
 
  /* TIM enable counter */
  TIM_Cmd(TIM7, DISABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM7,TIM_IT_Update, ENABLE);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 5;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel 1 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
	
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
    USART_InitStructure.USART_BaudRate = 9600;
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


static void USART3_initial(void)
{ 
	/* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* GPIO_Remap: selects the pin to remap.*/
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* USART3 configuration */
    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure the USART3 */
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

	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void EXTI_initial(void)
{
	 /* Enable GPIOC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
		 
	/* Enable the EXTI2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI Line2 to PC.02 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2 );  
	
	/* Configure EXTI Line2 to generate an interrupt on rising edge */  
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#if 0
static void i2c_initial(void)
{ 	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7; 
	I2C_InitStructure.I2C_OwnAddress1 = 0x70; 
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_Cmd(I2C1, ENABLE);
}

void DMA_initial(void)
{
    /* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)_ADC_ValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_BufferSize = 5;
    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    //인터럽트
	/* Enable DMA1 Channel6 Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    
    /* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

#endif
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

/******************************************************************************************************/
/* peripheral_initial	*/												
/******************************************************************************************************/
void peripheral_initial(void) 
{
	RCC_initial();					// PLL Initial
	
	TIME4_initial();				// Timer4 , Run timer
	   
	switch (_tx_type)
	{
		case TX_CONTROL:
			GPIO_port_initial_PWM();
			TIME3_initial_PWM();
			break;

		case TX_SUPERVISOR:
			GPIO_port_initial();	// gpio initial
		
			USART1_initial();		//RS-232 to RX
			USART3_initial();		//Debug Port	
		
			ADC_initial();			// ADC
			DMA_initial();          // DMA for adsac
		
			TIME2_initial();		// timer2 , Freq
			TIME3_initial();		// timer3 , Clock

			TIME5_initial();		// timer5, AC voltage sense
			TIME6_initial();		// timer6, current sense
			
			TIME7_initial();		// timer7, FND
		
			EXTI_initial();
			break;

		default:
			break;
	}

}



