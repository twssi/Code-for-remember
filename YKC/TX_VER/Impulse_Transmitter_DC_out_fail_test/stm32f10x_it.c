/**
  ******************************************************************************
  * @file    SysTick/TimeBase/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "mcu.h"
#include "main.h"

unsigned long long int _button_cnt=0;
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SysTick_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    /* This interrupt is generated when HSE clock fails */

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
  {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

	/* Enable HSE Ready interrupt */
    RCC_ITConfig(RCC_IT_HSERDY, ENABLE);

    /* Enable PLL Ready interrupt */
    RCC_ITConfig(RCC_IT_PLLRDY, ENABLE);

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}


/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{
    if(RCC_GetITStatus(RCC_IT_HSERDY) != RESET)
  { 
    /* Clear HSERDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_HSERDY);

    /* Check if the HSE clock is still available */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    { 
      /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
      RCC_PLLCmd(ENABLE);
    }
  }

  if(RCC_GetITStatus(RCC_IT_PLLRDY) != RESET)
  { 
    /* Clear PLLRDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLLRDY);

    /* Check if the PLL is still locked */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
    { 
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
  }
}




/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}


/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		com_rx_irq_handler(COM1);
	}
	else
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		com_tx_irq_handler(COM1);
	}
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
//		com_rx_irq_handler(COM2);
	}
	else
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
//		com_tx_irq_handler(COM2);
	}
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		com_rx_irq_handler(COM3);
	}
	else
	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		com_tx_irq_handler(COM3);
	}
}


/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
	/* Test on DMA1 Channel1 Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA1_IT_TC1))	
	{
		/* Clear DMA1 Channel6 Half Transfer, Transfer Complete and Global interrupt pending bits */
		DMA_ClearITPendingBit(DMA1_IT_TC1);
        
//        calculate_adc_irq_handler();	//adc dma
	}
}
/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{		
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		/* Clear TIM2 Capture Compare1 interrupt pending bit*/		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    check_impuls_freqout();
	}
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4) != RESET)	
	{
        /* Clear TIM2 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		
		//calculate_inputcapture_TIM2();	//PIN Freq 
        rising_edge_flag_TIM2();
    }
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	unsigned int type;
	type = get_type_transmitter();
	
	if (TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		/* Clear TIM3 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}

	if (TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)
	{
		/* Clear TIM3 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	
//		if(2u == type)	//supervisor
//		{
//			calculate_inputcapture_TIM3 ();	//PIN clock
      
 //     rising_edge_flag_TIM3();	         
//		}
//		else;
	}
	
}
/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		timer_500usec_irq_handler();
		
		timer_watchdog_irq_handler();
		/* Clear TIM4 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}	
}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)
	{
		get_AC_voltage_irq_handler();
		
		/* Clear TIM4 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}	
}

/*******************************************************************************
* Function Name  : TIM6_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		//50us
		
		
	//
		
		/* Clear TIM4 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}	
}


/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
	{
		output_fnd_display();	//fnd ���
				
		/* Clear TIM4 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}	
}


void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1,TIM_IT_Update) != RESET)
	{
    get_impulse_voltage_irq_handler ();	
    get_tx_current_irq_handler();
      
    /* Clear TIM4 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		/* Clear the EXTI line 2 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
		input_front_mode_button();
   
  }
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
          /* Clear the EXTI line 14 pending bit */
          EXTI_ClearITPendingBit(EXTI_Line7);

     }
}

/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void) // clock feedback
{
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    /* Clear the EXTI line 14 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line11);
    calculate_inputcapture_TIM3 ();	//PIN clock
      
    rising_edge_flag_TIM3();	

  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

