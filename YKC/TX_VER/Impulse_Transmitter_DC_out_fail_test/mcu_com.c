/////////////////////////////////////////////////////////////////////////////
//===========================================================================
//
// ref: https://www.digikey.com/eewiki/display/microcontroller/Software+FIFO+Buffer+for+UART+Communication
//





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>

#include "stm32f10x_init.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"

#include "type.h"
#include "bfifo.h"
#include "bsb.h"
#include "checksum.h"

#include "mcu_com.h"



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
io_fifo_t _com_fifo[COM_MAX_PORT];

//===========================================================================
#ifndef DEBUG

static USART_TypeDef* _MCU_USART[COM_MAX_PORT] = { USART1, USART2, USART3 };

#else

static USART_TypeDef* _MCU_USART[COM_MAX_PORT] = { CX_NULL_POINTER, CX_NULL_POINTER, CX_NULL_POINTER};

#endif





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#ifndef DEBUG

void com_initialize (void)
{
}

#else

void com_initialize (void)
{
	_MCU_USART[0] = USART1;
	_MCU_USART[1] = USART2;
	_MCU_USART[2] = USART3;
	_MCU_USART[3] = UART4;
//	_MCU_USART[4] = UART5;
}

#endif





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void com_rx_irq_handler (cx_uint_t com_port)
{
	bfifo_t* fifo;
	cx_byte_t ch;


	fifo = &_com_fifo[com_port].rx;


	//-----------------------------------------------------------------------
	// TODO:
	// Explicitly clear the source of interrupt if necessary
//	USART_ClearITPendingBit(_MCU_USART[com_port], USART_IT_RXNE);
	//-----------------------------------------------------------------------


	//-----------------------------------------------------------------------
	// TODO:
	// read error/status reg here if desired        
	//-----------------------------------------------------------------------


	//-----------------------------------------------------------------------
	// TODO:
	// handle any hardware RX errors here if desired
	//-----------------------------------------------------------------------
		
		
	//-----------------------------------------------------------------------
	// TODO:
	ch = (cx_byte_t)(_MCU_USART[com_port]->DR & (u16)0x01FF);
	//-----------------------------------------------------------------------


	bfifo_push(fifo, ch);
}

void com_tx_irq_handler (cx_uint_t com_port)
{
	bfifo_t* fifo;
	cx_byte_t ch;
	cx_bool_t retval;

	fifo = &_com_fifo[com_port].tx;


	//-----------------------------------------------------------------------
	// TODO:
	// Explicitly clear the source of interrupt if necessary
//	USART_ClearITPendingBit(_MCU_USART[com_port], USART_IT_TXE);
	//-----------------------------------------------------------------------



	retval = bfifo_pop(fifo, &ch);


	if (CX_TRUE==retval) 
	{
		_MCU_USART[com_port]->DR = (ch & (u16)0x01FFu);
	}


	if (CX_FALSE==retval) 
	{
		//-------------------------------------------------------------------
		// TODO:
		// disable "TX hw buffer empty" interrupt here            
		USART_ITConfig(_MCU_USART[com_port], USART_IT_TXE, DISABLE);
		//-------------------------------------------------------------------


		//-------------------------------------------------------------------
		// TODO:
		// if using shared RX/TX hardware buffer, enable "RX data received" interrupt here 
/*
		{
		USART_ITConfig(_MCU_USART[com_port], USART_IT_RXNE, ENABLE);
		}
*/
		//-------------------------------------------------------------------
	}
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t com_send_byte (cx_uint_t com_port, cx_byte_t ch)
{
	bfifo_t* fifo;
	cx_bool_t retval;


	fifo = &_com_fifo[com_port].tx;


	retval = bfifo_push(fifo, ch);


	if (CX_TRUE==retval)
	{
		//------------------------------------------------------------------
		// TODO: 
		// if using shared RX/TX hardware buffer, disable "RX data received" interrupt here
		/*
		{
		USART_ITConfig(_MCU_USART[com_port], USART_IT_RXNE, DISABLE);
		}
		*/
		//------------------------------------------------------------------


		//------------------------------------------------------------------
		// TODO: 
		// enable "TX hw buffer empty" interrupt here                      
		USART_ITConfig(_MCU_USART[com_port], USART_IT_TXE, ENABLE);
		//------------------------------------------------------------------
	}	
	

	return retval;
}

#ifndef MCU_SIMULATOR

cx_uint_t com_send_buffer (cx_uint_t com_port, cx_byte_t* pointer, cx_uint_t size)
{
	cx_uint_t i;

	
	for (i=0u; i<size; i++)
	{
		if (!com_send_byte(com_port, *(pointer+i)))
		{
			break;
		}
	}
	
	return i;
}

#endif

//===========================================================================
cx_bool_t com_recv_byte (cx_uint_t com_port, cx_byte_t* ch)
{
	bfifo_t* fifo;


	fifo = &_com_fifo[com_port].rx;


	return bfifo_pop(fifo, ch);
}

#ifndef MCU_SIMULATOR

cx_uint_t com_recv_buffer (cx_uint_t com_port, cx_byte_t* pointer, cx_uint_t size)
{
	cx_uint_t i;


	for (i=0u; i<size; i++)
	{
		if (!com_recv_byte(com_port, (pointer+i)))
		{
			break;
		}
	}

	return i;
}

#endif

//===========================================================================
void com_flush (cx_uint_t com_port)
{
	bfifo_t* fifo;
	
	
	fifo = &_com_fifo[com_port].tx;

	while (!fifo->underflow_flag);
}

void com_check_error (cx_uint_t com_port)
{
	bfifo_t* fifo;
	
	
	fifo = &_com_fifo[com_port].rx;

	if (CX_TRUE==fifo->overflow_flag) 
	{
		fifo->overflow_flag = CX_FALSE;

		//printf ("\nCOM%d rx overflow\n", com_port+1u);
	}
	if (CX_TRUE==fifo->underflow_flag) 
	{
		fifo->underflow_flag =CX_FALSE;

		//printf ("\nCOM%d rx underflow\n", com_port+1u);
	}


	fifo = &_com_fifo[com_port].tx;

	if (CX_TRUE==fifo->overflow_flag) 
	{
		fifo->overflow_flag = CX_FALSE;

		//printf ("\nCOM%d tx overflow\n", com_port+1u);
	}
	if (CX_TRUE==fifo->underflow_flag) 
	{
		fifo->underflow_flag = CX_FALSE;

		//printf ("\nCOM%d tx underflow\n", com_port+1u);
	}

	//io_fifo_clear_error (&_com_fifo[com_port]);
}




