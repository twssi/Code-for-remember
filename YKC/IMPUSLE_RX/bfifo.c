/****************************************************************************
**
** File: bfifo.c
**
** Created by MOON, Eui-kwon.
** Created on Jan-15th, 2010.
**
****************************************************************************/





/****************************************************************************
**
** Headers
**
****************************************************************************/
/*=========================================================================*/
#include "bfifo.h"





/****************************************************************************
**
** Global functions
**
****************************************************************************/
/*=========================================================================*/
void bfifo_initialize (bfifo_t* ctx, bfifo_byte_t* pointer, bfifo_uint_t size)
{
	ctx->pointer = pointer;
	ctx->maxsize = size;

	ctx->front = 0u;
	ctx->back  = 0u;

	ctx->overflow_flag  = BFIFO_FALSE;
	ctx->underflow_flag = BFIFO_FALSE;
}

bfifo_bool_t bfifo_push (bfifo_t* ctx, bfifo_byte_t ch)
{
	bfifo_uint_t front;
	bfifo_uint_t back ;
	bfifo_uint_t next ;


	back  = ctx->back;
	front = ctx->front;
	next  = (front+1u) % ctx->maxsize;


	if (next==back)
	{
		ctx->overflow_flag = BFIFO_TRUE;


		return BFIFO_FALSE;
	}


	ctx->pointer[front] = ch;

	ctx->front = next;


	return BFIFO_TRUE;
}

bfifo_bool_t bfifo_pop (bfifo_t* ctx, bfifo_byte_t* ch)
{
	bfifo_uint_t front;
	bfifo_uint_t back ;
	bfifo_uint_t next ;


	front = ctx->front;
	back  = ctx->back;
	next  = (back+1u) % ctx->maxsize;


	if (back==front)
	{
		ctx->underflow_flag = BFIFO_TRUE;


		return BFIFO_FALSE;
	}


	*ch = ctx->pointer[back];

	ctx->back = next;


	return BFIFO_TRUE;
}

void bfifo_clear_error (bfifo_t* ctx)
{
	ctx->overflow_flag  = BFIFO_FALSE;
	ctx->underflow_flag = BFIFO_FALSE;
}



/*=========================================================================*/
void io_fifo_initialize (io_fifo_t* ctx, bfifo_byte_t* rx_pointer, bfifo_uint_t rx_size, bfifo_byte_t* tx_pointer, bfifo_uint_t tx_size)
{
	bfifo_initialize(&ctx->rx, rx_pointer, rx_size);
	bfifo_initialize(&ctx->tx, tx_pointer, tx_size);
}

void io_fifo_clear_error (io_fifo_t* ctx)
{
	bfifo_clear_error(&ctx->rx);
	bfifo_clear_error(&ctx->tx);
}
