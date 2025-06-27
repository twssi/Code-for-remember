#ifndef INCLUDED__CX__BUFFER__BFIFO__H
#define INCLUDED__CX__BUFFER__BFIFO__H

/****************************************************************************
**
** File: bfifo.h
**
** Created by MOON, Eui-kwon.
** Created on Jan-15th, 2010.
**
**
****************************************************************************/





/****************************************************************************
**
** Define
**
****************************************************************************/
/*=========================================================================*/
#if defined ( __cplusplus )
#	define CX_API_BFIFO extern "C"
#else
#	define CX_API_BFIFO extern
#endif




/****************************************************************************
**
** Type declarations
**
****************************************************************************/
/*=========================================================================*/
typedef unsigned int  bfifo_uint_t;
typedef unsigned char bfifo_byte_t;
typedef unsigned char bfifo_bool_t;



/*=========================================================================*/
#ifndef BFIFO_TRUE
#define BFIFO_TRUE 1u
#endif

#ifndef BFIFO_FALSE
#define BFIFO_FALSE 0u
#endif





/****************************************************************************
**
** Structures
**
****************************************************************************/
/*=========================================================================*/
typedef struct _bfifo_t 
{
	bfifo_byte_t* pointer;
	bfifo_uint_t  maxsize;

	bfifo_uint_t  front;
	bfifo_uint_t  back;

	volatile bfifo_bool_t overflow_flag;
	volatile bfifo_bool_t underflow_flag;
}
bfifo_t;



/*=========================================================================*/
typedef struct _io_fifo_t
{
	bfifo_t rx;
	bfifo_t tx;
}
io_fifo_t;





/****************************************************************************
**
** Global functions prototypes
**
****************************************************************************/
/*=========================================================================*/
CX_API_BFIFO void bfifo_initialize (bfifo_t* ctx, bfifo_byte_t* pointer, bfifo_uint_t size);

CX_API_BFIFO bfifo_bool_t bfifo_push (bfifo_t* ctx, bfifo_byte_t ch);
CX_API_BFIFO bfifo_bool_t bfifo_pop  (bfifo_t* ctx, bfifo_byte_t* ch);

CX_API_BFIFO void bfifo_clear_error (bfifo_t* ctx);



/*=========================================================================*/
CX_API_BFIFO void io_fifo_initialize (io_fifo_t* ctx, 
                                      bfifo_byte_t* rx_pointer, bfifo_uint_t rx_size, 
                                      bfifo_byte_t* tx_pointer, bfifo_uint_t tx_size);

CX_API_BFIFO void io_fifo_clear_error(io_fifo_t* ctx);





#endif




