/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "type.h"
#include "bsb.h"
#include "checksum.h"
#include "bfifo.h"

#include "message_queue.h"






/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t message_queue_initialize (message_queue_t* ctx, cx_uint_t message_max_count, cx_uint_t message_max_size, message_t* message, cx_byte_t* message_buffer)
{
	cx_uint_t i;


	ctx->message_max_count = message_max_count;
	ctx->message_max_size  = message_max_size ;

	ctx->message = message;


	for (i=0u; i<message_max_count; i++)
	{
		ctx->message[i].pointer = message_buffer + (i*message_max_size);
		ctx->message[i].size    = 0u;
	}

	ctx->message_count   = 0u;
	ctx->message_wcursor = 0u;
	ctx->message_rcursor = 0u;


	return CX_TRUE;
}

//===========================================================================
cx_uint_t message_queue_count (message_queue_t* ctx)
{
	return ctx->message_count;
}

void message_queue_clear (message_queue_t* ctx)
{
	ctx->message_count   = 0u;
	ctx->message_wcursor = 0u;
	ctx->message_rcursor = 0u;
}

cx_bool_t message_queue_push (message_queue_t* ctx, cx_byte_t* pointer, cx_uint_t size)
{
	if (ctx->message_max_count <= ctx->message_count)
	{
//		ctx->message_count = ctx->message_max_count;
		return CX_FALSE;
	}
	if (ctx->message_max_size < size)
	{
		return CX_FALSE;
	}

	
	message_t* m;


	m = &ctx->message[ctx->message_wcursor];

	
	memcpy (m->pointer, pointer, size);
	m->size = size;


	ctx->message_wcursor = (ctx->message_wcursor+1u) % ctx->message_max_count;
	ctx->message_count++;


	return CX_TRUE;
}

cx_bool_t message_queue_pop (message_queue_t* ctx, cx_byte_t* pointer, cx_uint_t* size)
{
	if (0u==ctx->message_count)
	{
		*size = 0u;
		return CX_FALSE;
	}


	message_t* m;
	cx_uint_t copy_size;


	m = &ctx->message[ctx->message_rcursor];


	copy_size = (m->size > (*size)) ? (*size) : m->size;
	memcpy (pointer, m->pointer, copy_size);
	*size = copy_size;


	ctx->message_rcursor = (ctx->message_rcursor+1u) % ctx->message_max_count;
	ctx->message_count--;


	return CX_TRUE;
}


