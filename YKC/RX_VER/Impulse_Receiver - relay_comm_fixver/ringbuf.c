/////////////////////////////////////////////////////////////////////////////
//
// File: ringbuf.c
//
// Created by MOON, Eui-kwon.
// Created on Jan-15th, 2010.
//
/////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////
//
// Headers
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <string.h>

//===========================================================================
#include "ringbuf.h"



/////////////////////////////////////////////////////////////////////////////
//
// Global functions
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void ringbuf_init (RINGBUF* ringbuf, unsigned char* memory, RINGBUF_SIZE memory_size)
{
	RINGBUF_SIZE size;
	RINGBUF_SIZE power_of_two;


	for (power_of_two = 1; 
	     (RINGBUF_SIZE)(1 << power_of_two) < memory_size; 
		 power_of_two++);

	size = 1 << power_of_two;

	ringbuf->size       = size;
	ringbuf->size_mask  = size-1;
	ringbuf->w_pointer  = 0;
	ringbuf->r_pointer  = 0;
	ringbuf->buf        = memory;
}

void ringbuf_reset (RINGBUF* ringbuf)
{
	ringbuf->r_pointer = 0;
	ringbuf->w_pointer = 0;
}

RINGBUF_SIZE ringbuf_get_readable_space (RINGBUF* ringbuf)
{
	RINGBUF_SIZE w, r;

	w = ringbuf->w_pointer;
	r = ringbuf->r_pointer;

	if (w > r) 
	{
		return w - r;
	} 

	return (w - r + ringbuf->size) & ringbuf->size_mask;
}

RINGBUF_SIZE ringbuf_get_writable_space (RINGBUF* ringbuf)
{
	RINGBUF_SIZE w, r;

	w = ringbuf->w_pointer;
	r = ringbuf->r_pointer;

	if (w > r) 
	{
		return ((r - w + ringbuf->size) & ringbuf->size_mask) - 1;
	} 
	else if (w < r) 
	{
		return (r - w) - 1;
	} 

	return ringbuf->size - 1;
}

RINGBUF_SIZE ringbuf_peek (RINGBUF* ringbuf, unsigned char* buf, RINGBUF_SIZE size)
{
	RINGBUF_SIZE space;
	RINGBUF_SIZE ring;
	RINGBUF_SIZE to_read;
	RINGBUF_SIZE n1, n2;

	RINGBUF_SIZE r_pointer;

	if ((space = ringbuf_get_readable_space (ringbuf)) == 0) 
	{
		return 0;
	}

	to_read = size > space ? space : size;
	ring    = ringbuf->r_pointer + to_read;

	if (ring > ringbuf->size) 
	{
		n1 = ringbuf->size - ringbuf->r_pointer;
		n2 = ring & ringbuf->size_mask;
	} 
	else 
	{
		n1 = to_read;
		n2 = 0;
	}

	r_pointer = ringbuf->r_pointer;

	memcpy (buf, &(ringbuf->buf[r_pointer]), n1);
	r_pointer += n1;
	r_pointer &= ringbuf->size_mask;

	if (n2) 
	{
		memcpy (buf + n1, &(ringbuf->buf[r_pointer]), n2);
		r_pointer += n2;
		r_pointer &= ringbuf->size_mask;
	}

	return to_read;
}

RINGBUF_SIZE ringbuf_read (RINGBUF* ringbuf, unsigned char* buf, RINGBUF_SIZE size)
{
	RINGBUF_SIZE space;
	RINGBUF_SIZE ring;
	RINGBUF_SIZE to_read;
	RINGBUF_SIZE n1, n2;

	RINGBUF_SIZE r_pointer;
	
	if ((space = ringbuf_get_readable_space (ringbuf)) == 0) 
	{
		return 0;
	}

	to_read = size > space ? space : size;
	ring    = ringbuf->r_pointer + to_read;

	if (ring > ringbuf->size) 
	{
		n1 = ringbuf->size - ringbuf->r_pointer;
		n2 = ring & ringbuf->size_mask;
	} 
	else 
	{
		n1 = to_read;
		n2 = 0;
	}
	
	r_pointer = ringbuf->r_pointer;

	memcpy (buf, &(ringbuf->buf[r_pointer]), n1);
	r_pointer += n1;
	r_pointer &= ringbuf->size_mask;

	if (n2) 
	{
		memcpy (buf + n1, &(ringbuf->buf[r_pointer]), n2);
		r_pointer += n2;
		r_pointer &= ringbuf->size_mask;
	}
	
	ringbuf->r_pointer=r_pointer;
	
	return to_read;
}

RINGBUF_SIZE ringbuf_write (RINGBUF* ringbuf, const unsigned char* buf, RINGBUF_SIZE size)
{
	RINGBUF_SIZE space;
	RINGBUF_SIZE ring;
	RINGBUF_SIZE to_write;
	RINGBUF_SIZE n1, n2;

	RINGBUF_SIZE w_pointer;
	
	if ((space = ringbuf_get_writable_space (ringbuf)) == 0) 
	{
		return 0;
	}

	to_write = size > space ? space : size;
	ring     = ringbuf->w_pointer + to_write;

	if (ring > ringbuf->size) 
	{
		n1 = ringbuf->size - ringbuf->w_pointer;
		n2 = ring & ringbuf->size_mask;
	} 
	else 
	{
		n1 = to_write;
		n2 = 0;
	}
	
	w_pointer = ringbuf->w_pointer;

	memcpy (&(ringbuf->buf[w_pointer]), buf, n1);
	w_pointer += n1;
	w_pointer &= ringbuf->size_mask;

	if (n2) 
	{
		memcpy (&(ringbuf->buf[w_pointer]), buf + n1, n2);
		w_pointer += n2;
		w_pointer &= ringbuf->size_mask;
	}
	
	ringbuf->w_pointer = w_pointer;

	return to_write;
}
