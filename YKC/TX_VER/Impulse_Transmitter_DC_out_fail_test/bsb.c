/****************************************************************************
**
** File: bsb.c
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
#include <string.h>
#include "bsb.h"





/****************************************************************************
**
** Global functions
**
****************************************************************************/
/*=========================================================================*/
bsb_t* bsb_initialize (bsb_t* bsb, bsb_byte_t* pointer, bsb_size_t max_size)
{
	bsb->pointer  = pointer;
	bsb->max_size = max_size;
	bsb->size     = 0u;

	// The max_size must be greater than zero.

	if (0u==bsb->max_size)
	{
		return 0; // error
	}

	return bsb;
}

bsb_size_t bsb_get_max_size (bsb_t* bsb)
{
	return bsb->max_size;
}

bsb_size_t bsb_get_size (bsb_t* bsb)
{
	return bsb->size;
}

bsb_byte_t* bsb_get_pointer (bsb_t* bsb)
{
	return bsb->pointer;
}

bsb_size_t bsb_get_space(bsb_t* bsb)
{
	bsb_size_t space;


	space = bsb->max_size - bsb->size;

	return space;
}

void bsb_clear (bsb_t* bsb)
{
	bsb->size = 0u;
}

void bsb_erase(bsb_t* bsb, bsb_size_t size)
{
	if (bsb->size <= size)
	{
		bsb_clear(bsb);
		return;
	}

	
	bsb_size_t  msize;
	bsb_size_t  moffset;
	bsb_byte_t* dpointer;
	bsb_byte_t* spointer;


	msize    = bsb->size - size;
	moffset  = bsb->size - msize;
	dpointer = bsb->pointer;
	spointer = bsb->pointer+moffset;
	memmove(dpointer, spointer, msize); // The memory areas may overlap.

	
	bsb->size -= size;
}

void bsb_erase_byte(bsb_t* bsb)
{
	bsb_erase(bsb, 1u);
}

void bsb_push (bsb_t* bsb, bsb_byte_t* pointer, bsb_size_t size)
{
#if 0
	if (0u==bsb->max_size)
	{
		return;
	}
#endif

#if 1
	if (1u==size)
	{
		bsb_push_byte(bsb, *pointer);
		return;
	}
#endif


	bsb_byte_t* dpointer;
	bsb_byte_t* spointer;
	
	bsb_size_t psize;
	bsb_size_t ssize;
	bsb_size_t esize;


	if (bsb->max_size < size)
	{
		psize    = bsb->max_size;
		spointer = pointer+(size - bsb->max_size);
	}
	else
	{
		psize    = size;
		spointer = pointer;
	}
	
	
	ssize = bsb_get_space(bsb);

	if ( ssize < psize )
	{
		esize = psize - ssize;
		bsb_erase (bsb, esize);
	}


	dpointer = bsb->pointer + bsb->size;
	memcpy(dpointer, spointer, psize); // The memory areas must not overlap.


	bsb->size = bsb->size + psize;
}

void bsb_push_byte (bsb_t* bsb, bsb_byte_t ch)
{
#if 0
	if (0u==bsb->max_size)
	{
		return;
	}
#endif

#if 0

	bsb_push(bsb, &ch, 1u);

#else

	if (bsb->size == bsb->max_size)
	{
		bsb_erase_byte(bsb);
	}

	
	bsb_byte_t* dpointer;
	

	dpointer = bsb->pointer + bsb->size;

	*dpointer = ch;
	
	
	bsb->size++;

#endif
}

void bsb_shrink (bsb_t* bsb, bsb_size_t size)
{
	if (bsb->size > size)
	{
		bsb->size = size;
	}
}

void bsb_shrink_byte (bsb_t* bsb)
{
	if (bsb->size > 0u)
	{
		bsb->size--;
	}
}




