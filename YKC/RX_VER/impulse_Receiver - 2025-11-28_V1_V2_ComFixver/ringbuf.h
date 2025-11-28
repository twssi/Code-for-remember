/////////////////////////////////////////////////////////////////////////////
//
// File: ringbuf.h
//
// Created by MOON, Eui-kwon.
// Created on Jan-15th, 2010.
//
/////////////////////////////////////////////////////////////////////////////



#pragma once



/////////////////////////////////////////////////////////////////////////////
//
// Type declarations
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef unsigned int RINGBUF_SIZE;



/////////////////////////////////////////////////////////////////////////////
//
// Structures
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _RINGBUF
{
	RINGBUF_SIZE size     ;
	RINGBUF_SIZE size_mask;
	volatile RINGBUF_SIZE w_pointer;
	volatile RINGBUF_SIZE r_pointer;
	unsigned char*        buf      ;
} RINGBUF;

#if defined ( __cplusplus )

extern "C" void ringbuf_init  (RINGBUF* ringbuf, unsigned char* memory, RINGBUF_SIZE memory_size);
extern "C" void ringbuf_reset (RINGBUF* ringbuf);

extern "C" RINGBUF_SIZE ringbuf_get_readable_space (RINGBUF* ringbuf);
extern "C" RINGBUF_SIZE ringbuf_get_writable_space (RINGBUF* ringbuf);

extern "C" RINGBUF_SIZE ringbuf_peek  (RINGBUF* ringbuf, unsigned char*       buf, RINGBUF_SIZE size);
extern "C" RINGBUF_SIZE ringbuf_read  (RINGBUF* ringbuf, unsigned char*       buf, RINGBUF_SIZE size);
extern "C" RINGBUF_SIZE ringbuf_write (RINGBUF* ringbuf, const unsigned char* buf, RINGBUF_SIZE size);

#else

extern void ringbuf_init  (RINGBUF* ringbuf, unsigned char* memory, RINGBUF_SIZE memory_size);
extern void ringbuf_reset (RINGBUF* ringbuf);

extern RINGBUF_SIZE ringbuf_get_readable_space (RINGBUF* ringbuf);
extern RINGBUF_SIZE ringbuf_get_writable_space (RINGBUF* ringbuf);

extern RINGBUF_SIZE ringbuf_peek  (RINGBUF* ringbuf, unsigned char*       buf, RINGBUF_SIZE size);
extern RINGBUF_SIZE ringbuf_read  (RINGBUF* ringbuf, unsigned char*       buf, RINGBUF_SIZE size);
extern RINGBUF_SIZE ringbuf_write (RINGBUF* ringbuf, const unsigned char* buf, RINGBUF_SIZE size);

#endif
