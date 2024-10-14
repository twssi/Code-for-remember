#ifndef INCLUDED__CX__BUFFER__BSB__H
#define INCLUDED__CX__BUFFER__BSB__H

/****************************************************************************
**
** File: bsb.h
**
** Created by MOON, Eui-kwon.
** Created on Jan-15th, 2010.
**
**
** - byte stream buffer
**
****************************************************************************/





/****************************************************************************
**
** Define
**
****************************************************************************/
/*=========================================================================*/
#if defined ( __cplusplus )
#	define CX_API_BSB extern "C"
#else
#	define CX_API_BSB extern
#endif




/****************************************************************************
**
** Type declarations
**
****************************************************************************/
/*=========================================================================*/
typedef unsigned int  bsb_size_t;
typedef unsigned char bsb_byte_t;



/****************************************************************************
**
** Structures
**
****************************************************************************/
/*=========================================================================*/
typedef struct _bsb_t
{
	bsb_byte_t* pointer;
	bsb_size_t  max_size;
	bsb_size_t  size;
}
bsb_t;



/****************************************************************************
**
** Global functions prototypes
**
****************************************************************************/
/*=========================================================================*/
CX_API_BSB bsb_t*      bsb_initialize   (bsb_t* bsb, bsb_byte_t* pointer, bsb_size_t max_size);
CX_API_BSB bsb_size_t  bsb_get_max_size (bsb_t* bsb);
CX_API_BSB bsb_size_t  bsb_get_size     (bsb_t* bsb);
CX_API_BSB bsb_byte_t* bsb_get_pointer  (bsb_t* bsb);
CX_API_BSB bsb_size_t  bsb_get_space    (bsb_t* bsb);
CX_API_BSB void        bsb_clear        (bsb_t* bsb);
CX_API_BSB void        bsb_erase        (bsb_t* bsb, bsb_size_t size);
CX_API_BSB void        bsb_erase_byte   (bsb_t* bsb);
CX_API_BSB void        bsb_push         (bsb_t* bsb, bsb_byte_t* pointer, bsb_size_t size);
CX_API_BSB void        bsb_push_byte    (bsb_t* bsb, bsb_byte_t ch);
CX_API_BSB void        bsb_shrink       (bsb_t* bsb, bsb_size_t size);
CX_API_BSB void        bsb_shrink_byte  (bsb_t* bsb);






#endif




