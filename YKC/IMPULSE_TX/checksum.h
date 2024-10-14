#ifndef INCLUDED__CX__CHECKSUM__H
#define INCLUDED__CX__CHECKSUM__H

/****************************************************************************
**
** File: checksum.h
**
** Created by MOON, Eui-kwon.
** Created on Jan-28th, 2010.
**
****************************************************************************/





/****************************************************************************
**
** Define
**
****************************************************************************/
/*=========================================================================*/
#if defined ( __cplusplus )
#	define CX_API_CHECKSUM extern "C"
#else
#	define CX_API_CHECKSUM extern
#endif





/****************************************************************************
**
** Type declarations
**
****************************************************************************/
/*=========================================================================*/
typedef unsigned long int  checksum_size_t;
typedef unsigned char      checksum_u8_t  ;
typedef unsigned short int checksum_u16_t ;
typedef unsigned       int checksum_u32_t ;





/****************************************************************************
**
** Global function prototypes
**
****************************************************************************/
/*=========================================================================*/
CX_API_CHECKSUM checksum_u8_t  calc_sum8  (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u8_t  calc_bcc8  (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u16_t calc_crc16 (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u32_t calc_crc32 (const void* buf, checksum_size_t len);

CX_API_CHECKSUM checksum_u16_t calc_crc16_ccitt_std (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u16_t calc_crc16_ccitt_n   (checksum_u16_t initial_remainder, checksum_u16_t final_xor_value, const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u16_t calc_crc16_ccitt_r   (checksum_u16_t initial_remainder, checksum_u16_t final_xor_value, const void* buf, checksum_size_t len);

CX_API_CHECKSUM checksum_u16_t calc_crc16_ibm_std (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u16_t calc_crc16_ibm_n   (checksum_u16_t initial_remainder, checksum_u16_t final_xor_value, const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u16_t calc_crc16_ibm_r   (checksum_u16_t initial_remainder, checksum_u16_t final_xor_value, const void* buf, checksum_size_t len);

CX_API_CHECKSUM checksum_u32_t calc_crc32_std (const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u32_t calc_crc32_n   (checksum_u32_t initial_remainder, checksum_u32_t final_xor_value, const void* buf, checksum_size_t len);
CX_API_CHECKSUM checksum_u32_t calc_crc32_r   (checksum_u32_t initial_remainder, checksum_u32_t final_xor_value, const void* buf, checksum_size_t len);





#endif


