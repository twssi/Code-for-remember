/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "mcu.h"





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define DEBUG_MESSAGE_BUFFER_SIZE 256





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#ifndef MCU_SIMULATOR

void debug_flush (void)
{
	com_flush(COM3);
}

void debug_printf (const cx_char_t* format, ...)
{
	cx_char_t buffer[DEBUG_MESSAGE_BUFFER_SIZE] = { 0 };
	cx_int_t  max_count;
	cx_int_t  length;
	va_list   args;


	max_count = sizeof(buffer)-1;

	va_start (args, format);
	length = vsnprintf (buffer, max_count, format, args);
	va_end (args);
	

	if (length>0)
	{
		buffer[length] = '\0';

//		printf(buffer);

		com_send_buffer(COM3, (cx_byte_t*)buffer, length);
	}

}

#else

void debug_flush (void)
{
}


#endif





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#if 1
static
void print_type_size (void)
{
	//-----------------------------------------------------------------------
	debug_printf("=============================================================================\r\n");
	debug_printf(" TYPE SIZE \r\n");
	debug_printf("=============================================================================\r\n");



	//-----------------------------------------------------------------------
	debug_printf("sizeof( void*       ) = %d bytes \r\n", sizeof( void*       ) );
	debug_printf("sizeof( char        ) = %d bytes \r\n", sizeof( char        ) );
//	debug_printf("sizeof( wchar_t     ) = %d bytes \r\n", sizeof( wchar_t     ) );
	debug_printf("sizeof( int         ) = %d bytes \r\n", sizeof( int         ) );
	debug_printf("sizeof( short int   ) = %d bytes \r\n", sizeof( short int   ) );
	debug_printf("sizeof( long int    ) = %d bytes \r\n", sizeof( long int    ) );
	debug_printf("sizeof( float       ) = %d bytes \r\n", sizeof( float       ) );
	debug_printf("sizeof( double      ) = %d bytes \r\n", sizeof( double      ) );
	debug_printf("sizeof( long double ) = %d bytes \r\n", sizeof( long double ) );
	debug_printf("sizeof( size_t      ) = %d bytes \r\n", sizeof( size_t      ) );
//	debug_printf("sizeof( time_t      ) = %d bytes \r\n", sizeof( time_t      ) );
	debug_printf("\r\n");

	//-----------------------------------------------------------------------
	debug_printf("sizeof( cx_pointer_t ) = %d bytes \r\n", sizeof( cx_pointer_t ) );
	debug_printf("\r\n");

	debug_printf("sizeof( cx_int_t    ) = %d bytes \r\n", sizeof( cx_int_t    ) );
	debug_printf("sizeof( cx_uint_t   ) = %d bytes \r\n", sizeof( cx_uint_t   ) );
	debug_printf("sizeof( cx_int8_t   ) = %d bytes \r\n", sizeof( cx_int8_t   ) );
	debug_printf("sizeof( cx_int16_t  ) = %d bytes \r\n", sizeof( cx_int16_t  ) );
	debug_printf("sizeof( cx_int32_t  ) = %d bytes \r\n", sizeof( cx_int32_t  ) );
	debug_printf("sizeof( cx_int64_t  ) = %d bytes \r\n", sizeof( cx_int64_t  ) );
	debug_printf("sizeof( cx_uint8_t  ) = %d bytes \r\n", sizeof( cx_uint8_t  ) );
	debug_printf("sizeof( cx_uint16_t ) = %d bytes \r\n", sizeof( cx_uint16_t ) );
	debug_printf("sizeof( cx_uint32_t ) = %d bytes \r\n", sizeof( cx_uint32_t ) );
	debug_printf("sizeof( cx_uint64_t ) = %d bytes \r\n", sizeof( cx_uint64_t ) );
	debug_printf("\r\n");

	debug_printf("sizeof( cx_float_t     ) = %d bytes \r\n", sizeof( cx_float_t     ) );
	debug_printf("sizeof( cx_double_t    ) = %d bytes \r\n", sizeof( cx_double_t    ) );
	debug_printf("sizeof( cx_float32_t   ) = %d bytes \r\n", sizeof( cx_float32_t   ) );
	debug_printf("sizeof( cx_double64_t  ) = %d bytes \r\n", sizeof( cx_double64_t  ) );
	debug_printf("\r\n");

	debug_printf("sizeof( cx_char_t    ) = %d bytes \r\n", sizeof( cx_char_t    ) );
	debug_printf("sizeof( cx_uchar8_t  ) = %d bytes \r\n", sizeof( cx_uchar8_t  ) );
	debug_printf("sizeof( cx_uchar16_t ) = %d bytes \r\n", sizeof( cx_uchar16_t ) );
	debug_printf("sizeof( cx_uchar32_t ) = %d bytes \r\n", sizeof( cx_uchar32_t ) );
	debug_printf("\r\n");

	debug_printf("sizeof( cx_bool_t ) = %d bytes \r\n", sizeof( cx_bool_t ) );
	debug_printf("sizeof( cx_byte_t ) = %d bytes \r\n", sizeof( cx_byte_t ) );
	debug_printf("sizeof( cx_size_t ) = %d bytes \r\n", sizeof( cx_size_t ) );
	debug_printf("sizeof( cx_time_t ) = %d bytes \r\n", sizeof( cx_time_t ) );
	debug_printf("\r\n");



	//-----------------------------------------------------------------------
	typedef struct _my_test_struct_t
	{
		cx_uint32_t i32;
//		cx_uint16_t i16;
		cx_char_t   ch;
	} my_test_struct_t;

	debug_printf("typedef struct _my_test_struct_t \r\n");
	debug_printf("{                                \r\n");
	debug_printf("    cx_uint32_t i32;             \r\n");
//	debug_printf("    cx_uint16_t i16;             \r\n");
	debug_printf("    cx_char_t   ch;              \r\n");
	debug_printf("} my_test_struct_t;              \r\n");

	debug_printf("sizeof( my_test_struct_t ) = %d bytes \r\n", sizeof( my_test_struct_t ) );
	debug_printf("\r\n");



	//-----------------------------------------------------------------------
	debug_printf("\r\n");
}

static
void print_endianness (void)
{
	cx_uint32_t word = 0x12345678;
	
	cx_byte_t   byteorder[4];

	cx_byte_t   be[4] = { 0x12,0x34,0x56,0x78 };
	cx_byte_t   le[4] = { 0x78,0x56,0x34,0x12 };


	memcpy (byteorder, &word, sizeof(word));


	debug_printf("=============================================================================\r\n");
	debug_printf(" ENDIANNESS \r\n");
	debug_printf("=============================================================================\r\n");

	debug_printf ("0x%08x = [0:0x%02x] [1:0x%02x] [2:0x%02x] [3:0x%02x] \r\n",
		word,
		byteorder[0],
		byteorder[1],
		byteorder[2],
		byteorder[3]
		);

	if      (0==memcmp (byteorder, be, sizeof(byteorder))) 
	{
		debug_printf( "BIG ENDIANNESS SYSTEM \r\n" );
	}
	else if (0==memcmp (byteorder, le, sizeof(byteorder))) 
	{
		debug_printf( "LITTLE ENDIANNESS SYSTEM \r\n" );
	}
	else
	{
		debug_printf( "UNKNOWN ENDIANNESS SYSTEM \r\n" );
	}

	debug_printf("\r\n");
}

static
void print_bitfield_order (void)
{
	//===========================================================================
	typedef unsigned bitfield_t;

	//===========================================================================
	typedef union _uint8_bit_des_t
	{
		struct _8d_bit_t
		{
			bitfield_t b7:1; 
			bitfield_t b6:1;
			bitfield_t b5:1;
			bitfield_t b4:1;
			bitfield_t b3:1;
			bitfield_t b2:1;
			bitfield_t b1:1;
			bitfield_t b0:1;
		} bit;
		cx_uint8_t value;
	} uint8_bit_des_t;

	typedef union _uint16_bit_des_t
	{
		struct _16d_bit_t
		{
			bitfield_t b15:1;
			bitfield_t b14:1;
			bitfield_t b13:1;
			bitfield_t b12:1;
			bitfield_t b11:1;
			bitfield_t b10:1;
			bitfield_t b9:1;
			bitfield_t b8:1;
			bitfield_t b7:1;
			bitfield_t b6:1;
			bitfield_t b5:1;
			bitfield_t b4:1;
			bitfield_t b3:1;
			bitfield_t b2:1;
			bitfield_t b1:1;
			bitfield_t b0:1;
		} bit;
		cx_uint16_t value;
	} uint16_bit_des_t;

	typedef union _uint32_bit_des_t
	{
		struct _32d_bit_t
		{
			bitfield_t b31:1;
			bitfield_t b30:1;
			bitfield_t b29:1;
			bitfield_t b28:1;
			bitfield_t b27:1;
			bitfield_t b26:1;
			bitfield_t b25:1;
			bitfield_t b24:1;
			bitfield_t b23:1;
			bitfield_t b22:1;
			bitfield_t b21:1;
			bitfield_t b20:1;
			bitfield_t b19:1;
			bitfield_t b18:1;
			bitfield_t b17:1;
			bitfield_t b16:1;
			bitfield_t b15:1;
			bitfield_t b14:1;
			bitfield_t b13:1;
			bitfield_t b12:1;
			bitfield_t b11:1;
			bitfield_t b10:1;
			bitfield_t b9:1;
			bitfield_t b8:1;
			bitfield_t b7:1;
			bitfield_t b6:1;
			bitfield_t b5:1;
			bitfield_t b4:1;
			bitfield_t b3:1;
			bitfield_t b2:1;
			bitfield_t b1:1;
			bitfield_t b0:1;
		} bit;
		cx_uint32_t value;
	} uint32_bit_des_t;

	//===========================================================================
	typedef union _uint8_bit_asc_t
	{
		struct _8a_bit_t
		{
			bitfield_t b0:1;
			bitfield_t b1:1; 
			bitfield_t b2:1;
			bitfield_t b3:1;
			bitfield_t b4:1;
			bitfield_t b5:1;
			bitfield_t b6:1;
			bitfield_t b7:1;
		} bit;
		cx_uint8_t value;
	} uint8_bit_asc_t;

	typedef union _uint16_bit_asc_t
	{
		struct _16a_bit_t
		{
			bitfield_t b0:1;
			bitfield_t b1:1;
			bitfield_t b2:1;
			bitfield_t b3:1;
			bitfield_t b4:1;
			bitfield_t b5:1;
			bitfield_t b6:1;
			bitfield_t b7:1;
			bitfield_t b8:1;
			bitfield_t b9:1;
			bitfield_t b10:1;
			bitfield_t b11:1;
			bitfield_t b12:1;
			bitfield_t b13:1;
			bitfield_t b14:1;
			bitfield_t b15:1;
		} bit;
		cx_uint16_t value;
	} uint16_bit_asc_t;

	typedef union _uint32_bit_asc_t
	{
		struct _32a_bit_t
		{
			bitfield_t b0:1;
			bitfield_t b1:1;
			bitfield_t b2:1;
			bitfield_t b3:1;
			bitfield_t b4:1;
			bitfield_t b5:1;
			bitfield_t b6:1;
			bitfield_t b7:1;
			bitfield_t b8:1;
			bitfield_t b9:1;
			bitfield_t b10:1;
			bitfield_t b11:1;
			bitfield_t b12:1;
			bitfield_t b13:1;
			bitfield_t b14:1;
			bitfield_t b15:1;
			bitfield_t b16:1;
			bitfield_t b17:1;
			bitfield_t b18:1;
			bitfield_t b19:1;
			bitfield_t b20:1;
			bitfield_t b21:1;
			bitfield_t b22:1;
			bitfield_t b23:1;
			bitfield_t b24:1;
			bitfield_t b25:1;
			bitfield_t b26:1;
			bitfield_t b27:1;
			bitfield_t b28:1;
			bitfield_t b29:1;
			bitfield_t b30:1;
			bitfield_t b31:1;
		} bit;
		cx_uint32_t value;
	} uint32_bit_asc_t;

	//===========================================================================
	//--------------------------------------------------------------------------
	debug_printf("=============================================================================\r\n");
	debug_printf(" BITFIELD \r\n");
	debug_printf("=============================================================================\r\n");

	uint32_bit_des_t u32bit;


	u32bit.value   = 0u;
	u32bit.bit.b31 = 1u;

	if      ( u32bit.value == 0x80000000u )
	{
		debug_printf( "BITFIELD descending order \r\n" );

		// big endian system
		// vxworks, vxworks653
	}
	else if ( u32bit.value == 0x00000001u )
	{
		debug_printf( "BITFIELD ascending order \r\n" );

		// little endian system
		// windows
	}
	else
	{
		debug_printf( "BITFIELD unknown order \r\n" );
	}

	debug_printf("uint32_bit_des_t.bit.b31==1, uint32_bit_des_t.value = 0x%08x  \r\n", u32bit.value);


	//--------------------------------------------------------------------------
	uint32_bit_des_t u32d;
	uint32_bit_asc_t u32a;
	uint16_bit_des_t u16d;
	uint16_bit_asc_t u16a;
	uint8_bit_des_t  u8d ;
	uint8_bit_asc_t  u8a ;


	u32d.value = 0u;
	u32a.value = 0u;
	u16d.value = 0u;
	u16a.value = 0u;
	u8d .value = 0u;
	u8a .value = 0u;

	u32d.bit.b0 = 1u;
	u32a.bit.b0 = 1u;
	u16d.bit.b0 = 1u;
	u16a.bit.b0 = 1u;
	u8d .bit.b0 = 1u;
	u8a .bit.b0 = 1u;

	debug_printf("uint32_bit_des_t b0 is 0x%x \r\n", u32d.value);
	debug_printf("uint32_bit_asc_t b0 is 0x%x \r\n", u32a.value);
	debug_printf("uint16_bit_des_t b0 is 0x%x \r\n", u16d.value);
	debug_printf("uint16_bit_asc_t b0 is 0x%x \r\n", u16a.value);
	debug_printf("uint8_bit_des_t  b0 is 0x%x \r\n", u8d .value);
	debug_printf("uint8_bit_asc_t  b0 is 0x%x \r\n", u8a .value);


	//--------------------------------------------------------------------------
	cx_uint16_t      v;
	uint16_bit_asc_t x;

	v         = 6u;
	x.value   = 0u;
	x.bit.b10 = v;
	if (x.value == 0x0000u)
	{
		debug_printf ("bitfield test1 : OK\n");
	}
	else
	{
		debug_printf ("bitfield test1 : fail\n");
	}

	v         = 5;
	x.value   = 0u;
	x.bit.b10 = v;
	if (x.value == 0x0400u)
	{
		debug_printf ("bitfield test2 : OK\n");
	}
	else
	{
		debug_printf ("bitfield test2 : fail\n");
	}

	v = 0u;
	v = (cx_uint16_t)x.bit.b10;
	if (v == 1u)
	{
		debug_printf ("bitfield test3 : OK\n");
	}
	else
	{
		debug_printf ("bitfield test3 : fail\n");
	}
}

void debug_print_compiling_information (void)
{
	print_type_size();
	print_endianness();
	print_bitfield_order();
}

#else

void debug_print_compiling_information (void)
{
}

#endif




