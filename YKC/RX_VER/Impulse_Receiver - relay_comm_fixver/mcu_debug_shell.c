/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "mcu.h"





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define DEBUG_SHELL_RX_MAX_SIZE            64
#define DEBUG_SHELL_RX_BSB_BUFFER_MAX_SIZE (DEBUG_SHELL_RX_MAX_SIZE*2u)

//===========================================================================
#define DEBUG_SHELL_COMMAND_NO_PARAM

#if defined(DEBUG_SHELL_COMMAND_NO_PARAM)
#define DEBUG_SHELL_COMMAND_FUNCTION_ARG void
#else
#define DEBUG_SHELL_COMMAND_FUNCTION_ARG cx_char_t* name, cx_char_t* command_string, cx_uint_t command_string_size
#endif





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef void (*debug_shell_command_function_t)(DEBUG_SHELL_COMMAND_FUNCTION_ARG);

typedef struct _debug_shell_command_t
{
	cx_char_t*                     name;
	debug_shell_command_function_t function;

}
debug_shell_command_t;





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static bsb_t     _debug_shell_rx_bsb;
static cx_byte_t _debug_shell_rx_bsb_buffer[DEBUG_SHELL_RX_BSB_BUFFER_MAX_SIZE];



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void debug_shell_command_00(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_01(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_02(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_03(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_04(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_05(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_06(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_07(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_08(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
static void debug_shell_command_09(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
#if 0
static void debug_shell_command_10(DEBUG_SHELL_COMMAND_FUNCTION_ARG);
#endif

//===========================================================================
static debug_shell_command_t _debug_shell_command_collection[]=
{
	{"?"         , debug_shell_command_00 },
	{"help"      , debug_shell_command_00 },
	{"version"   , debug_shell_command_01 },
	{"memory"    , debug_shell_command_02 },
	{"config"    , debug_shell_command_03 },
	{"peer"      , debug_shell_command_04 },
	{"equipment" , debug_shell_command_05 },
	{"message" 	 , debug_shell_command_06 },
	{"frequency" , debug_shell_command_07 },
	{"analog" 	 , debug_shell_command_08 },
	{"input" 	 , debug_shell_command_09 }
#if 0		
	{"out"		 , debug_shell_command_10 }
#endif	
};

//===========================================================================
void debug_shell_command_00(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	/*
	cx_char_t* param_string_pointer;
	cx_uint_t  param_string_size;


	param_string_pointer = command_string     +strlen(name);
	param_string_size    = command_string_size-strlen(name);
	*/

	debug_shell_show_help();
}

void debug_shell_command_01(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	application_show_version();
}

void debug_shell_command_02(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	application_show_memory();
	debug_print_compiling_information();
}

void debug_shell_command_03(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	config_show();
}

void debug_shell_command_04(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	peer_show();
}

void debug_shell_command_05(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	equipment_show();
}

void debug_shell_command_06(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	message_show();
}

void debug_shell_command_07(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	input_frequency_show();
}

void debug_shell_command_08(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	measured_analog_data_show();
}

void debug_shell_command_09(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
	io_control_show();
}
#if 0
void debug_shell_command_10(DEBUG_SHELL_COMMAND_FUNCTION_ARG)
{
//	reset_cause_show();
}

#endif
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void debug_shell_show_prompt (void)
{
	debug_printf(">");
}

void debug_shell_show_help (void)
{
	debug_printf("# HELP\n");


	debug_shell_command_t* command;

	cx_uint_t i;
	cx_uint_t count;


	count = sizeof(_debug_shell_command_collection)/sizeof(debug_shell_command_t);
	for (i=0u; i<count; i++)
	{
		command = &_debug_shell_command_collection[i];

		debug_printf("\t%s\n", command->name);
	}
}

void debug_shell_command (cx_char_t* command_string, cx_uint_t command_string_size)
{
	if (0u==command_string_size)
	{
		debug_shell_show_prompt();
		return;
	}


	debug_shell_command_t* command;

	cx_uint_t i;
	cx_uint_t count;


	count = sizeof(_debug_shell_command_collection)/sizeof(debug_shell_command_t);
	for (i=0u; i<count; i++)
	{
		command = &_debug_shell_command_collection[i];
		

		if (0==strncmp(command->name, command_string, strlen(command->name)))
		{
#if defined(DEBUG_SHELL_COMMAND_NO_PARAM)
			command->function();
#else
			command->function(command->name, command_string, command_string_size);
#endif
			debug_shell_show_prompt();
			return;
		}
	}


	debug_printf("Invaild command !\n");
	debug_shell_show_prompt();
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t debug_shell_initialize(void)
{
	cx_byte_t* rx_bsb_buffer_pointer;
	cx_uint_t  rx_bsb_buffer_size;


	rx_bsb_buffer_size    = sizeof(_debug_shell_rx_bsb_buffer);
	rx_bsb_buffer_pointer = _debug_shell_rx_bsb_buffer;


	if (0==bsb_initialize (&_debug_shell_rx_bsb, rx_bsb_buffer_pointer, rx_bsb_buffer_size))
	{
		return CX_FALSE;
	}


	return CX_TRUE;
}

void debug_shell (void)
{
	cx_char_t* stream_pointer;
	cx_uint_t  stream_size;

	cx_uint_t i;
	cx_uint_t count;
	cx_char_t ch;


	cx_byte_t rx[DEBUG_SHELL_RX_MAX_SIZE];
	cx_uint_t rx_size;


	rx_size = com_recv_buffer(COM1, rx, sizeof(rx));
	if (rx_size > 0u)
	{
		count = rx_size;
		for (i=0u; i<count; i++)
		{
			ch = rx[i];

			if     ( '\n'==ch )
			{
				com_send_byte(COM1, ch);

				stream_size    = bsb_get_size(&_debug_shell_rx_bsb);
				stream_pointer = (cx_char_t*)bsb_get_pointer(&_debug_shell_rx_bsb);

				debug_shell_command(stream_pointer, stream_size);

				bsb_erase(&_debug_shell_rx_bsb, stream_size);
			}
			else if ( '\r'==ch )
			{
			}
			else if ( '\t'==ch )
			{
			}
			else if ( '\b'==ch )
			{
				bsb_shrink_byte(&_debug_shell_rx_bsb);
			}
			else if ( ' '<=ch && ch<='~' ) // 0x20~0x7F
			{
				com_send_byte(COM1, ch);
				bsb_push_byte(&_debug_shell_rx_bsb, ch);
			}
		}
	}
}
