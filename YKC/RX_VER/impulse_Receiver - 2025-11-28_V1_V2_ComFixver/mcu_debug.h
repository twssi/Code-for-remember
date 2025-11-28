#ifndef INCLUDED__MCU_DEBUG__H
#define INCLUDED__MCU_DEBUG__H





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#ifndef MCU_SIMULATOR

MCU_API void debug_printf (const cx_char_t* format, ...);


#else

#include <stdio.h>

#define debug_printf printf 

#endif

MCU_API void debug_print_compiling_information (void);
MCU_API void debug_flush(void);





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_bool_t debug_shell_initialize  (void);
MCU_API void      debug_shell             (void);
MCU_API void      debug_shell_show_prompt (void);
MCU_API void      debug_shell_show_help   (void);
MCU_API void      debug_shell_command     (cx_char_t* command_string, cx_uint_t command_string_size);





#endif
