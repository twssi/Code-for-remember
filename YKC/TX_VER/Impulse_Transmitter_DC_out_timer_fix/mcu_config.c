/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "stm32f10x_bit_define.h"

#include "mcu.h"
#include "mcu_gpio_alias.h"


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
config_t _config;


/////////////////////////////////////////////////////////////////////////////
//===========================================================================

static void config_update_gpio_input (void)
{
	//--------------------------------------------------------------------------	
	_config.gpio_i_worker = GPIO_I_LOCATION_INPUT();

	_config.gpio_i_id 	= 1u;

}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void config_show (void)
{
	debug_printf("# CONFIGURATION \n");
	
	config_update_gpio_input();

	debug_printf("\tGPIO INPUT: TX LOCATION INPUT   = %d \n"     , _config.gpio_i_worker);
	debug_printf("\tGPIO INPUT: RX HEXA SW          = %08xh \n"  , _config.gpio_i_id);
	
	debug_printf("\ttype              	     	= %d:%s \n" , _config.type, equipment_get_type_string(_config.type));
	debug_printf("\tunit              	     	= %d \n"    , _config.unit     );
	debug_printf("\tworker            	        = %d \n"    , _config.worker   );

	//===========================================================================
	debug_flush();
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t config_initialize (void)
{
	//-----------------------------------------------------------------------
	memset (&_config, 0, sizeof(_config));
	
	config_update_gpio_input();

	_config.unit	= _config.gpio_i_id;
	_config.worker 	= _config.gpio_i_worker+1;
	
	_config.type 	= DEFINE_TRANSMITTER_SUPERVISOR;
	

	if (_config.unit==0u)
	{
		return CX_FALSE;
	}

	switch(_config.type)
	{
	case EQUIPMENT_TYPE_TRANSMITTER:
		if (_config.unit>EQUIPMENT_TRANSMITTER_MAX_COUNT)
		{
			return CX_FALSE;
		}
		if (_config.unit>1u)
		{
			return CX_FALSE;
		}
		if (_config.worker>2u)
		{
			return CX_FALSE;
		}
		
		break;
		
	case EQUIPMENT_TYPE_RECEIVER:
		if (_config.unit>EQUIPMENT_RECEIVER_MAX_COUNT)
		{
			return CX_FALSE;
		}
		if (_config.unit>1u)
		{
			return CX_FALSE;
		}
		if (_config.worker>2u)
		{
			return CX_FALSE;
		}
		
		break;

	case DEFINE_DATA_GATHERING_BOARD:
		if (_config.unit>EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT)
		{
			return CX_FALSE;
		}
		if (_config.unit>1u)
		{
			return CX_FALSE;
		}
		if (_config.worker>2u)
		{
			return CX_FALSE;
		}
		
		break;
		
		
	default:
		return CX_FALSE;
		break;
	}
	
	//-----------------------------------------------------------------------

	return CX_TRUE;
}




