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


cx_uint16_t reverse4 (cx_uint16_t input)
{
    cx_uint16_t result 	= 0u;
    cx_uint16_t mask 	= 1u;
    int i;

    for (i= 0; i < 4; i++)
    {
        result = result + result;
        if (0u != (input & mask))
        {
            result += 1u;
        }
        mask = mask + mask;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void config_update_gpio_input (void)
{
	//--------------------------------------------------------------------------	
	cx_uint16_t temp_ReadID1 = 0u;
	cx_uint16_t temp_ReadID2 = 0u;
	
	cx_uint16_t rotary_sw_1 = 0u;
	cx_uint16_t rotary_sw_2 = 0u;
	
	//-----------------------------------------------------------------------
	temp_ReadID1 = GPIO_ReadInputData(GPIOE);
	temp_ReadID1 = ~(temp_ReadID1) & 0x3C;
	rotary_sw_1 = (temp_ReadID1 >> 2)&0x0F;  //SW 1 read   

	temp_ReadID2 = GPIO_ReadInputData(GPIOC);
	temp_ReadID2 = ~(temp_ReadID2) & 0x000F;		
	rotary_sw_2 = (temp_ReadID2 >> 0)&0x0F;	//SW 2 read
		
	//-----------------------------------------------------------------------	
	rotary_sw_1 = reverse4(rotary_sw_1);	//reverse
	rotary_sw_2 = reverse4(rotary_sw_2);	//reverse
	
	_config.gpio_i_worker 	= 1u;
	_config.gpio_i_id = ((rotary_sw_1<<4)&0xF0 | (rotary_sw_2 & 0x0F));
	
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void config_show (void)
{
	debug_printf("# CONFIGURATION \n");
	
	config_update_gpio_input();

	debug_printf("\tGPIO INPUT: RE HEXA SW  = %08xh \n"  , _config.gpio_i_id);
	
	debug_printf("\ttype               	= %d:%s \n" , _config.type, equipment_get_type_string(_config.type));
	debug_printf("\tunit              	= %d \n"    , _config.unit     );
	debug_printf("\tworker             	= %d \n"    , _config.worker   );

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

    _config.unit 	= _config.gpio_i_id;
	_config.worker 	= _config.gpio_i_worker;
	
	_config.type 	= DEFINE_RECEIVER; 
	
	//-----------------------------------------------------------------------	
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
		if (_config.unit>0xFFU)
		{
			return CX_FALSE;
		}
		if (_config.gpio_i_id>0xFFu)
		{
			return CX_FALSE;
		}
		if (_config.worker>1u)
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
		if (_config.worker>1u)
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




