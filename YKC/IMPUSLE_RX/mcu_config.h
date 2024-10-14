#ifndef INCLUDED__MCU_CONFIG__H
#define INCLUDED__MCU_CONFIG__H


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define DEFINE_TRANSMITTER_SUPERVISOR	1
#define DEFINE_RECEIVER  				2
#define DEFINE_DATA_GATHERING_BOARD  	3
#define DEFINE_TRANSMITTER_CONTROL		4


typedef struct _config_t
{
	cx_uint_t 		gpio_i_worker         ; // 송신모듈 1계 or 2계 입력값
	cx_uint_t		gpio_i_id             ; // 수신모듈 hexa sw 입력값

    cx_uint_t 		type     		;
	cx_uint_t 		unit     		;
	cx_uint_t 		worker   		;
}
config_t;


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API config_t _config;

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_bool_t config_initialize (void);
MCU_API cx_uint16_t reverse4 (cx_uint16_t input);

MCU_API void config_show (void);

#endif




