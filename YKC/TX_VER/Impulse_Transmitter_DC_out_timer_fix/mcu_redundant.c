/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "stm32f10x_init.h"
#include "stm32f10x_bit_define.h"

#include "mcu.h"
#include "mcu_gpio_alias.h"


#define RUN_STOP_DELAY_COUNT 10u

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static cx_uint_t _worker_number   = 0u;
static cx_uint_t _worker_state    = 0u;
static cx_uint_t _coworker_number = 0u;
static cx_uint_t _coworker_state  = 0u;

static cx_uint_t _worker_fault    = 0u;
static cx_uint_t _worker_switch   = 0u;

static cx_uint_t _pair_dido_heartbeat_fault = 0u;	

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static cx_char_t* get_worker_state_string (cx_uint_t state)
{
	cx_char_t* state_string;


	switch (state)
	{
	case WORKER_STATE_UNKNOWN     		: state_string="UNKNOWN"     	; break;
	case WORKER_STATE_BOOT        		: state_string="BOOT"        	; break;
	case WORKER_STATE_ACTIVE_SWITCH  	: state_string="ACTIVE_SWITCH"  ; break;
	case WORKER_STATE_ACTIVE_NONE 		: state_string="ACTIVE_NONE" 	; break;
	case WORKER_STATE_STANDBY_SWITCH 	: state_string="STANDBY_SWITCH" ; break;
	case WORKER_STATE_STANDBY_NONE		: state_string="STANDBY_NONE"	; break;
	case WORKER_STATE_HALT        		: state_string="HALT"        	; break;

	default:
		state_string = "UNKNOWN?";
		break;
	}


	return state_string;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_uint_t get_worker_number (void)
{
	return _worker_number;
}

cx_uint_t get_coworker_number (void)
{
	return _coworker_number;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t check_worker_active (cx_uint_t state)
{
	switch (state)
	{
	case WORKER_STATE_ACTIVE_SWITCH  :
	case WORKER_STATE_ACTIVE_NONE :
		return CX_TRUE;
		break;

	default:
		break;
	}


	return CX_FALSE;
}

cx_bool_t check_worker_health (cx_uint_t state)
{
	switch (state)
	{
	case WORKER_STATE_BOOT:
	case WORKER_STATE_ACTIVE_SWITCH  :
	case WORKER_STATE_ACTIVE_NONE :
	case WORKER_STATE_STANDBY_SWITCH :
	case WORKER_STATE_STANDBY_NONE:
		return CX_TRUE;
		break;

	default:
		break;
	}


	return CX_FALSE;
}

cx_bool_t check_worker_halt (cx_uint_t state)
{
	switch (state)
	{
	case WORKER_STATE_BOOT:
	case WORKER_STATE_ACTIVE_SWITCH  :
	case WORKER_STATE_ACTIVE_NONE :
	case WORKER_STATE_STANDBY_SWITCH :
	case WORKER_STATE_STANDBY_NONE:
		return CX_FALSE;
		break;

	default:
		break;
	}


	return CX_TRUE;
}

cx_bool_t check_worker_switch (cx_uint_t state)
{
	switch (state)
	{
	case WORKER_STATE_ACTIVE_SWITCH  :
	case WORKER_STATE_STANDBY_SWITCH :
		return CX_TRUE;
		break;

	default:
		break;
	}


	return CX_FALSE;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_uint_t get_worker_state (void)
{
	return _worker_state;
}

cx_bool_t get_worker_active (void)
{
	return check_worker_active(_worker_state);
}

cx_bool_t get_worker_health (void)
{
	return check_worker_health(_worker_state);
}

cx_bool_t get_worker_halt (void)
{
	return check_worker_halt (_worker_state);
}

cx_bool_t get_worker_switch (void)
{
	return check_worker_switch (_worker_state);
}

//===========================================================================
cx_uint_t get_coworker_state (void)
{
	return _coworker_state;
}

cx_bool_t get_coworker_active (void)
{
	return check_worker_active(_coworker_state);
}

cx_bool_t get_coworker_health (void)
{
	return check_worker_health(_coworker_state);
}

cx_bool_t get_coworker_halt (void)
{
	return check_worker_halt (_coworker_state);
}

cx_bool_t get_coworker_switch (void)
{
	return check_worker_switch (_coworker_state);
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_uint_t get_worker1_state (void)
{
	if (1u==get_worker_number()) { return get_worker_state  (); }
	if (2u==get_worker_number()) { return get_coworker_state(); }

	return WORKER_STATE_HALT;
}

cx_uint_t get_worker2_state (void)
{
	if (1u==get_worker_number()) { return get_coworker_state (); }
	if (2u==get_worker_number()) { return get_worker_state   (); }

	return WORKER_STATE_HALT;
}

//===========================================================================
cx_bool_t get_worker1_active (void)
{
	return check_worker_active(get_worker1_state());
}

cx_bool_t get_worker1_health (void)
{
	return check_worker_health(get_worker1_state());
}

cx_bool_t get_worker1_halt(void)
{
	return check_worker_halt(get_worker1_state());
}

cx_bool_t get_worker1_switch(void)
{
	return check_worker_switch(get_worker1_state());
}

//===========================================================================
cx_bool_t get_worker2_active (void)
{
	return check_worker_active(get_worker2_state());
}

cx_bool_t get_worker2_health (void)
{
	return check_worker_health(get_worker2_state());
}

cx_bool_t get_worker2_halt(void)
{
	return check_worker_halt(get_worker2_state());
}

cx_bool_t get_worker2_switch(void)
{
	return check_worker_switch(get_worker2_state());
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void set_worker_run (void)
{
	if (1u==_worker_fault)
	{
		debug_printf("# WORKER RUN\n");
	}
    
    _worker_fault = 0u;
}

void set_worker_fault (void)
{
	if (0u==_worker_fault)
	{
		debug_printf("# WORKER FAULT\n");
	}

	_worker_fault = 1u;
}

void set_worker_non_switch (void)
{
	if (1u==_worker_switch)
	{
		debug_printf("# WORKER NON SWITCH\n");
	}

	_worker_switch = 0u;
}

void set_worker_switch (void)
{
	if (0u==_worker_switch)
	{
		debug_printf("# WORKER SWITCH\n");
	}

	_worker_switch = 1u;
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _dido_heartbeat_t
{
	cx_uint_t clock       ;
	cx_uint_t error       ;
	cx_uint_t success     ;
	cx_uint_t max_check   ;

	cx_uint_t output;
	cx_uint_t input ;
}
dido_heartbeat_t;

//===========================================================================
static void dido_heartbeat_initialize (dido_heartbeat_t* ctx, cx_uint_t max_check)
{
	ctx->clock     = 0u;
	ctx->error     = 0u;
	ctx->success   = 0u;
	ctx->max_check = max_check;

	ctx->output = 0u;
	ctx->input  = 0u;
}

static void dido_heartbeat_request_set_output (dido_heartbeat_t* ctx, cx_uint_t v)
{
	ctx->output = v;
}

static cx_uint_t dido_heartbeat_request_get_input (dido_heartbeat_t* ctx)
{
	return ctx->input;
}

static void dido_heartbeat_response_set_output (dido_heartbeat_t* ctx, cx_uint_t v)
{
	ctx->output = v;
}

static cx_uint_t dido_heartbeat_response_get_input (dido_heartbeat_t* ctx)
{
	return ctx->input;
}

//===========================================================================
static cx_int_t dido_heartbeat_request (dido_heartbeat_t* ctx)
{
	cx_uint_t input;
	
	
	input = dido_heartbeat_request_get_input(ctx);
	

	if (input == ctx->clock)
	{
		ctx->clock = (ctx->clock+1)&0x01u;
		
		if (ctx->success < ctx->max_check)
		{
			ctx->success++;
		}
		else
		{
			ctx->error = 0u;
		}
	}
	else
	{
		if (ctx->error < ctx->max_check)
		{
			ctx->error++;
		}
		else
		{
			ctx->success = 0u;
		}
	}
	
	
	dido_heartbeat_request_set_output (ctx, ctx->clock);
	
	if (ctx->success == ctx->max_check)
		return 1;
	
	if (ctx->error   == ctx->max_check)
		return -1;
		
	return 0;
}

static cx_int_t dido_heartbeat_response (dido_heartbeat_t* ctx)
{
	cx_uint_t input;
	
	
	input = dido_heartbeat_response_get_input(ctx);
	

	if (input != ctx->clock)
	{
		ctx->clock = (ctx->clock+1)&0x01u;
		
		if (ctx->success < ctx->max_check)
		{
			ctx->success++;
		}
		else
		{
			ctx->error = 0u;
		}
	}
	else
	{
		if (ctx->error < ctx->max_check)
		{
			ctx->error++;
		}
		else
		{
			ctx->success = 0u;
		}
	}
	
	
	dido_heartbeat_response_set_output (ctx, input);
	
	if (ctx->success == ctx->max_check)
		return 1;

	if (ctx->error   == ctx->max_check)
		return -1;
		
	return 0;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define DIDO_HEARTBEAT_STATE_UNKNOWN 0u
#define DIDO_HEARTBEAT_STATE_OK      1u
#define DIDO_HEARTBEAT_STATE_FAIL    2u

//===========================================================================
typedef struct _pair_dido_heartbeat_t
{
	dido_heartbeat_t a;
	dido_heartbeat_t b;

	cx_int_t   a_value;
	cx_uint_t  a_state ;
	
	cx_int_t   b_value ;
	cx_uint_t  b_state;

	cx_uint_t  fail_max_count;
	cx_uint_t  fail_count;
	cx_bool_t  fail;
}
pair_dido_heartbeat_t;

//===========================================================================
static void pair_dido_heartbeat_initialize (pair_dido_heartbeat_t* ctx, cx_uint_t dido_heartbeat_max_check, cx_uint_t fail_max_count)
{
	dido_heartbeat_initialize(&ctx->a, dido_heartbeat_max_check);
	dido_heartbeat_initialize(&ctx->b, dido_heartbeat_max_check);

	ctx->a_value = 0;
	ctx->a_state = DIDO_HEARTBEAT_STATE_UNKNOWN;

	ctx->b_value = 0;
	ctx->b_state = DIDO_HEARTBEAT_STATE_UNKNOWN;

	ctx->fail_max_count = fail_max_count;
	ctx->fail_count     = 0u;
	ctx->fail           = CX_FALSE;
}

static void pair_dido_heartbeat_update (pair_dido_heartbeat_t* ctx)
{
	//-----------------------------------------------------------------------
	cx_int_t value;


	//-----------------------------------------------------------------------
	value = ctx->a_value;


	switch (value)
	{
	case  0:
		break;

	case  1:
		ctx->a_state = DIDO_HEARTBEAT_STATE_OK;
		break;

	case -1:
		ctx->a_state = DIDO_HEARTBEAT_STATE_FAIL;
		break;

	default:
		break;
	}


	//-----------------------------------------------------------------------
	value = ctx->b_value;


	switch (value)
	{
	case  0:
		break;

	case  1:
		ctx->b_state = DIDO_HEARTBEAT_STATE_OK;
		break;

	case -1:
		ctx->b_state = DIDO_HEARTBEAT_STATE_FAIL;
		break;

	default:
		break;
	}


	//-----------------------------------------------------------------------
	if ( ctx->a_state != ctx->b_state )
	{
		if (ctx->fail_count < ctx->fail_max_count)
		{
			ctx->fail_count++;
		}
	}
	else
	{
		ctx->fail_count = 0u;
	}

	if (ctx->fail_count >= ctx->fail_max_count)
	{
		ctx->fail = CX_TRUE;
	}
	else
	{
		ctx->fail = CX_FALSE;
	}
}




/////////////////////////////////////////////////////////////////////////////
//===========================================================================
/*
-------+------+--------------------------------------------------------------
 VALUE |  BIT | DESCRIPTION
-------+------+--------------------------------------------------------------
       |  ASR | Active Standby Run
-------+------+--------------------------------------------------------------
     4 |  100 | ACTIVE_NONE
     5 |  101 | ACTIVE_SWITCH
     3 |  011 | STANDBY_SWITCH
     2 |  010 | STANDBY_NONE
-------+------+--------------------------------------------------------------
     1 |  001 | BOOT
     6 |  110 | HALT
     7 |  111 | 
     0 |  000 | 
-------+------+--------------------------------------------------------------
*/
//      2계               1계
//d2, d1, d0, clk, d2, d1, d0, clk
//===========================================================================
typedef struct _dido_worker_state_t
{
	cx_uint_t fail_max_count;
	cx_uint_t fail_count;
	cx_bool_t fail;
	
	cx_uint_t worker_state_do ;
	cx_uint_t worker_state_do0;
	cx_uint_t worker_state_do1;
	cx_uint_t worker_state_do2;
	
	cx_uint_t worker_state_di0;
	cx_uint_t worker_state_di1;
	cx_uint_t worker_state_di2;
	cx_uint_t worker_state_di ;

	cx_uint_t coworker_state_do ;
	cx_uint_t coworker_state_do0;
	cx_uint_t coworker_state_do1;
	cx_uint_t coworker_state_do2;
	
	cx_uint_t coworker_state_di0;
	cx_uint_t coworker_state_di1;
	cx_uint_t coworker_state_di2;
	cx_uint_t coworker_state_di ;
}
dido_worker_state_t;

//===========================================================================
static void dido_worker_state_initialize (dido_worker_state_t* ctx, cx_uint_t fail_max_count)
{
	ctx->fail_max_count   = fail_max_count;
	ctx->fail_count       = 0u;
	ctx->fail             = CX_FALSE;

	ctx->worker_state_do  = 0u;
	ctx->worker_state_do0 = 0u;
	ctx->worker_state_do1 = 0u;
	ctx->worker_state_do2 = 0u;

	ctx->worker_state_di0 = 0u;
	ctx->worker_state_di1 = 0u;
	ctx->worker_state_di2 = 0u;
	ctx->worker_state_di  = 0u;

	ctx->coworker_state_do  = 0u;
	ctx->coworker_state_do0 = 0u;
	ctx->coworker_state_do1 = 0u;
	ctx->coworker_state_do2 = 0u;

	ctx->coworker_state_di0 = 0u;
	ctx->coworker_state_di1 = 0u;
	ctx->coworker_state_di2 = 0u;
	ctx->coworker_state_di  = 0u;
}

static void dido_worker_state_update (dido_worker_state_t* ctx, cx_uint_t worker_state, cx_bool_t coworker_connected)
{
	//-----------------------------------------------------------------------
	ctx->worker_state_do  = worker_state;
	ctx->worker_state_do0 = (ctx->worker_state_do & 0x01) >> 0u;
	ctx->worker_state_do1 = (ctx->worker_state_do & 0x02) >> 1u;
	ctx->worker_state_do2 = (ctx->worker_state_do & 0x04) >> 2u;

	ctx->worker_state_di =
		(ctx->worker_state_di0<<0u) |
		(ctx->worker_state_di1<<1u) |
		(ctx->worker_state_di2<<2u) ;

	if (CX_TRUE==coworker_connected)
	{
		
		if (ctx->worker_state_do!=ctx->worker_state_di)
		{
			if (ctx->fail_count<ctx->fail_max_count)
			{
				ctx->fail_count++;
			}
		}
		else
		{
			ctx->fail_count = 0u;
		}


		if (ctx->fail_count>=ctx->fail_max_count)
		{
			ctx->fail = CX_TRUE;
		}
		else
		{
			ctx->fail = CX_FALSE;
		}
	}
	else
	{
		ctx->fail_count = 0u;	
		ctx->fail       = CX_FALSE;
	}


	//-----------------------------------------------------------------------
	ctx->coworker_state_do0 = ctx->coworker_state_di0;
	ctx->coworker_state_do1 = ctx->coworker_state_di1;
	ctx->coworker_state_do2 = ctx->coworker_state_di2;

	ctx->coworker_state_di =
		(ctx->coworker_state_di0<<0u) |
		(ctx->coworker_state_di1<<1u) |
		(ctx->coworker_state_di2<<2u) ;

	ctx->coworker_state_do =
		(ctx->coworker_state_do0<<0u) |
		(ctx->coworker_state_do1<<1u) |
		(ctx->coworker_state_do2<<2u) ;	
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static pair_dido_heartbeat_t _pair_dido_heartbeat;
static dido_worker_state_t   _dido_worker_state;

//===========================================================================
void hotstandby_irq_handler (void)
{
	cx_uint_t worker_state ;
	cx_uint_t worker_number;
	cx_bool_t coworker_connected;


	worker_number = _worker_number;
	worker_state  = _worker_state ;

	if (worker_number==1u)
	{
		//---------------------------------------------------------------
		_pair_dido_heartbeat.a.input = GPIO_I_COWORKER_DI0();
		_pair_dido_heartbeat.b.input = GPIO_I_COWORKER_DI4();

		_pair_dido_heartbeat.a_value = dido_heartbeat_request (&_pair_dido_heartbeat.a);
		_pair_dido_heartbeat.b_value = dido_heartbeat_response(&_pair_dido_heartbeat.b);

		GPIO_O_COWORKER_DO0(_pair_dido_heartbeat.a.output);
		GPIO_O_COWORKER_DO4(_pair_dido_heartbeat.b.output);
		
		pair_dido_heartbeat_update(&_pair_dido_heartbeat);


		if ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_OK) ||
		     (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_OK) )
		{
			coworker_connected = CX_TRUE;
		}
		else
		{
			coworker_connected = CX_FALSE;
		}

		//---------------------------------------------------------------
		_dido_worker_state.worker_state_di0   = GPIO_I_COWORKER_DI1();	
		_dido_worker_state.worker_state_di1   = GPIO_I_COWORKER_DI2();
		_dido_worker_state.worker_state_di2   = GPIO_I_COWORKER_DI3();
		_dido_worker_state.coworker_state_di0 = GPIO_I_COWORKER_DI5();
		_dido_worker_state.coworker_state_di1 = GPIO_I_COWORKER_DI6();
		_dido_worker_state.coworker_state_di2 = GPIO_I_COWORKER_DI7();

		dido_worker_state_update(&_dido_worker_state, worker_state, coworker_connected);

		GPIO_O_COWORKER_DO1(_dido_worker_state.worker_state_do0  );
		GPIO_O_COWORKER_DO2(_dido_worker_state.worker_state_do1  );
		GPIO_O_COWORKER_DO3(_dido_worker_state.worker_state_do2  );
		GPIO_O_COWORKER_DO5(_dido_worker_state.coworker_state_do0);
		GPIO_O_COWORKER_DO6(_dido_worker_state.coworker_state_do1);
		GPIO_O_COWORKER_DO7(_dido_worker_state.coworker_state_do2);
	}
	else
	if (worker_number==2u)
	{
		//---------------------------------------------------------------
		_pair_dido_heartbeat.a.input = GPIO_I_COWORKER_DI4();
		_pair_dido_heartbeat.b.input = GPIO_I_COWORKER_DI0();

		_pair_dido_heartbeat.a_value = dido_heartbeat_request (&_pair_dido_heartbeat.a);
		_pair_dido_heartbeat.b_value = dido_heartbeat_response(&_pair_dido_heartbeat.b);

		GPIO_O_COWORKER_DO4(_pair_dido_heartbeat.a.output);
		GPIO_O_COWORKER_DO0(_pair_dido_heartbeat.b.output);

		pair_dido_heartbeat_update(&_pair_dido_heartbeat);


		if ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_OK) ||
		     (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_OK) )
		{
			coworker_connected = CX_TRUE;
		}
		else
		{
			coworker_connected = CX_FALSE;
		}


		//---------------------------------------------------------------
		_dido_worker_state.worker_state_di0   = GPIO_I_COWORKER_DI5();
		_dido_worker_state.worker_state_di1   = GPIO_I_COWORKER_DI6();
		_dido_worker_state.worker_state_di2   = GPIO_I_COWORKER_DI7();
		_dido_worker_state.coworker_state_di0 = GPIO_I_COWORKER_DI1();
		_dido_worker_state.coworker_state_di1 = GPIO_I_COWORKER_DI2();
		_dido_worker_state.coworker_state_di2 = GPIO_I_COWORKER_DI3();
		
		dido_worker_state_update(&_dido_worker_state, worker_state, coworker_connected);

		GPIO_O_COWORKER_DO5(_dido_worker_state.worker_state_do0  );	
		GPIO_O_COWORKER_DO6(_dido_worker_state.worker_state_do1  );
		GPIO_O_COWORKER_DO7(_dido_worker_state.worker_state_do2  );
		GPIO_O_COWORKER_DO1(_dido_worker_state.coworker_state_do0);
		GPIO_O_COWORKER_DO2(_dido_worker_state.coworker_state_do1);
		GPIO_O_COWORKER_DO3(_dido_worker_state.coworker_state_do2);
	}
}

//===========================================================================
cx_bool_t hotstandby_initialize (void)
{
	cx_uint_t worker_number = 1u;

	worker_number = _config.worker;

	memset (&_pair_dido_heartbeat, 0, sizeof(_pair_dido_heartbeat));
	memset (&_dido_worker_state  , 0, sizeof(_dido_worker_state  ));
//	pair_dido_heartbeat_initialize(&_pair_dido_heartbeat, 30u, 4000u);
//	dido_worker_state_initialize  (&_dido_worker_state  , 8000u);
    
// Original
//	pair_dido_heartbeat_initialize(&_pair_dido_heartbeat, 5u, 20u);  
//	dido_worker_state_initialize  (&_dido_worker_state  , 40u);

	pair_dido_heartbeat_initialize(&_pair_dido_heartbeat, 5u, 200u);  //DIDO 체크 타이밍 수정
	dido_worker_state_initialize  (&_dido_worker_state  , 400u);
	
	switch (worker_number)
	{
	case 1u:
		_coworker_number = 2u;
		break;

	case 2u:
		_coworker_number = 1u;
		break;

	default:
		return CX_FALSE;
		break;
	}

	_worker_state   = WORKER_STATE_BOOT;
	_coworker_state = WORKER_STATE_UNKNOWN;

	_worker_number = worker_number;
	
	return CX_TRUE;
}

//===========================================================================
static void hotstandby_update_coworker (void)
{
	cx_uint_t coworker_state_di;
	cx_uint_t coworker_state;


	coworker_state_di = _dido_worker_state.coworker_state_di;
	coworker_state    = WORKER_STATE_UNKNOWN;

	//---------------------------------------------------------------
	if     ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_OK) ||
	         (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_OK) )
	{
		switch(coworker_state_di)
		{
			case WORKER_STATE_BOOT         		: coworker_state = WORKER_STATE_BOOT        	; break;
			case WORKER_STATE_ACTIVE_SWITCH   	: coworker_state = WORKER_STATE_ACTIVE_SWITCH  	; break;
			case WORKER_STATE_ACTIVE_NONE  		: coworker_state = WORKER_STATE_ACTIVE_NONE 	; break;
			case WORKER_STATE_STANDBY_SWITCH  	: coworker_state = WORKER_STATE_STANDBY_SWITCH 	; break;
			case WORKER_STATE_STANDBY_NONE 		: coworker_state = WORKER_STATE_STANDBY_NONE	; break;
			case WORKER_STATE_HALT         		: coworker_state = WORKER_STATE_HALT        	; break;
			default:
				coworker_state = WORKER_STATE_UNKNOWN;
				break;
		}
	}
	else if ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_FAIL) &&
	          (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_FAIL) )
	{
		coworker_state = WORKER_STATE_HALT;
	}
	else
	{
		coworker_state = WORKER_STATE_UNKNOWN;
	}


	//-----------------------------------------------------------------------
	if (_coworker_state==coworker_state)
	{
		return;
	}
	else
	{
#if 1		
		static cx_uint_t pre_coworker_state = 0u;
		static cx_uint_t switchover_count = 0u;
		cx_uint_t switchover_max_count = 3u;	//3번 체크 
		
		if(pre_coworker_state == coworker_state)
		{
			if(switchover_count>=switchover_max_count)
			{
				switchover_count = 0u;
			}	
			else
			{
				debug_printf("# PRE COWORKER = %s \n",get_worker_state_string(pre_coworker_state));
				switchover_count++;
				
				return;
			}
		}	
		else
		{
			pre_coworker_state = coworker_state;
			switchover_count = 0u;
			
			return;
		}
#endif		
	}	


	debug_printf("# COWORKER = %s <- %s : di=%d:%d(%d.%d.%d) \n",
		get_worker_state_string(coworker_state),
		get_worker_state_string(_coworker_state),
		coworker_state_di,
		_dido_worker_state.coworker_state_di  ,
		_dido_worker_state.coworker_state_di0 ,
		_dido_worker_state.coworker_state_di1 ,
		_dido_worker_state.coworker_state_di2 );
		
	debug_flush();	

	//-----------------------------------------------------------------------
	_coworker_state = coworker_state;
}

//===========================================================================
static cx_uint_t hotstandby_update_worker1_boot (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_BOOT        ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_STANDBY_NONE; 	break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	default:
		break;
	}

	
	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker1_active_switch (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break; 
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;	
	case WORKER_STATE_ACTIVE_SWITCH  	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break; 
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	default:
		break;
	}


	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker1_active_none (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	default:
		break;
	}

	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker1_standby_switch (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_SWITCH ; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	default:
		break;
	}
	
	
	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker1_standby_none (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	default:
		break;
	}

	return worker_transition_state;
}
/*
static cx_uint_t hotstandby_update_worker1_halt (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	return WORKER_STATE_HALT;
}
*/

//===========================================================================
static cx_uint_t hotstandby_update_worker2_boot (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_BOOT        ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_STANDBY_NONE ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE ; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE ; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE  ; break;
	default:
		break;
	}


	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker2_active_switch (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break; 
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_SWITCH; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break; 
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH ; break;
	default:
		break;
	}
	
	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker2_active_none (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	default:
		break;
	}

	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker2_standby_switch (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_SWITCH ; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_SWITCH ; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
	default:
		break;
	}

	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker2_standby_none (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( coworker_state )
	{
	//case WORKER_STATE_UNKNOWN      	: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_BOOT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	case WORKER_STATE_ACTIVE_SWITCH   	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_ACTIVE_NONE  		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_SWITCH  	: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_STANDBY_NONE 		: worker_transition_state = WORKER_STATE_STANDBY_NONE; break;
	case WORKER_STATE_HALT         		: worker_transition_state = WORKER_STATE_ACTIVE_NONE ; break;
	default:
		break;
	}

	return worker_transition_state;
}
/*
static cx_uint_t hotstandby_update_worker2_halt (cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	return WORKER_STATE_HALT;
}
*/

//===========================================================================
static cx_uint_t hotstandby_update_worker1 (cx_uint_t worker_state, cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( worker_state )
	{
	case WORKER_STATE_BOOT        	 : worker_transition_state = hotstandby_update_worker1_boot        	 (worker_transition_state, coworker_state); break;
	case WORKER_STATE_ACTIVE_SWITCH	 : worker_transition_state = hotstandby_update_worker1_active_switch (worker_transition_state, coworker_state); break;
	case WORKER_STATE_ACTIVE_NONE  	 : worker_transition_state = hotstandby_update_worker1_active_none   (worker_transition_state, coworker_state); break;
	case WORKER_STATE_STANDBY_SWITCH : worker_transition_state = hotstandby_update_worker1_standby_switch(worker_transition_state, coworker_state); break;
	case WORKER_STATE_STANDBY_NONE 	 : worker_transition_state = hotstandby_update_worker1_standby_none	 (worker_transition_state, coworker_state); break;
//	case WORKER_STATE_HALT         : worker_transition_state = hotstandby_update_worker1_halt        (worker_transition_state, coworker_state); break;
	default:
		worker_transition_state = WORKER_STATE_HALT;
		break;
	}


	return worker_transition_state;
}

static cx_uint_t hotstandby_update_worker2 (cx_uint_t worker_state, cx_uint_t worker_transition_state, cx_uint_t coworker_state)
{
	switch ( worker_state )
	{
	case WORKER_STATE_BOOT           : worker_transition_state = hotstandby_update_worker2_boot           (worker_transition_state, coworker_state); break;
	case WORKER_STATE_ACTIVE_SWITCH  : worker_transition_state = hotstandby_update_worker2_active_switch  (worker_transition_state, coworker_state); break;
	case WORKER_STATE_ACTIVE_NONE    : worker_transition_state = hotstandby_update_worker2_active_none 	  (worker_transition_state, coworker_state); break;
	case WORKER_STATE_STANDBY_SWITCH : worker_transition_state = hotstandby_update_worker2_standby_switch (worker_transition_state, coworker_state); break;
	case WORKER_STATE_STANDBY_NONE   : worker_transition_state = hotstandby_update_worker2_standby_none	  (worker_transition_state, coworker_state); break;
//	case WORKER_STATE_HALT         : worker_transition_state = hotstandby_update_worker2_halt        (worker_transition_state, coworker_state); break;
	default:
		worker_transition_state = WORKER_STATE_HALT;
		break;
	}

	return worker_transition_state;
}

//===========================================================================
static void hotstandby_update_worker (void)
{
	cx_uint_t worker_number;
	cx_uint_t worker_state;
	cx_uint_t coworker_state;
	cx_uint_t worker_transition_state;
	
	static cx_bool_t pre_worker_state = 0u;;
	//-----------------------------------------------------------------------
	worker_number           = _worker_number;
	worker_state            = _worker_state ;
	coworker_state          = _coworker_state;
	worker_transition_state = WORKER_STATE_HALT;
	worker_transition_state = worker_state;
	
	//-----------------------------------------------------------------------	
	if (CX_TRUE==_worker_switch )
	{

		switch ( worker_state )
		{
			case WORKER_STATE_ACTIVE_NONE  : worker_state = WORKER_STATE_ACTIVE_SWITCH  ; break;
			case WORKER_STATE_STANDBY_NONE : worker_state = WORKER_STATE_STANDBY_SWITCH ; break;
			default:
				break;
		}

	}
	else
	{

		switch ( worker_state )
		{
			case WORKER_STATE_ACTIVE_SWITCH   : worker_state = WORKER_STATE_ACTIVE_NONE ; break;
			case WORKER_STATE_STANDBY_SWITCH  : worker_state = WORKER_STATE_STANDBY_NONE; break;
			default:
				break;
		}

	}
	//-----------------------------------------------------------------------
	if (CX_TRUE==_worker_fault)
	{
		worker_state = WORKER_STATE_HALT;
		if(pre_worker_state == CX_FALSE)
		{
			debug_printf("\tworker fail\n");
			pre_worker_state = CX_TRUE;
		}	
	}
	
	if (CX_TRUE==_pair_dido_heartbeat.fail)
	{
		if (_worker_number==2u)
		{
			worker_state = WORKER_STATE_HALT;

			_pair_dido_heartbeat_fault = CX_TRUE;
			
			if(pre_worker_state == CX_FALSE)
			{
				debug_printf("\tpair dido heartbeat fail\n");
				pre_worker_state = CX_TRUE;
			}		
		}
	}
	if (CX_TRUE==_dido_worker_state.fail)
	{
		worker_state = WORKER_STATE_HALT;
		
		if(pre_worker_state == CX_FALSE)
		{
			pre_worker_state = CX_TRUE;
			debug_printf("\tpair dido state fail\n");
		}	
	}
	
	// HALT 조건이 아니라면, 리셋 하지 않도록 리셋 관련 flag는 클리어
	if ((CX_FALSE==_worker_fault)&&(CX_FALSE==_pair_dido_heartbeat.fail)&&(CX_FALSE==_dido_worker_state.fail))
	{
		_pair_dido_heartbeat_fault = CX_FALSE;
	}
	
	//-----------------------------------------------------------------------
    if      (1==worker_number)
	{
		worker_transition_state = hotstandby_update_worker1(worker_state, worker_transition_state, coworker_state);
	}
	else if (2==worker_number)
	{
		worker_transition_state = hotstandby_update_worker2(worker_state, worker_transition_state, coworker_state);
	}


	//-----------------------------------------------------------------------
    if (_worker_state==worker_transition_state)
	{
		return;
	}
    
    
	debug_printf("# WORKER = %s <- %s(%s) : coworker=%s fault=%d run=%d \n",
		get_worker_state_string(worker_transition_state),
		get_worker_state_string(_worker_state),
		get_worker_state_string(worker_state),
		get_worker_state_string(coworker_state),
		_worker_fault,
		_worker_switch 
		);


	_worker_state=worker_transition_state;

}

void hotstandby_update (void)
{
	hotstandby_update_coworker();
	hotstandby_update_worker();	
}

cx_bool_t check_pair_dido_heartbeat_fault(cx_uint_t pair_dido_fault_cause)
{
	if(pair_dido_fault_cause == 1u)
	{
		return CX_TRUE;
	}
	else return CX_FALSE;
}
cx_bool_t pair_dido_heartbeat_fault_update (void)
{
	return check_pair_dido_heartbeat_fault (_pair_dido_heartbeat_fault);
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================

cx_byte_t redundant_get_status (void)
{
	cx_byte_t code;

	code = 0u;

	if (1u==get_worker_number())
	{
		code |= ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_OK) ? 0x01 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di0                       ? 0x02 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di1                       ? 0x04 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di2                       ? 0x08 : 0x00 );


		code |= ( (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_OK) ? 0x10 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di0                     ? 0x20 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di1                     ? 0x40 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di2                     ? 0x80 : 0x00 );
	}

	if (2u==get_worker_number())
	{
		code |= ( (_pair_dido_heartbeat.a_state == DIDO_HEARTBEAT_STATE_OK) ? 0x10 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di0                       ? 0x20 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di1                       ? 0x40 : 0x00 );
		code |= ( _dido_worker_state.worker_state_di2                       ? 0x80 : 0x00 );

		code |= ( (_pair_dido_heartbeat.b_state == DIDO_HEARTBEAT_STATE_OK) ? 0x01 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di0                     ? 0x02 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di1                     ? 0x04 : 0x00 );
		code |= ( _dido_worker_state.coworker_state_di2                     ? 0x08 : 0x00 );
	}

	return code;
}

void redundant_show (void)
{
	//-----------------------------------------------------------------------
	debug_printf("# REDUNDANT\n");


	//-----------------------------------------------------------------------
	debug_printf("\tworker  [%d] = %s(%d) \n", _worker_number  , get_worker_state_string(_worker_state  ), _worker_state  );
	debug_printf("\tcoworker[%d] = %s(%d) \n", _coworker_number, get_worker_state_string(_coworker_state), _coworker_state);
	debug_printf("\tfault       = %d \n", _worker_fault );
	debug_printf("\trun         = %d \n", !(_worker_fault) );


	//-----------------------------------------------------------------------
	debug_printf("\tpair dido heartbeat\n");

	debug_printf("\t\ta clock        = %d\n", _pair_dido_heartbeat.a.clock       );
	debug_printf("\t\ta error        = %d\n", _pair_dido_heartbeat.a.error       );
	debug_printf("\t\ta success      = %d\n", _pair_dido_heartbeat.a.success     );
	debug_printf("\t\ta max check    = %d\n", _pair_dido_heartbeat.a.max_check   );
	debug_printf("\t\ta output       = %d\n", _pair_dido_heartbeat.a.output      );
	debug_printf("\t\ta input        = %d\n", _pair_dido_heartbeat.a.input       );
	debug_printf("\t\ta value        = %d\n", _pair_dido_heartbeat.a_value       );
	debug_printf("\t\ta state        = %d\n", _pair_dido_heartbeat.a_state       );

	debug_printf("\t\tb clock        = %d\n", _pair_dido_heartbeat.b.clock       );
	debug_printf("\t\tb error        = %d\n", _pair_dido_heartbeat.b.error       );
	debug_printf("\t\tb success      = %d\n", _pair_dido_heartbeat.b.success     );
	debug_printf("\t\tb max_check    = %d\n", _pair_dido_heartbeat.b.max_check   );
	debug_printf("\t\tb output       = %d\n", _pair_dido_heartbeat.b.output      );
	debug_printf("\t\tb input        = %d\n", _pair_dido_heartbeat.b.input       );
	debug_printf("\t\tb value        = %d\n", _pair_dido_heartbeat.b_value       );
	debug_printf("\t\tb state        = %d\n", _pair_dido_heartbeat.b_state       );

	debug_printf("\t\tfail max count = %d\n", _pair_dido_heartbeat.fail_max_count);
	debug_printf("\t\tfail count     = %d\n", _pair_dido_heartbeat.fail_count    );
	debug_printf("\t\tfail           = %d\n", _pair_dido_heartbeat.fail          );


	//-----------------------------------------------------------------------
	debug_printf("\tdido worker state\n");

	debug_printf("\t\tworker   do    = %d(%d.%d.%d)\n", 
		_dido_worker_state.worker_state_do  ,
		_dido_worker_state.worker_state_do0 ,
		_dido_worker_state.worker_state_do1 ,
		_dido_worker_state.worker_state_do2 );

	debug_printf("\t\tworker   di    = %d(%d.%d.%d)\n", 
		_dido_worker_state.worker_state_di  ,
		_dido_worker_state.worker_state_di0 ,
		_dido_worker_state.worker_state_di1 ,
		_dido_worker_state.worker_state_di2 );

	debug_printf("\t\tcoworker do    = %d(%d.%d.%d)\n", 
		_dido_worker_state.coworker_state_do  ,
		_dido_worker_state.coworker_state_do0 ,
		_dido_worker_state.coworker_state_do1 ,
		_dido_worker_state.coworker_state_do2 );

	debug_printf("\t\tcoworker di    = %d(%d.%d.%d)\n", 
		_dido_worker_state.coworker_state_di  ,
		_dido_worker_state.coworker_state_di0 ,
		_dido_worker_state.coworker_state_di1 ,
		_dido_worker_state.coworker_state_di2 );

	debug_printf("\t\tfail max count = %d\n", _dido_worker_state.fail_max_count   );
	debug_printf("\t\tfail count     = %d\n", _dido_worker_state.fail_count       );
	debug_printf("\t\tfail           = %d\n", _dido_worker_state.fail             );


	//-----------------------------------------------------------------------
	debug_printf("\tapplication halt = %d \n", _application_halt );
}
