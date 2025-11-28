/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "mcu.h"





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
//#define PEER_DEBUG_SEQUENCE_ERROR





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static cx_bool_t _peer_timer_flag = CX_FALSE;


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
peer_connection_system_t _peer_connection[PEER_CONNECTION_MAX_COUNT];
//===========================================================================


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_connection_initialize (peer_connection_t* ctx, cx_uint_t id, cx_uint_t timeout_maxcount)
{
	ctx->id = id;

	ctx->rx_sequence = 0u;
	ctx->rx_flag     = CX_FALSE;
	ctx->timer_flag  = CX_FALSE;

	ctx->timeout          = CX_TRUE;
	ctx->timeout_count    = timeout_maxcount;
	ctx->timeout_maxcount = timeout_maxcount;
}

void peer_connection_set_rx_flag (peer_connection_t* ctx, cx_bool_t rx_flag)
{
	ctx->rx_flag = rx_flag;

	if ( CX_TRUE==rx_flag )
	{
		peer_connection_set_timeout(ctx, CX_FALSE);
	}
}

void peer_connection_set_timeout (peer_connection_t* ctx, cx_bool_t timeout)
{
    cx_uint8_t type   ;
	cx_uint8_t unit   ;
	cx_uint8_t worker ;
	
	if ( ctx->timeout!=timeout )
	{
		ctx->timeout=timeout;

		type   = nb_packet_get_id_type   (/*(cx_uint8_t)*/ctx->id); //
		unit   = nb_packet_get_id_unit   (/*(cx_uint8_t)*/ctx->id);
		worker = nb_packet_get_id_worker (/*(cx_uint8_t)*/ctx->id);
	
		if ( CX_TRUE==timeout )
		{
			debug_printf ("peer[%x] t%d.u%d.w%d disconnected\n", 
				ctx->id,
				type   ,
				unit   ,
				worker 				
				);

			equipment_set_connection(type, unit, worker, CX_FALSE);
		}
		else
		{
			debug_printf ("peer[%x] t%d.u%d.w%d connected\n", 
				ctx->id,
				type   ,
				unit   ,
				worker 
				);

			equipment_set_connection(type, unit, worker, CX_TRUE);
		}
	}
}

cx_bool_t peer_connection_get_timeout (peer_connection_t* ctx)
{
	return ctx->timeout;
}

void peer_connection_check_timeout (peer_connection_t* ctx)
{
	if ( CX_FALSE==ctx->timer_flag )
	{
		return;
	}


	if ( CX_FALSE==ctx->rx_flag )
	{
		if (ctx->timeout_count<ctx->timeout_maxcount)
		{
			ctx->timeout_count++;
		}
		else
		{
			peer_connection_set_timeout(ctx, CX_TRUE);
		}
	}
	else
	{
		ctx->timeout_count = 0u;

		peer_connection_set_timeout(ctx, CX_FALSE);

		ctx->rx_flag = CX_FALSE;
	}


	ctx->timer_flag = CX_FALSE;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_timer_irq_handler (void)
{
	// 100msec
	_peer_timer_flag = CX_TRUE;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
peer_connection_system_t* peer_find_connection (cx_uint16_t id)
{
	peer_connection_system_t* pc;

	cx_uint_t index;
	cx_uint_t type;
	cx_uint_t unit;


	pc    = CX_NULL_POINTER;
	type  = nb_packet_get_id_type (id);
	unit  = nb_packet_get_id_unit (id);
	index = PEER_CONNECTION_MAX_COUNT;
	
	switch (type)
	{
		case EQUIPMENT_TYPE_TRANSMITTER:
			if (unit<=EQUIPMENT_TRANSMITTER_MAX_COUNT)
			{
				index = PEER_CONNECTION_INDEX_B_TRANSMITTER+unit-1u;
			}
			break;

		case EQUIPMENT_TYPE_RECEIVER  :
			if (unit<=EQUIPMENT_RECEIVER_MAX_COUNT)
			{
				index = PEER_CONNECTION_INDEX_B_RECEIVER+unit-1u;
			}
			break;

		case EQUIPMENT_TYPE_DATA_GATHERING_BOARD:
			if (unit<=EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT)
			{
				index = PEER_CONNECTION_INDEX_B_DATA_GATHERING_BOARD+unit-1u;
			}
			break;

		default:
			break;
	}


	if ( index<PEER_CONNECTION_MAX_COUNT )
	{
		pc = &_peer_connection[index];
	}

	return pc;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_initialize (void)
{
	//-----------------------------------------------------------------------
	peer_connection_system_t* pc;

	cx_uint_t i;

	for (i=0u; i<PEER_CONNECTION_MAX_COUNT; i++)
	{
		pc = &_peer_connection[i];
		
		if      (i<PEER_CONNECTION_INDEX_E_TRANSMITTER)
		{
			pc->type = EQUIPMENT_TYPE_TRANSMITTER;
			pc->unit = i+1u-PEER_CONNECTION_INDEX_B_TRANSMITTER;
		}
		else if (i<PEER_CONNECTION_INDEX_E_RECEIVER)
		{
			pc->type = EQUIPMENT_TYPE_RECEIVER;
			pc->unit = i+1u-PEER_CONNECTION_INDEX_B_RECEIVER;
		}
		else if (i<PEER_CONNECTION_INDEX_E_DATA_GATHERING_BOARD)
		{
			pc->type = EQUIPMENT_TYPE_DATA_GATHERING_BOARD;
			pc->unit = i+1u-PEER_CONNECTION_INDEX_B_DATA_GATHERING_BOARD;
		}
		else
		{
			pc->type = 0u;
			pc->unit = 0u;
		}
		
		
		peer_connection_initialize(&pc->worker1, nb_packet_make_id(pc->type, pc->unit, 1u), 30u);
		pc->worker1.rx_sequence = 0u;    

		peer_connection_initialize(&pc->worker2, nb_packet_make_id(pc->type, pc->unit, 2u), 30u);
		pc->worker2.rx_sequence = 0u;		
		
	}
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_update (void)
{
	//-----------------------------------------------------------------------
	if ( CX_FALSE==_peer_timer_flag )
	{
		return;
	}

	//-----------------------------------------------------------------------
	peer_connection_system_t* pc;
	cx_uint_t i;


	for (i=0u; i<PEER_CONNECTION_MAX_COUNT; i++)
	{
		pc = &_peer_connection[i];
	
		pc->worker1.timer_flag = CX_TRUE;
		pc->worker2.timer_flag = CX_TRUE;
		
		peer_connection_check_timeout(&pc->worker1);
		peer_connection_check_timeout(&pc->worker2);
		
		if (  CX_TRUE==peer_connection_get_timeout(&pc->worker1))
		{
			pc->worker1.rx_sequence = 0u;
			
			peer_reset_data(pc->type, pc->unit, 1u);
		}	
		
		if (  CX_TRUE==peer_connection_get_timeout(&pc->worker2))
		{
			pc->worker2.rx_sequence = 0u;
			
			peer_reset_data(pc->type, pc->unit, 2u);
		}		
	}	
	//-----------------------------------------------------------------------
	_peer_timer_flag = CX_FALSE;
}

void peer_rx_packet (nb_packet_t* packet)
{
	peer_connection_system_t* pc;

	pc = peer_find_connection( packet->head.source );
	if (CX_NULL_POINTER==pc)
	{
		return;
	}

	//-----------------------------------------------------------------------
	if(packet->head.source==pc->worker1.id)
	{
		//packet->head.destination==0xFFFFu || packet->head.destination==0x000Fu 

		if (CX_TRUE==nb_packet_check_sequence_range(pc->worker1.rx_sequence, packet->head.sequence))
		{
			pc->worker1.rx_sequence = packet->head.sequence;

			peer_set_data(packet);
			
			peer_connection_set_rx_flag(&pc->worker1, CX_TRUE);
		}
		else
		{
//#ifdef PEER_DEBUG_SEQUENCE_ERROR
			debug_printf ("peer[%x] t%d.u%d.w%d.sequence error (%d->%d)\n", 
				packet->head.source,
				nb_packet_get_id_type   (packet->head.source),
				nb_packet_get_id_unit   (packet->head.source),
				nb_packet_get_id_worker (packet->head.source),
				pc->worker1.rx_sequence, packet->head.sequence
				);
//#endif
		}
	}
	//-----------------------------------------------------------------------
	else if (packet->head.source==pc->worker2.id)
	{
		if (CX_TRUE==nb_packet_check_sequence_range(pc->worker2.rx_sequence, packet->head.sequence))
		{
			pc->worker2.rx_sequence = packet->head.sequence;

			peer_set_data(packet);
			
			peer_connection_set_rx_flag(&pc->worker2, CX_TRUE);
		}
		else
		{
//#ifdef PEER_DEBUG_SEQUENCE_ERROR
			debug_printf ("peer[%x] t%d.u%d.w%d.sequence error (%d->%d)\n", 
				packet->head.source,
				nb_packet_get_id_type   (packet->head.source),
				nb_packet_get_id_unit   (packet->head.source),
				nb_packet_get_id_worker (packet->head.source),
				pc->worker2.rx_sequence, packet->head.sequence
				);
//#endif
		}	
	}
	//-----------------------------------------------------------------------
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_set_data (nb_packet_t* packet)
{
	cx_uint_t type;
	cx_uint_t unit;
	cx_uint_t worker;

	type   = nb_packet_get_id_type   (packet->head.source);
	unit   = nb_packet_get_id_unit   (packet->head.source);
	worker = nb_packet_get_id_worker (packet->head.source);

	
	equipment_set_data
	(
		type, unit, worker, 
		packet->head.sequence, packet->head.id, packet->head.timestamp, 
		packet->body.pointer , packet->body.size
	);
}

void peer_reset_data (cx_uint_t type, cx_uint_t unit, cx_uint_t worker)
{
	equipment_reset_data(type, unit, worker);
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void peer_show (void)
{
	peer_connection_system_t* pc;
	cx_uint_t i;


	debug_printf("# PEER CONNECTION\n");
	
	pc = &_peer_connection[0];

	debug_printf("\t[%3d] %d:%-12s %2d w1= {%04xh=%04xh:%c} w2= {%04xh=%04xh:%c}\n",
		0,
		pc->type, equipment_get_type_string(pc->type), pc->unit,

		pc->worker1.id,
		pc->worker1.rx_sequence,
		pc->worker1.timeout    ? ' ' : 'C' ,

		pc->worker2.id,
		pc->worker2.rx_sequence,
		pc->worker2.timeout    ? ' ' : 'C' );

	debug_flush();
	
	for (i=1u; i<PEER_CONNECTION_MAX_COUNT; i++)
	{
		pc = &_peer_connection[i];
		
		debug_printf("\t[%3d] %d:%-12s %2d w1= {%04xh=%04xh:%c}\n",
		i,
		pc->type, equipment_get_type_string(pc->type), pc->unit,

		pc->worker1.id,
		pc->worker1.rx_sequence,
		pc->worker1.timeout    ? ' ' : 'C' );
		
		debug_flush();
	}
}



