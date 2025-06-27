/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "mcu.h"





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
equipment_t _equipment;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void equipment_initialize(void)
{
	//-----------------------------------------------------------------------
	memset (&_equipment, 0, sizeof(_equipment));
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_char_t* equipment_get_type_string (cx_uint_t v)
{
	cx_char_t* s;

	switch(v)
	{		
		case EQUIPMENT_TYPE_TRANSMITTER				: s = "TRANSMITTER"; 	break;
		case EQUIPMENT_TYPE_RECEIVER				: s = "RECEIVER"; 		break;
		case EQUIPMENT_TYPE_DATA_GATHERING_BOARD	: s = "DG_BOARD"; 		break;
		default                      				: s = "NONE";      		break;
	}

	return s;
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void equipment_reset_data (cx_uint_t type, cx_uint_t unit, cx_uint_t worker)
{
	equipment_transmitter_t*			transmitter;
	equipment_receiver_t*				receiver;
	equipment_data_gathering_board_t*	data_gathering_board;

	cx_uint_t i;
	cx_uint_t j;


	switch (type)
	{
	case EQUIPMENT_TYPE_TRANSMITTER:
		if ( (1u<=unit) && (unit<=EQUIPMENT_TRANSMITTER_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;

			transmitter = &_equipment.transmitter[i][j];
			equipment_reset_data_transmitter (transmitter);
		}
		break;

	case EQUIPMENT_TYPE_RECEIVER  :
		if ( (1u<=unit) && (unit<=EQUIPMENT_RECEIVER_MAX_COUNT) )
		{
			i = unit-1u;
			j = 0u;
			
			receiver =	&_equipment.receiver[i][j];
			equipment_reset_data_receiver (receiver);
		}
		break;

	case EQUIPMENT_TYPE_DATA_GATHERING_BOARD:
		if ( (1u<=unit) && (unit<=EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT) )
		{
			i = unit-1u;
			j = 0u;
			
			data_gathering_board = &_equipment.data_gathering_board[i][j];
			equipment_reset_data_gathering_board (data_gathering_board);
		}
		break;
		
	default:
		break;
	}
}


void equipment_set_data (cx_uint_t type, cx_uint_t unit, cx_uint_t worker, 
                         cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, 
                         cx_byte_t* data_pointer, cx_uint_t data_size)
{
	equipment_transmitter_t*			transmitter;
	equipment_receiver_t*				receiver;
	equipment_data_gathering_board_t*	data_gathering_board;

	cx_uint_t i;
	cx_uint_t j;


	switch (type)
	{
	case EQUIPMENT_TYPE_TRANSMITTER:
		if ( (1u<=unit) && (unit<=EQUIPMENT_TRANSMITTER_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			transmitter = &_equipment.transmitter[i][j];
			equipment_set_data_transmitter (data_sequence, data_id, data_timestamp, data_pointer, data_size, transmitter);
		}
		break;

	case EQUIPMENT_TYPE_RECEIVER  :
		if ( (1u<=unit) && (unit<=EQUIPMENT_RECEIVER_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			receiver =	&_equipment.receiver[i][j];
			equipment_set_data_receiver (data_sequence, data_id, data_timestamp, data_pointer, data_size, receiver);
		}
		break;

	case EQUIPMENT_TYPE_DATA_GATHERING_BOARD:
		if ( (1u<=unit) && (unit<=EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			data_gathering_board = &_equipment.data_gathering_board[i][j];
			equipment_set_data_gathering_board (data_sequence, data_id, data_timestamp, data_pointer, data_size, data_gathering_board);
		}
		break;
		
	default:
		break;
	}
}

void equipment_set_connection(cx_uint_t type, cx_uint_t unit, cx_uint_t worker, cx_bool_t connection)
{
	cx_uint_t i;
	cx_uint_t j;


	switch (type)
	{
	case EQUIPMENT_TYPE_TRANSMITTER:
		if ( (1u<=unit) && (unit<=EQUIPMENT_TRANSMITTER_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			_equipment.transmitter[i][j].connection = connection;

		}
		break;

	case EQUIPMENT_TYPE_RECEIVER  :
		if ( (1u<=unit) && (unit<=EQUIPMENT_RECEIVER_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			_equipment.receiver[i][j].connection = connection;
			
		}
		break;

	case EQUIPMENT_TYPE_DATA_GATHERING_BOARD:
		if ( (1u<=unit) && (unit<=EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT) )
		{
			i = unit-1u;
			j = (worker==1u) ? 0u : 1u;
			
			_equipment.data_gathering_board[i][j].connection = connection;
			
		}
		break;

	//===========================================================================		
	default:
		break;
	}
}

void equipment_update (void)
{
	cx_uint_t i;
		
	for (i=0u; i<EQUIPMENT_TRANSMITTER_MAX_COUNT; i++)
	{
		equipment_update_transmitter ( &_equipment.transmitter[i][0] );
		equipment_update_transmitter ( &_equipment.transmitter[i][1] );
	}
	for (i=0u; i<EQUIPMENT_RECEIVER_MAX_COUNT  ; i++) 
	{
		equipment_update_receiver   ( &_equipment.receiver  [i][0] ); 
	}
	for (i=0u; i<EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT; i++) 
	{
		equipment_update_data_gathering_board ( &_equipment.data_gathering_board[i][0] );
	}
	//===========================================================================	
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void equipment_set_data_transmitter 
(
	cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, 
	cx_byte_t* data_pointer, cx_uint_t data_size, 
	equipment_transmitter_t* e		
)
{
	if (1u!=data_id)
	{
		return;	
	}
	
	e->sequence   = data_sequence;
	e->timestamp2 = data_timestamp;


	if (CX_TRUE==e->timestamp_stuck)
	{
		return;
	}


	cx_uint_t copy;

	copy = ( sizeof(e->data) < data_size ) ? sizeof(e->data) :  data_size;
	memset(e->data, 0, sizeof(e->data));
	memcpy(e->data, data_pointer, copy);
}

void equipment_set_data_receiver
(
	cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, 
	cx_byte_t* data_pointer, cx_uint_t data_size, 
	equipment_receiver_t* e
)
{
	if (2u!=data_id)
	{
		return;
	}
	
	e->sequence   = data_sequence;
	e->timestamp2 = data_timestamp;


	if (CX_TRUE==e->timestamp_stuck)
	{
		return;
	}


	cx_uint_t copy;

	copy = ( sizeof(e->data) < data_size ) ? sizeof(e->data) :  data_size;
	memset(e->data, 0, sizeof(e->data));
	memcpy(e->data, data_pointer, copy);
}

void equipment_set_data_gathering_board
(
	cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, 
	cx_byte_t* data_pointer, cx_uint_t data_size, 
	equipment_data_gathering_board_t* e
)
{
	if (3u!=data_id)
	{
		return;
	}
	
	e->sequence   = data_sequence;
	e->timestamp2 = data_timestamp;


	if (CX_TRUE==e->timestamp_stuck)
	{
		return;
	}


	cx_uint_t copy;

	copy = ( sizeof(e->data) < data_size ) ? sizeof(e->data) :  data_size;
	memset(e->data, 0, sizeof(e->data));
	memcpy(e->data, data_pointer, copy);
}

//===========================================================================
void equipment_update_transmitter (equipment_transmitter_t* e)
{
	if (e->timestamp1==e->timestamp2)
	{
		if (CX_TRUE==e->connection)  
		{
			e->timestamp_stuck = CX_TRUE;

			equipment_reset_data_transmitter(e);	
		}
		else
		{
			e->timestamp_stuck = CX_FALSE;
			e->timestamp1 = 0u;
			e->timestamp2 = 0u;
		}
	}
	else
	{
		e->timestamp_stuck = CX_FALSE;

		e->timestamp1 = e->timestamp2;
	}
}

void equipment_update_receiver (equipment_receiver_t* e)
{
	if (e->timestamp1==e->timestamp2)
	{
		if (CX_TRUE==e->connection) 
		{
			e->timestamp_stuck = CX_TRUE;
			
			equipment_reset_data_receiver(e);		
		}
		else
		{
			e->timestamp_stuck = CX_FALSE;
			e->timestamp1 = 0u;
			e->timestamp2 = 0u;
		}
	}
	else
	{
		e->timestamp_stuck = CX_FALSE;

		e->timestamp1 = e->timestamp2;
	}
}

void equipment_update_data_gathering_board (equipment_data_gathering_board_t* e)
{
	if (e->timestamp1==e->timestamp2)
	{
		if (CX_TRUE==e->connection) 
		{
			e->timestamp_stuck = CX_TRUE;
			
			equipment_reset_data_gathering_board(e);		
		}
		else
		{
			e->timestamp_stuck = CX_FALSE;
			e->timestamp1 = 0u;
			e->timestamp2 = 0u;
		}
	}
	else
	{
		e->timestamp_stuck = CX_FALSE;

		e->timestamp1 = e->timestamp2;
	}
}

//===========================================================================
void equipment_reset_data_transmitter (equipment_transmitter_t* e)
{
//	e->sequence   = 0u;
//	e->timestamp1 = 0u;
//	e->timestamp2 = 0u;

	memset(e->data, 0, sizeof(e->data));
}

void equipment_reset_data_receiver (equipment_receiver_t* e)
{
//	e->sequence   = 0u;
//	e->timestamp1 = 0u;
//	e->timestamp2 = 0u;

	memset(e->data, 0, sizeof(e->data));
}

void equipment_reset_data_gathering_board (equipment_data_gathering_board_t* e)
{
//	e->sequence   = 0u;
//	e->timestamp1 = 0u;
//	e->timestamp2 = 0u;

	memset(e->data, 0, sizeof(e->data));
}

//===========================================================================
void equipment_show_data_transmitter (cx_uint_t i, cx_uint_t w, equipment_transmitter_t* e)
{
	debug_printf("[%2d:w%d:%c] %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x", i, w,
		e->connection ? 'C' : ' ',
		e->data[ 0] & 0xFFu,
		e->data[ 1] & 0xFFu,
		e->data[ 2] & 0xFFu,
		e->data[ 3] & 0xFFu,
		e->data[ 4] & 0xFFu,
		e->data[ 5] & 0xFFu,
		e->data[ 6] & 0xFFu,
		e->data[ 7] & 0xFFu,
		e->data[ 8] & 0xFFu,
		e->data[ 9] & 0xFFu,
		e->data[10] & 0xFFu,
		e->data[11] & 0xFFu,
		e->data[12] & 0xFFu,
		e->data[13] & 0xFFu,
		e->data[14] & 0xFFu,
		e->data[15] & 0xFFu,
		e->data[16] & 0xFFu,
		e->data[17] & 0xFFu,
		e->data[18] & 0xFFu,
		e->data[19] & 0xFFu,
		e->data[20] & 0xFFu,
		e->data[21] & 0xFFu,
		e->data[22] & 0xFFu,
		e->data[23] & 0xFFu,
		e->data[24] & 0xFFu,
		e->data[25] & 0xFFu);
}

void equipment_show_data_receiver (cx_uint_t i, cx_uint_t w, equipment_receiver_t* e)
{
	debug_printf("[%2d:w%d:%c] %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x", i, w,
		e->connection ? 'C' : ' ',
		e->data[ 0] & 0xFFu,
		e->data[ 1] & 0xFFu,
		e->data[ 2] & 0xFFu,
		e->data[ 3] & 0xFFu,
		e->data[ 4] & 0xFFu,
		e->data[ 5] & 0xFFu,
		e->data[ 6] & 0xFFu,
		e->data[ 7] & 0xFFu,
		e->data[ 8] & 0xFFu,
		e->data[ 9] & 0xFFu,
		e->data[10] & 0xFFu,
		e->data[11] & 0xFFu,
		e->data[12] & 0xFFu,
		e->data[13] & 0xFFu,
		e->data[14] & 0xFFu,
		e->data[15] & 0xFFu,
		e->data[16] & 0xFFu,
		e->data[17] & 0xFFu,
		e->data[18] & 0xFFu,
		e->data[19] & 0xFFu,
		e->data[20] & 0xFFu,
		e->data[21] & 0xFFu,
		e->data[22] & 0xFFu,
		e->data[23] & 0xFFu,
		e->data[24] & 0xFFu,
		e->data[25] & 0xFFu);
}

void equipment_show_data_gathering_board (cx_uint_t i, cx_uint_t w, equipment_data_gathering_board_t* e)
{
	debug_printf("[%2d:w%d:%c] %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x", i, w,
		e->connection ? 'C' : ' ',
		e->data[ 0] & 0xFFu,
		e->data[ 1] & 0xFFu,
		e->data[ 2] & 0xFFu,
		e->data[ 3] & 0xFFu,
		e->data[ 4] & 0xFFu,
		e->data[ 5] & 0xFFu,
		e->data[ 6] & 0xFFu,
		e->data[ 7] & 0xFFu,
		e->data[ 8] & 0xFFu,
		e->data[ 9] & 0xFFu,
		e->data[10] & 0xFFu,
		e->data[11] & 0xFFu,
		e->data[12] & 0xFFu,
		e->data[13] & 0xFFu,
		e->data[14] & 0xFFu,
		e->data[15] & 0xFFu,
		e->data[16] & 0xFFu,
		e->data[17] & 0xFFu,
		e->data[18] & 0xFFu,
		e->data[19] & 0xFFu,
		e->data[20] & 0xFFu,
		e->data[21] & 0xFFu,
		e->data[22] & 0xFFu,
		e->data[23] & 0xFFu,
		e->data[24] & 0xFFu,
		e->data[25] & 0xFFu);
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void equipment_show (void)
{
	equipment_transmitter_t*			transmitter;
	equipment_receiver_t*				receiver;
	equipment_data_gathering_board_t*	data_gathering_board;

	cx_uint_t i;

	debug_printf("# EQUIPMENT DATA\n");
	
	debug_printf("\tTRANSMITTER\n");
	for (i=0u; i<EQUIPMENT_TRANSMITTER_MAX_COUNT; i++)
	{
		debug_printf("\t");

		transmitter = &_equipment.transmitter[i][0];
		equipment_show_data_transmitter(i, 1u, transmitter);

		debug_printf(" / ");
		
		debug_printf("\n");
		debug_printf("\t");
		
		transmitter = &_equipment.transmitter[i][1];
		equipment_show_data_transmitter(i, 2u, transmitter);

		debug_printf("\n");
		debug_flush();
	}

	debug_printf("\tRECEIVER\n");
	for (i=0u; i<EQUIPMENT_RECEIVER_MAX_COUNT; i++)
	{
		debug_printf("\t");

		receiver = &_equipment.receiver[i][0];
		equipment_show_data_receiver(i, 1u, receiver);

//		debug_printf(" / ");

		debug_printf("\n");
		debug_flush();
	}

	debug_printf("\tDATA GATHERING BOARD\n");
	for (i=0u; i<EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT; i++)
	{
		debug_printf("\t");

		data_gathering_board = &_equipment.data_gathering_board[i][0];
		equipment_show_data_gathering_board(i, 1u, data_gathering_board);

//		debug_printf(" / ");

		debug_printf("\n");
		debug_flush();
	}

	//===========================================================================
}




