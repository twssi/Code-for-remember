#ifndef INCLUDED__MCU_EQUIPMENT__H
#define INCLUDED__MCU_EQUIPMENT__H

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define EQUIPMENT_TYPE_NONE      0u

#define EQUIPMENT_TYPE_TRANSMITTER 			((0u<<1u) | (1u)) 
#define EQUIPMENT_TYPE_RECEIVER 			((1u<<1u) | (0u))
#define EQUIPMENT_TYPE_DATA_GATHERING_BOARD ((1u<<1u) | (1u))

#define EQUIPMENT_TRANSMITTER_MAX_COUNT 			1u
#define EQUIPMENT_RECEIVER_MAX_COUNT   				1u
#define EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT   	1u

#define EQUIPMENT_MAX_COUNT (EQUIPMENT_TRANSMITTER_MAX_COUNT+EQUIPMENT_RECEIVER_MAX_COUNT+EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT)	

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _equipment_transmitter_t
{
	cx_bool_t connection;

	cx_uint_t sequence;

	cx_uint_t timestamp1;
	cx_uint_t timestamp2;
	cx_bool_t timestamp_stuck;

	cx_byte_t data[26];
}
equipment_transmitter_t;

typedef struct _equipment_receiver_t
{
	cx_bool_t connection;

	cx_uint_t sequence;

	cx_uint_t timestamp1;
	cx_uint_t timestamp2;
	cx_bool_t timestamp_stuck;

	cx_byte_t data[26];
}
equipment_receiver_t;

typedef struct _equipment_data_gathering_board_t
{
	cx_bool_t connection;

	cx_uint_t sequence;

	cx_uint_t timestamp1;
	cx_uint_t timestamp2;
	cx_bool_t timestamp_stuck;

	cx_byte_t data[26];
}
equipment_data_gathering_board_t;


typedef struct _equipment_t
{
	equipment_transmitter_t				transmitter			[EQUIPMENT_TRANSMITTER_MAX_COUNT	][2];
	equipment_receiver_t				receiver			[EQUIPMENT_RECEIVER_MAX_COUNT		][1];
	equipment_data_gathering_board_t	data_gathering_board[EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT][1];
}
equipment_t;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API equipment_t _equipment;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void equipment_initialize (void);

//===========================================================================
MCU_API cx_char_t* equipment_get_type_string (cx_uint_t v);

//===========================================================================
MCU_API void equipment_reset_data (cx_uint_t type, cx_uint_t unit, cx_uint_t worker);
MCU_API void equipment_set_data   (cx_uint_t type, cx_uint_t unit, cx_uint_t worker, 
                                   cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, 
                                   cx_byte_t* data_pointer, cx_uint_t data_size);

MCU_API void equipment_set_connection(cx_uint_t type, cx_uint_t unit, cx_uint_t worker, cx_bool_t connection);
MCU_API void equipment_update (void);
MCU_API void equipment_show   (void);

//===========================================================================
MCU_API void equipment_set_data_transmitter 			(cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, cx_byte_t* data_pointer, cx_uint_t data_size, equipment_transmitter_t* e);
MCU_API void equipment_set_data_receiver   				(cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, cx_byte_t* data_pointer, cx_uint_t data_size, equipment_receiver_t*   e);
MCU_API void equipment_set_data_gathering_board			(cx_uint_t data_sequence, cx_uint_t data_id, cx_uint_t data_timestamp, cx_byte_t* data_pointer, cx_uint_t data_size, equipment_data_gathering_board_t*   e);

MCU_API void equipment_reset_data_transmitter 			(equipment_transmitter_t* 		e);
MCU_API void equipment_reset_data_receiver   			(equipment_receiver_t*			e);
MCU_API void equipment_reset_data_gathering_board		(equipment_data_gathering_board_t* 	e);

void equipment_show_data_transmitter 					(cx_uint_t i, cx_uint_t w, equipment_transmitter_t* e);
void equipment_show_data_receiver 						(cx_uint_t i, cx_uint_t w, equipment_receiver_t* e);
void equipment_show_data_gathering_board 				(cx_uint_t i, cx_uint_t w, equipment_data_gathering_board_t* e);

MCU_API void equipment_update_transmitter 				(equipment_transmitter_t* e);
MCU_API void equipment_update_receiver 					(equipment_receiver_t* e);
MCU_API void equipment_update_data_gathering_board 		(equipment_data_gathering_board_t* e);


#endif




