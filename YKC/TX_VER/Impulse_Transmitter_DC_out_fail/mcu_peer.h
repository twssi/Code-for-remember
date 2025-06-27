#ifndef INCLUDED__MCU_PEER__H
#define INCLUDED__MCU_PEER__H


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define PEER_CONNECTION_MAX_COUNT EQUIPMENT_MAX_COUNT

#define PEER_CONNECTION_INDEX_B_TRANSMITTER 	(0u    																		)
#define PEER_CONNECTION_INDEX_E_TRANSMITTER 	(PEER_CONNECTION_INDEX_B_TRANSMITTER	+EQUIPMENT_TRANSMITTER_MAX_COUNT	)

#define PEER_CONNECTION_INDEX_B_RECEIVER 		(PEER_CONNECTION_INDEX_E_TRANSMITTER                                		)
#define PEER_CONNECTION_INDEX_E_RECEIVER 		(PEER_CONNECTION_INDEX_B_RECEIVER		+EQUIPMENT_RECEIVER_MAX_COUNT		)

#define PEER_CONNECTION_INDEX_B_DATA_GATHERING_BOARD (PEER_CONNECTION_INDEX_E_RECEIVER                                		)
#define PEER_CONNECTION_INDEX_E_DATA_GATHERING_BOARD (PEER_CONNECTION_INDEX_B_DATA_GATHERING_BOARD+EQUIPMENT_DATA_GATHERING_BOARD_MAX_COUNT)

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _peer_connection_t
{
	cx_uint_t id;

	cx_uint_t rx_sequence;
	cx_bool_t rx_flag;
	cx_bool_t timer_flag;

	cx_bool_t timeout         ;
	cx_uint_t timeout_count   ;
	cx_uint_t timeout_maxcount;
}
peer_connection_t;

typedef struct _peer_connection_system_t
{
	cx_uint_t type;
	cx_uint_t unit;

	peer_connection_t worker1;
	peer_connection_t worker2;
}
peer_connection_system_t;


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API peer_connection_system_t _peer_connection [PEER_CONNECTION_MAX_COUNT];



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void      peer_connection_initialize    (peer_connection_t* ctx, cx_uint_t id, cx_uint_t timeout_maxcount);
MCU_API void      peer_connection_set_rx_flag   (peer_connection_t* ctx, cx_bool_t rx_flag);
MCU_API void      peer_connection_set_timeout   (peer_connection_t* ctx, cx_bool_t timeout);
MCU_API cx_bool_t peer_connection_get_timeout   (peer_connection_t* ctx);
MCU_API void      peer_connection_check_timeout (peer_connection_t* ctx);



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void peer_timer_irq_handler (void);

//===========================================================================
MCU_API peer_connection_system_t* peer_find_connection (cx_uint16_t id);

//===========================================================================
MCU_API void peer_initialize (void);
MCU_API void peer_update     (void);

MCU_API void peer_rx_packet 	(nb_packet_t* packet);
MCU_API void peer_set_data 		(nb_packet_t* packet);
MCU_API void peer_reset_data	(cx_uint_t type, cx_uint_t unit, cx_uint_t worker);

MCU_API void peer_show (void);


#endif




