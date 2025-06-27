#ifndef INCLUDED__MCU_NB_PACKET__H
#define INCLUDED__MCU_NB_PACKET__H


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define NB_PEER_MAX_COUNT 	(EQUIPMENT_MAX_COUNT*2u)

#define NB_O_MESSAGE_MAX_COUNT 2
#define NB_O_MESSAGE_MAX_SIZE  (6+26)	//(SP, ID, timestamp) + PAYLOAD

#define NB_PACKET_TX_MAX_SIZE  (NB_O_MESSAGE_MAX_SIZE-6+16)	
#define NB_PACKET_RX_MAX_SIZE  (NB_PACKET_TX_MAX_SIZE*NB_PEER_MAX_COUNT)

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
// 14 bytes
typedef struct _nb_packet_head_t
{
	cx_uint8_t  mark       ; // 0xAA
	cx_uint8_t  version    ; // 0x01
	cx_uint16_t length     ;

	cx_uint16_t source     ;

	cx_uint16_t sequence   ;
	
	cx_uint8_t sp		;
	
    cx_uint8_t id         ;
	cx_uint32_t timestamp  ;
}
nb_packet_head_t;

// 2 bytes
typedef struct _nb_packet_tail_t
{
	cx_uint16_t crc;
}
nb_packet_tail_t;

// n bytes
typedef struct _nb_packet_body_t
{
	cx_byte_t* pointer;
	cx_uint_t  size;
}
nb_packet_body_t;

//===========================================================================
typedef struct _nb_packet_t
{
	nb_packet_head_t head;
	nb_packet_body_t body;
	nb_packet_tail_t tail;
}
nb_packet_t;


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_bool_t nb_packet_parse (cx_byte_t* stream_pointer, cx_uint_t stream_size, cx_uint_t* stream_parsed_size);
MCU_API cx_bool_t nb_packet_get   (cx_byte_t* stream_pointer, cx_uint_t stream_size, nb_packet_t* packet);
MCU_API cx_uint_t nb_packet_make  (cx_byte_t* stream_pointer, cx_uint_t stream_size, nb_packet_t* packet);

MCU_API cx_bool_t nb_packet_check_sequence_range (cx_uint_t current, cx_uint_t next);

MCU_API cx_uint16_t nb_packet_make_id       (cx_uint_t type, cx_uint_t unit, cx_uint_t worker);	
MCU_API cx_uint_t   nb_packet_get_id_type   (cx_uint16_t id);
MCU_API cx_uint_t   nb_packet_get_id_unit   (cx_uint16_t id);
MCU_API cx_uint_t   nb_packet_get_id_worker (cx_uint16_t id);


#endif




