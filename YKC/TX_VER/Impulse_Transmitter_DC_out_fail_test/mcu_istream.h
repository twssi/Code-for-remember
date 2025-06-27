#ifndef INCLUDED__MCU_ISTREAM__H
#define INCLUDED__MCU_ISTREAM__H



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _istream_t
{
	bsb_t bsb;

	cx_uint_t id;

	cx_bool_t rx_flag;
	cx_bool_t timer_flag;

	cx_bool_t timeout         ;
	cx_uint_t timeout_count   ;
	cx_uint_t timeout_maxcount;
}
istream_t;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API istream_t _istream1;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_bool_t istream_initialize    (istream_t* ctx, cx_uint_t id, cx_byte_t* bsb_buffer_pointer, cx_uint_t bsb_buffer_size, cx_uint_t timeout_maxcount);
MCU_API void      istream_set_rx_flag   (istream_t* ctx, cx_bool_t rx_flag);
MCU_API void      istream_set_timeout   (istream_t* ctx, cx_bool_t timeout);
MCU_API cx_bool_t istream_get_timeout   (istream_t* ctx);
MCU_API void      istream_check_timeout (istream_t* ctx);

MCU_API void istream_timer_irq_handler (void);

MCU_API cx_bool_t istream_packet_parse        (istream_t* ctx, cx_byte_t* stream_pointer, cx_uint_t stream_size, cx_uint_t* stream_parsed_size);
MCU_API cx_bool_t istream_packet_process_dust (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size);
MCU_API cx_bool_t istream_packet_process      (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size);
MCU_API cx_bool_t istream_packet_push         (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size);


#endif




