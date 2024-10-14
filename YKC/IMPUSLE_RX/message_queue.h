#ifndef INCLUDED__MESSAGE_QUEUE__H
#define INCLUDED__MESSAGE_QUEUE__H





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
typedef struct _message_t
{
	cx_uint_t  size;
	cx_byte_t* pointer;
}
message_t;





//===========================================================================
typedef struct _message_queue_t
{
	cx_uint_t message_max_count;
	cx_uint_t message_max_size ;

	message_t* message;

	cx_uint_t message_count ;
	cx_uint_t message_wcursor;
	cx_uint_t message_rcursor;
}
message_queue_t;





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_bool_t message_queue_initialize (message_queue_t* ctx, cx_uint_t message_max_count, cx_uint_t message_max_size, message_t* message, cx_byte_t* message_buffer);

//===========================================================================
MCU_API cx_uint_t message_queue_count (message_queue_t* ctx);
MCU_API void      message_queue_clear (message_queue_t* ctx);
MCU_API cx_bool_t message_queue_push  (message_queue_t* ctx, cx_byte_t* pointer, cx_uint_t size);
MCU_API cx_bool_t message_queue_pop   (message_queue_t* ctx, cx_byte_t* pointer, cx_uint_t* size);





#endif




