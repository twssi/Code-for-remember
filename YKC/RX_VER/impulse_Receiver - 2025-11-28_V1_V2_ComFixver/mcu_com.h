#ifndef INCLUDED__MCU_COM__H
#define INCLUDED__MCU_COM__H





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define COM_MAX_PORT 4u

#define COM1 0u
#define COM2 1u
#define COM3 2u
#define COM4 3u
#define COM5 4u





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API io_fifo_t _com_fifo[COM_MAX_PORT];





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void com_rx_irq_handler (cx_uint_t com_port);
MCU_API void com_tx_irq_handler (cx_uint_t com_port);
MCU_API void com_timer_irq_handler (void);

//===========================================================================
MCU_API cx_bool_t com_send_byte   (cx_uint_t com_port, cx_byte_t ch);
MCU_API cx_uint_t com_send_buffer (cx_uint_t com_port, cx_byte_t* pointer, cx_uint_t size);

MCU_API cx_bool_t com_recv_byte   (cx_uint_t com_port, cx_byte_t* ch);
MCU_API cx_uint_t com_recv_buffer (cx_uint_t com_port, cx_byte_t* pointer, cx_uint_t size);

//===========================================================================
MCU_API void com_flush       (cx_uint_t com_port);
MCU_API void com_check_error (cx_uint_t com_port);

//===========================================================================
MCU_API void com_initialize (void);





#endif




