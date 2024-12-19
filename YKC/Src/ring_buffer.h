/*
 * ring_buffer.h
 *
 *  Created on: Feb 14, 2024
 *      Author: 송현일
 *
 */

#ifndef SRC_RING_BUFFER_H_
#define SRC_RING_BUFFER_H_


#define UART_RX_BUFFER_SIZE 1024

#endif /* SRC_RING_BUFFER_H_ */
#include "type.h"
typedef cx_uint_t RINGBUF_SIZE;



typedef struct{


	RINGBUF_SIZE size     ;
	RINGBUF_SIZE size_mask;

	volatile RINGBUF_SIZE w_pointer;
	volatile RINGBUF_SIZE r_pointer;

	cx_uint8_t* buf;


}uart_hal_rx_type;




void ringbuf_init (uart_hal_rx_type* ringbuf, cx_uint8_t* memory, RINGBUF_SIZE memory_size);
void ringbuf_reset (uart_hal_rx_type* ringbuf);
RINGBUF_SIZE ringbuf_get_readable_space (uart_hal_rx_type* ringbuf);
RINGBUF_SIZE ringbuf_get_writable_space (uart_hal_rx_type* ringbuf);
RINGBUF_SIZE ringbuf_peek (uart_hal_rx_type* ringbuf, cx_uint8_t* buf, RINGBUF_SIZE size);
RINGBUF_SIZE ringbuf_read (uart_hal_rx_type* ringbuf, cx_uint8_t* buf, RINGBUF_SIZE size);
RINGBUF_SIZE ringbuf_write (uart_hal_rx_type* ringbuf, const cx_uint8_t* buf, RINGBUF_SIZE size);
