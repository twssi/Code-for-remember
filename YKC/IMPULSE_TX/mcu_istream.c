/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "mcu.h"


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
istream_t _istream1;

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t istream_initialize (istream_t* ctx, cx_uint_t id, cx_byte_t* bsb_buffer_pointer, cx_uint_t bsb_buffer_size, cx_uint_t timeout_maxcount)
{
	if (0==bsb_initialize (&(ctx->bsb), bsb_buffer_pointer, bsb_buffer_size))
	{
		return CX_FALSE;
	}

	ctx->id = id;

	ctx->rx_flag    	= CX_FALSE;
	ctx->timer_flag 	= CX_FALSE;

	ctx->timeout          = CX_TRUE;
	ctx->timeout_count    = timeout_maxcount;
	ctx->timeout_maxcount = timeout_maxcount;

	return CX_TRUE;
}

void istream_set_rx_flag (istream_t* ctx, cx_bool_t rx_flag)
{
	ctx->rx_flag = rx_flag;

	if (CX_TRUE==rx_flag)
	{
		istream_set_timeout(ctx, CX_FALSE);
	}
}

void istream_set_timeout (istream_t* ctx, cx_bool_t timeout)
{
	if ( ctx->timeout!=timeout )
	{
		ctx->timeout=timeout;

		if (CX_TRUE==timeout)
		{
			debug_printf ("stream[%d] disconnected\n", ctx->id);
		}
		else
		{
			debug_printf ("stream[%d] connected\n", ctx->id);
		}
	}
}

cx_bool_t istream_get_timeout (istream_t* ctx)
{
	return ctx->timeout;
}

void istream_check_timeout (istream_t* ctx)
{
	if ( CX_FALSE==ctx->timer_flag )
	{
		return;
	}


	if (CX_FALSE==ctx->rx_flag)
	{
		if (ctx->timeout_count<ctx->timeout_maxcount)
		{
			ctx->timeout_count++;
		}
		else
		{
			istream_set_timeout(ctx, CX_TRUE);
		}
	}
	else
	{
		ctx->timeout_count = 0u;

		istream_set_timeout(ctx, CX_FALSE);

		ctx->rx_flag = CX_FALSE;
	}


	ctx->timer_flag = CX_FALSE;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void istream_timer_irq_handler (void)
{
	// 100msec
	_istream1.timer_flag = CX_TRUE;

	//======================================

}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t istream_packet_parse (istream_t* ctx, cx_byte_t* stream_pointer, cx_uint_t stream_size, cx_uint_t* stream_parsed_size)
{
	switch (ctx->id)
	{
	case 1u:
//	case 2u:
		return nb_packet_parse(stream_pointer, stream_size, stream_parsed_size);
		break;
	//===========================================================================

	default:
		break;
	}


	*stream_parsed_size = stream_size;

	return CX_FALSE;
}

cx_bool_t istream_packet_process_dust (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size)
{
	//cx_uint_t i;


#if 0
	debug_printf ("[%d] X:%d ", ctx->id, size);
	for (i=0u; i<size; i++)
	{
		debug_printf ("%02x ", (*(pointer+i)) & 0xFFu );
	}
	debug_printf ("\n");
#endif
	
	return CX_TRUE;
}

cx_bool_t istream_packet_process (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size)
{

#if 1
	nb_packet_t packet;


	switch (ctx->id)
	{
	case 1u:
		if (CX_TRUE==nb_packet_get (pointer, size, &packet))
		{
//			debug_printf ("stream[%d] R:%d seq=%d src=%x ts=%d id=%d\n", ctx->id, size, packet.head.sequence, packet.head.source, packet.head.timestamp, packet.head.id);
//			debug_printf ("stream[%x] seq=%d \n", packet.head.source, packet.head.sequence);
			
			if ( EQUIPMENT_TYPE_TRANSMITTER==nb_packet_get_id_type(packet.head.source) )
			{
				peer_rx_packet(&packet);

				istream_set_rx_flag(ctx, CX_TRUE);
			}
			else if( EQUIPMENT_TYPE_RECEIVER==nb_packet_get_id_type(packet.head.source) )
			{
				peer_rx_packet(&packet);

				istream_set_rx_flag(ctx, CX_TRUE);
			}	
			else	
			{
				debug_printf ("Xstream[%d] R:%d seq=%d src=%x ts=%d id=%d\n", ctx->id, size, packet.head.sequence, packet.head.source, packet.head.timestamp, packet.head.id);
			}
		}
		break;

	case 2u:
		if (CX_TRUE==nb_packet_get (pointer, size, &packet))
		{
//			debug_printf ("stream[%d] R:%d seq=%d src=%x ts=%d id=%d\n", ctx->id, size, packet.head.sequence, packet.head.source, packet.head.timestamp, packet.head.id);
//			debug_printf ("stream[%x] seq=%d \n", packet.head.source, packet.head.sequence);
			
			if ( EQUIPMENT_TYPE_RECEIVER==nb_packet_get_id_type(packet.head.source) )
			{
//				peer_rx_nb_packet (&packet);
				peer_rx_packet(&packet);

				istream_set_rx_flag(ctx, CX_TRUE);
			}
			else
			{
				debug_printf ("Xstream[%d] R:%d seq=%d src=%x ts=%d id=%d\n", ctx->id, size, packet.head.sequence, packet.head.source, packet.head.timestamp, packet.head.id);
			}
		}
		
		break;
		
	//====================================================================================================================================			

	default:
		break;
	}
#endif

	return CX_TRUE;
}

cx_bool_t istream_packet_push (istream_t* ctx, cx_byte_t* pointer, cx_uint_t size)
{
	cx_byte_t* stream_pointer     ;
	cx_uint_t  stream_size        ;
	cx_uint_t  stream_parsed_size ;
	cx_bool_t  stream_parsed      ;

	cx_bool_t result;


	result = CX_FALSE;


	if (size > 0u)
	{
		bsb_push(&(ctx->bsb), pointer, size);

		stream_pointer = bsb_get_pointer(&ctx->bsb);

		while ( (stream_size = bsb_get_size(&(ctx->bsb))) > 0u )
		{
			stream_parsed = istream_packet_parse (ctx, stream_pointer, stream_size, &stream_parsed_size);

			if (CX_TRUE==stream_parsed)
			{
				if (CX_TRUE==istream_packet_process(ctx, stream_pointer, stream_parsed_size))
				{
					result = CX_TRUE;
				}
			}
			else
			{
				if (0u<stream_parsed_size)
				{
					istream_packet_process_dust (ctx, stream_pointer, stream_parsed_size);
				}
			}


			if (0u==stream_parsed_size)
			{
				break;
			}


			bsb_erase (&(ctx->bsb), stream_parsed_size);
		}
	}


	return result;
}

