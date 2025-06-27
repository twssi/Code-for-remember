/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>

#include "mcu.h"





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static const cx_uint_t NB_PACKET_STREAM_MIN_SIZE = 16u; //14->16
static const cx_uint_t NB_PACKET_STREAM_MAX_SIZE = 42u;

////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t nb_packet_parse (cx_byte_t* stream_pointer, cx_uint_t stream_size, cx_uint_t* stream_parsed_size)
{
	//-----------------------------------------------------------------------
	cx_bool_t   result;
	cx_uint_t   offset;
	cx_uint_t	check;


	//-----------------------------------------------------------------------
	nb_packet_head_t head;
	nb_packet_tail_t tail;


	//-----------------------------------------------------------------------
	offset = 0u;
	result = CX_FALSE;



	//-----------------------------------------------------------------------
	if (stream_size<NB_PACKET_STREAM_MIN_SIZE)
	{
		goto need_more;
	}

	//-----------------------------------------------------------------------
	head.mark = stream_pointer[0];
	if (0xAAu!=head.mark)
	{
		goto error;
	}

	head.version = stream_pointer[1];
	if (0x01u!=head.version)
	{
		goto error;
	}

	head.length = ((stream_pointer[2]&0xFFu)<<8u) | (stream_pointer[3]&0xFFu);
	if (NB_PACKET_STREAM_MIN_SIZE>head.length)
	{
		goto error;
	}
	if (NB_PACKET_STREAM_MAX_SIZE<head.length)
	{
		goto error;
	}
	if (head.length>stream_size)
	{
		goto need_more;
	}

#if 0
	offset = head.length-4u;
	check  = calc_crc32_std(stream_pointer, offset);
	tail.crc = 
		((stream_pointer[offset+0]&0xFFu)<<24u) | 
		((stream_pointer[offset+1]&0xFFu)<<16u) | 
		((stream_pointer[offset+2]&0xFFu)<< 8u) | 
		((stream_pointer[offset+3]&0xFFu)     ) ;
#endif
    
	offset = head.length-2u;
	check  = calc_crc16_ccitt_std(stream_pointer, offset);
	tail.crc = 
		((stream_pointer[offset+0]&0xFFu)<< 8u) | 
		((stream_pointer[offset+1]&0xFFu)     ) ; 		
		
	
	if (check!=tail.crc)
	{
		goto error;
	}

	if (CX_NULL_POINTER!=stream_parsed_size)
	{
		*stream_parsed_size = head.length;
	}

	result = CX_TRUE;


	return result;


	//-----------------------------------------------------------------------
need_more:
	for (offset=0u; offset<stream_size; offset++)
	{
		if (0xAAu==(stream_pointer[offset]&0xFFu))
		{
			break;
		}
	}
	if (CX_NULL_POINTER!=stream_parsed_size)
	{
		*stream_parsed_size = offset;
	}

	return result;


	//-----------------------------------------------------------------------
error:
	for (offset=1u; offset<stream_size; offset++)
	{
		if (0xAAu==(stream_pointer[offset]&0xFFu))
		{
			break;
		}
	}
	if (CX_NULL_POINTER!=stream_parsed_size)
	{
		*stream_parsed_size = offset;
	}

	return result;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t nb_packet_get (cx_byte_t* stream_pointer, cx_uint_t stream_size, nb_packet_t* packet)
{
	//-----------------------------------------------------------------------
	cx_uint_t offset;


	offset = 0u;


	//-----------------------------------------------------------------------
#if 1
	if (stream_size<NB_PACKET_STREAM_MIN_SIZE)
	{
		return CX_FALSE;
	}
#endif

	//-----------------------------------------------------------------------
	packet->head.mark = stream_pointer[offset++];
#if 0
	if (0xAAu!=packet->head.mark)
	{
		return CX_FALSE;
	}
#endif


	packet->head.version = stream_pointer[offset++];
#if 0
	if (0x01u!=packet->head.version)
	{
		return CX_FALSE;
	}
#endif


	packet->head.length = ((stream_pointer[offset+0u]&0xFFu)<<8u) | (stream_pointer[offset+1u]&0xFFu);
#if 0
	if (NB_PACKET_STREAM_MIN_SIZE>packet->head.length)
	{
		return CX_FALSE;
	}
	if (NB_PACKET_STREAM_MAX_SIZE<packet->head.length)
	{
		return CX_FALSE;
	}
	if (packet->head.length!=stream_size)
	{
		return CX_FALSE;
	}
#endif
	offset+=2u;

	packet->head.source = ((stream_pointer[offset+0u]&0xFFu)<<8u) | (stream_pointer[offset+1u]&0xFFu);
	offset+=2u;


	packet->head.sequence = ((stream_pointer[offset+0u]&0xFFu)<<8u) | (stream_pointer[offset+1u]&0xFFu);
	offset+=2u;
	
	packet->head.sp = (stream_pointer[offset+0u]&0xFFu);
	offset+=1u;

	packet->head.id = (stream_pointer[offset+0u]&0xFFu);
	offset+=1u;


	packet->head.timestamp = 
		((stream_pointer[offset+0u]&0xFFu)<<24u) | 
		((stream_pointer[offset+1u]&0xFFu)<<16u) | 
		((stream_pointer[offset+2u]&0xFFu)<< 8u) | 
		((stream_pointer[offset+3u]&0xFFu)     ) ;
	offset+=4u;


	packet->body.pointer = &stream_pointer[offset];
	packet->body.size    = packet->head.length - offset - 2u;
	offset+=packet->body.size;


	packet->tail.crc = 
		((stream_pointer[offset+0]&0xFFu)<< 8u) | 
		((stream_pointer[offset+1]&0xFFu)     ) ;
	offset += 2u;
#if 0	
	packet->tail.crc = 
		((stream_pointer[offset+0]&0xFFu)<<24u) | 
		((stream_pointer[offset+1]&0xFFu)<<16u) | 
		((stream_pointer[offset+2]&0xFFu)<< 8u) | 
		((stream_pointer[offset+3]&0xFFu)     ) ;
	offset += 4u;
#endif

	return CX_TRUE;
}





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_uint_t nb_packet_make (cx_byte_t* stream_pointer, cx_uint_t stream_size, nb_packet_t* packet)
{
	cx_uint_t offset;
	cx_uint_t length;


	length = NB_PACKET_STREAM_MIN_SIZE+packet->body.size;

	if (stream_size<length)
	{
		return 0u;
	}

	if (NB_PACKET_STREAM_MAX_SIZE<length)
	{
		return 0u;
	}

	
	packet->head.mark    = 0xAAu;
	packet->head.version = 0x01u;
	packet->head.length  = (cx_uint16_t)length;


	offset = 0u;


	stream_pointer[offset++] = packet->head.mark;


	stream_pointer[offset++] = packet->head.version;


	stream_pointer[offset+0] = (packet->head.length >> 8u) & 0xFFu;
	stream_pointer[offset+1] = (packet->head.length      ) & 0xFFu;
	offset+=2u;
	
	stream_pointer[offset+0] = (packet->head.source >> 8u) & 0xFFu;
	stream_pointer[offset+1] = (packet->head.source      ) & 0xFFu;
	offset+=2u;
	

	stream_pointer[offset+0] = (packet->head.sequence >> 8u) & 0xFFu;
	stream_pointer[offset+1] = (packet->head.sequence      ) & 0xFFu;
	offset+=2u;
	
	stream_pointer[offset++] = packet->head.sp;
	
	stream_pointer[offset++] = packet->head.id;

	stream_pointer[offset+0] = (packet->head.timestamp >> 24u ) & 0xFFu;
	stream_pointer[offset+1] = (packet->head.timestamp >> 16u ) & 0xFFu;
	stream_pointer[offset+2] = (packet->head.timestamp >>  8u ) & 0xFFu;
	stream_pointer[offset+3] = (packet->head.timestamp        ) & 0xFFu;
	offset+=4u;


	if (packet->body.size>0u)
	{
		memcpy(&stream_pointer[offset], packet->body.pointer, packet->body.size);

		offset += packet->body.size;
	}
	
	packet->tail.crc = calc_crc16_ccitt_std(stream_pointer, offset);
	
	stream_pointer[offset+0] = (packet->tail.crc >>  8u ) & 0xFFu;
	stream_pointer[offset+1] = (packet->tail.crc        ) & 0xFFu;
	offset+=2u;
	
#if 0
	packet->tail.crc = calc_crc32_std(stream_pointer, offset);

	stream_pointer[offset+0] = (packet->tail.crc >> 24u ) & 0xFFu;
	stream_pointer[offset+1] = (packet->tail.crc >> 16u ) & 0xFFu;
	stream_pointer[offset+2] = (packet->tail.crc >>  8u ) & 0xFFu;
	stream_pointer[offset+3] = (packet->tail.crc        ) & 0xFFu;
	offset+=4u;
#endif
	return offset;
}




/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_bool_t nb_packet_check_sequence_range (cx_uint_t current, cx_uint_t next)
{
	if (current==0u)
	{
		return CX_TRUE;
	}
	if (next==0u)
	{
		return CX_TRUE;
	}
	if (current==next)
	{
		return CX_FALSE;
	}

	cx_uint_t sequence_begin;
	cx_uint_t sequence_end;


	sequence_begin = (current+ 1u) % 0x10000u;
	sequence_end   = (current+32u) % 0x10000u;
//	sequence_end   = (current+64u) % 0x10000u;

	// sequence범위가 1~0xFFFF일때
	if (current>sequence_begin)
	{
		sequence_begin++;
	}
	if (current>sequence_end)
	{
		sequence_end++;
	}

	if (sequence_begin < sequence_end)
	{
		if ( (sequence_begin<=next) && (next<=sequence_end)  )
		{
			return CX_TRUE;
		}
	}
	else
	{
		if (  sequence_begin<=next  )
		{
			return CX_TRUE;
		}
		if (  next<=sequence_end  )
		{
			return CX_TRUE;
		}
	}
	
	debug_printf("### seq error ###\n");
	return CX_FALSE;
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
cx_uint16_t nb_packet_make_id (cx_uint_t type, cx_uint_t unit, cx_uint_t worker)
{
	// 4 bit   type
	// 2 bit   worker
	// 8 bit   unit

	cx_uint16_t id;

	id  = 0u;

	id |= ( (type   & 0x000Fu) << 12u);
	id |= ( (worker & 0x0003u) << 8u );
	id |= ( (unit   & 0x00FFu) << 0u );
	
	return id;
}

//===========================================================================
cx_uint_t nb_packet_get_id_type (cx_uint16_t id)
{
	cx_uint_t v;

	v = (id&0xF000u) >> 12u;	//4bit

	return v;
}

cx_uint_t nb_packet_get_id_unit (cx_uint16_t id)
{
	cx_uint_t v;
	
	v = (id&0x000Fu) >> 0u;

	return v;
}

cx_uint_t nb_packet_get_id_worker (cx_uint16_t id)
{
	cx_uint_t v;
	
	v = (id&0x0300u) >> 8u;
	
	
	if (v&0x01) { return 1u; }
	if (v&0x02) { return 2u; }

	return 0u;
}
