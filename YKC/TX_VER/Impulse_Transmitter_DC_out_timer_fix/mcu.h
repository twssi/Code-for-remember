#ifndef INCLUDED__MCU__H
#define INCLUDED__MCU__H


//===========================================================================
#include "type.h"
#include "bfifo.h"
#include "bsb.h"
#include "checksum.h"
#include "message_queue.h"

//===========================================================================
#include "mcu_debug.h"
#include "mcu_com.h"

#include "mcu_equipment.h"
#include "mcu_config.h"

#include "mcu_nb_packet.h"
#include "mcu_istream.h"
#include "mcu_peer.h"

#include "mcu_control_analog.h"

#include "mcu_redundant.h"

//#include "mcu_io_control.h"
#include "stm32f10x_bit_define.h"
#include "mcu_application.h"
#define TEST_O 			GpioD->Bit.b8

#endif




