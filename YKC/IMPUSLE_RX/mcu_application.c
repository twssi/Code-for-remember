/////////////////////////////////////////////////////////////////////////////
//
// Header Files
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f10x_init.h"
#include "stm32f10x_bit_define.h"

#include "mcu.h"
#include "ringbuf.h"
#include "mcu_gpio_alias.h"


/////////////////////////////////////////////////////////////////////////////
//
// Private Global Variables
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static __IO uint32_t TimingDelay;

static cx_uint_t _timer_count_0020_msec  	= 0u;
static cx_uint_t _timer_count_0100_msec  	= 0u;
static cx_uint_t _timer_count_0200_msec  	= 0u;
static cx_uint_t _timer_count_0500_msec  	= 0u;
static cx_uint_t _timer_count_1000_msec  	= 0u;
static cx_uint_t _timer_count_5000_msec  	= 0u;

static cx_uint_t _timer_count_transmit  			= 0u;

static cx_uint_t _mode_sw_timer_count_0100_msec 	= 0u;
static cx_uint_t _count_active_fnd_display		 	= 0u;
static cx_uint_t _watchdog_timer_count_0100_msec	= 0u;
static cx_uint_t _operating_time_second 			= 0u;
static cx_uint_t _TX1_rx_timeout_count             	= 0u;
static cx_uint_t _TX2_rx_timeout_count	 			= 0u;

static cx_bool_t _external_watchdog_clock_enabled 	= CX_TRUE; //
static cx_uint_t _external_watchdog_clock_output  	= 0u;

static cx_bool_t _worker_run 			 	= CX_TRUE; 
static cx_bool_t _timer_run 			 	= CX_FALSE; 
static cx_bool_t _watchdog_run          	= CX_FALSE; 
static cx_bool_t _watchdog_fail          	= CX_FALSE;
static cx_bool_t _inconsistency_fail        = CX_FALSE;
static cx_bool_t _false_relay_drop 			= CX_FALSE;
static cx_bool_t _false_relay_excitation 	= CX_FALSE;
static cx_bool_t _application_halt          = CX_FALSE;

static cx_bool_t _flag_control_watch		= CX_FALSE;
static cx_bool_t _flag_update_equipment 	= CX_FALSE;
static cx_bool_t _flag_update_ostream   	= CX_FALSE;
static cx_bool_t _flag_transmit         	= CX_FALSE;

static cx_bool_t _flag_get_impulse_voltage  = CX_FALSE;
static cx_bool_t _flag_get_relay_voltage 	= CX_FALSE;

//static cx_bool_t _flag_check_watchdog_input_data = CX_FALSE;
static cx_bool_t _flag_control_input    	= CX_FALSE;
static cx_uint_t _relay_input_value 		= 0u;
static cx_uint_t _trans_tab_input_value 	= 0u;
static cx_uint_t _flag_make_display_data	= CX_FALSE;
static cx_uint_t _flag_mode_sw_sequence	    = CX_FALSE;
static cx_uint_t _flag_active_mode_switch	= CX_FALSE;
static cx_uint_t _status_mode_switch		= CX_TRUE;
static cx_uint_t _display_sequence			= 1u;
static cx_uint_t _flag_active_display		= CX_TRUE;

static cx_bool_t _flag_rising_edge_freq   		= CX_FALSE; 	//timer3 rising edge(freq)/ for impulse voltage value
static cx_bool_t _flag_rising_edge_freq_relay 	= CX_FALSE;     //timer3 rising edge(freq)/ for relay voltage value

static cx_uint_t _pre_hotstandby 			= 0u;
static cx_uint_t _check_rising_edge_tim3	= 0u;
//===========================================================================
// reg -> bfifo -> bsb -> stream
//debug
static cx_byte_t _com1_fifo_tx_buffer      [4096];
static cx_byte_t _com1_fifo_rx_buffer      [  64];


static cx_byte_t Tranmitt_active_data_buf  [  18];

static cx_uint_t _state_trans1		= 0;
static cx_uint_t _state_trans2		= 0;




//To Tx 2
static cx_byte_t _com2_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com2_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com2_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer 두배사이즈보다 커야 함

//To D.G
static cx_byte_t _com3_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com3_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com3_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer 두배사이즈보다 커야 함

//To TX 1
static cx_byte_t _com4_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com4_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com4_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer 두배사이즈보다 커야 함

// static RINGBUF               Trans1_rxringbuf;
// static cx_uint8_t        Trans1_rxbuf[128];
// static const cx_uint32_t    Trans1_rxbufsize = 128;

// static RINGBUF               Trans2_rxringbuf;
// static cx_uint8_t         Trans2_rxbuf[128];
// static const cx_uint32_t    Trans2_rxbufsize = 128;

static cx_uint8_t         Trans1_rxbuf_debug[25];
static cx_uint8_t         Trans2_rxbuf_debug[25];
static cx_uint8_t 			_DG_tx_buffer[46];
static cx_bool_t _flag_Trans1_rx_done		= CX_FALSE;
static cx_bool_t _flag_Trans2_rx_done	 	= CX_FALSE;
static cx_uint_t _count_Trans1_rx	= 0;
static cx_uint_t _count_Trans2_rx	= 0;

//===========================================================================
static message_queue_t _nb_o_message_queue  ;
static message_t       _nb_o_message        [NB_O_MESSAGE_MAX_COUNT];
static cx_byte_t       _nb_o_message_buffer [NB_O_MESSAGE_MAX_COUNT*NB_O_MESSAGE_MAX_SIZE];

static cx_byte_t _nb_tx_buffer [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _nb_rx_buffer [NB_PACKET_RX_MAX_SIZE];

static cx_byte_t _message [NB_O_MESSAGE_MAX_SIZE];

//--------------fnd 표시 변수 선언-----------------------------------------
cx_bool_t _fnd0[5];
cx_bool_t _fnd1[5];

#define FND_CH_00(pos)  ((pos)&0x000f)<<8   // first line 
#define FND_CH_01(pos)  ((pos)&0x000f)<<12  // second line 

#define min(x,y) ((x)<(y))?(x):(y)

#define FND_CHAR_0     0x3f
#define FND_CHAR_1     0x06
#define FND_CHAR_2     0x5b
#define FND_CHAR_3     0x4f
#define FND_CHAR_4     0x66
#define FND_CHAR_5     0x6d
#define FND_CHAR_6     0x7d
#define FND_CHAR_7     0x27
#define FND_CHAR_8     0x7f
#define FND_CHAR_9     0x67
#define FND_CHAR_DOT   0x80

#define FND_CHAR_BLANK 0x00
#define FND_CHAR_NEG   0x40

#define FND_CHAR_H     0x76
#define FND_CHAR_A     0x77
#define FND_CHAR_V     0x3E

/////////////////////////////////////////////////////////////////////////////
//
// Global Variables
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================


/////////////////////////////////////////////////////////////////////////////
//
// IRQ Function
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void external_watchdog_clock_output_irq_handler (void)
{
	if (CX_TRUE==_external_watchdog_clock_enabled)
	{
		_external_watchdog_clock_output = (0u==_external_watchdog_clock_output)?1u:0u;
		GPIO_O_EXTERNAL_WATCHDOG_CLOCK (_external_watchdog_clock_output);
	}
}

static void timer_boot_irq_handler (void)
{
	//-----------------------------------------------------------------------
	_timer_count_1000_msec++;
	if (_timer_count_1000_msec>1000u*2)
	{
		_timer_count_1000_msec = 0u;

		//-------------------------------------------------------------------
		_operating_time_second++;
	}
}

static void watchdog_run_irq_handler (void)
{
	_watchdog_timer_count_0100_msec++;
	if (_watchdog_timer_count_0100_msec>100u*2)	
	{
		_watchdog_timer_count_0100_msec = 0u;

		external_watchdog_clock_output_irq_handler();
	}
}

static void timer_run_irq_handler(void)
{
	//-----------------------------------------------------------------------
	_timer_count_5000_msec++;
	if (_timer_count_5000_msec>5000u*2)
	{
		_timer_count_5000_msec = 0u;

		//-------------------------------------------------------------------		
		_flag_update_equipment = CX_TRUE;	//equipment		
	}
		

	//-----------------------------------------------------------------------
	_timer_count_1000_msec++;
	if (_timer_count_1000_msec>1000u*2)
	{
		_timer_count_1000_msec = 0u;

		//-------------------------------------------------------------------	
		_operating_time_second++;
		
		_flag_update_ostream = CX_TRUE;	
	}
		
	//-----------------------------------------------------------------------
	_timer_count_0500_msec++;
	if (_timer_count_0500_msec>500u*2)
	{
		_timer_count_0500_msec = 0u;
		
		//-------------------------------------------------------------------
//		_flag_check_watchdog_input_data = CX_TRUE;
	
		_flag_make_display_data = CX_TRUE;
	}
	
	//-----------------------------------------------------------------------
	_timer_count_0200_msec++;
	if (_timer_count_0200_msec>200u*2)
	{
		_timer_count_0200_msec = 0u;

		//-------------------------------------------------------------------		
		_flag_control_input = CX_TRUE;
	}

	//-----------------------------------------------------------------------
	_timer_count_0100_msec++;
	if (_timer_count_0100_msec>100u*2)
	{
		_timer_count_0100_msec = 0u;
		
		//-------------------------------------------------------------------
		peer_timer_irq_handler   ();	//peer
		istream_timer_irq_handler();	//stream
		
		_flag_control_watch = CX_TRUE;
	}
	
	//-----------------------------------------------------------------------
	_timer_count_0020_msec++;
	if (_timer_count_0020_msec>200u*2)
	{
		_timer_count_0020_msec = 0u;
		_TX1_rx_timeout_count++;
		_TX2_rx_timeout_count++;

		//---------------------------------------------------------------------
        _flag_transmit = CX_TRUE;
       
	}
    
	//-----------------------------------------------------------------------		
	if(CX_TRUE == _flag_active_mode_switch)
	{
		_mode_sw_timer_count_0100_msec++;
		if (_mode_sw_timer_count_0100_msec>100u*2)
		{
			_mode_sw_timer_count_0100_msec = 0u;

			//------------------------------------------------------------------
			_flag_mode_sw_sequence = CX_TRUE;
		}	
	}
	else _mode_sw_timer_count_0100_msec = 0u;
	
	_timer_count_transmit++;

	//-----------------------------------------------------------------------		
    _flag_get_relay_voltage =  CX_TRUE;
	
}

void timer_50usec_irq_handler (void)
{
	_flag_get_impulse_voltage = CX_TRUE;	//
}


void timer_500usec_irq_handler (void)
{		
	if (CX_TRUE==_watchdog_run) watchdog_run_irq_handler();
	
	if (CX_TRUE==_timer_run)
	{
		timer_run_irq_handler();
	}
	else
	{
		timer_boot_irq_handler();
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Global Function
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay_msec (__IO cx_uint_t msec)
{
 	TimingDelay = msec;

  	while(TimingDelay != 0);
	
}

void delay_sec (cx_uint_t sec)
{
	cx_uint_t n;
	cx_uint_t i;


	for(n=0u; n<sec; n++)
	{
		for(i=0u; i<1000u; i++) 
		{
			delay_msec(1); 
		}
	}
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


/* Private function prototypes -----------------------------------------------*/
void delay_msec (__IO cx_uint_t msec);

/* Private functions ---------------------------------------------------------*/
//===========================================================================

void rising_edge_flag_TIM3(void)
{
    _flag_rising_edge_freq  = CX_TRUE;  //for impulse voltage
    
    _flag_rising_edge_freq_relay = CX_TRUE; //for relay voltage
    
	//-----------------------------------------------------------------------
	active_flag_frequency_rising_edge();
	_timer_count_transmit=0u;	
	_check_rising_edge_tim3 = 0u;
}

//===========================================================================
void calculate_inputcapture_TIM3 (void)
{
	static cx_uint16_t _capture_number_TIM3	= 0u;
	static cx_uint16_t _readvalue1_TIM3		= 0u;
    static cx_uint16_t _readvalue2_TIM3		= 0u;
	
	cx_uint16_t capture_value			= 0u;
	cx_float32_t pulse_frequecy_TIM3 	= 0u;
		
	if(_capture_number_TIM3 == 0)
	{
		/* Get the Input Capture value */
		_readvalue1_TIM3 = TIM_GetCapture2(TIM3);
		
		_capture_number_TIM3 = 1;
	}
	else if(_capture_number_TIM3 == 1)
	{
		/* Get the Input Capture value */
		_readvalue2_TIM3 = TIM_GetCapture2(TIM3); 
			 
		/* Capture computation */
		if (_readvalue2_TIM3 > _readvalue1_TIM3)
		{
			capture_value = _readvalue2_TIM3 - _readvalue1_TIM3; 
		}
		else
		{
			capture_value = (0xFFFF - _readvalue1_TIM3) + _readvalue2_TIM3;	//0xFFFF : Timer3의 period
		}
		
		/* Frequency computation */ 
		pulse_frequecy_TIM3 = ((cx_float32_t)(72000000/2400) / capture_value);	//2400 : Timer3의 TIM_Prescaler
		
		put_pulse_frequency(pulse_frequecy_TIM3);
	
		_capture_number_TIM3 = 0;	
	}
}

//adc dma 
void measure_current_irq_handler (void)	// dma 측정할때마다 호출
{	
	//dma interrupt 
	//put_current_value();

	//-----------------------------------------------------------------------
}

//-----------------------------------------------------------------------
void control_impulse_voltage_input(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_get_impulse_voltage)	//50us, 0.05ms
	{
		return;
	}
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_rising_edge_freq)		//rising edge
	{
		return;
	}
	//-----------------------------------------------------------------------
	static cx_uint_t _count_measure_impulse_voltage  = 0u;
	static cx_bool_t _active_measure_impulse_voltage = CX_FALSE;
    
	
	if(_count_measure_impulse_voltage >= 1)    //100us
    {
        GPIO_O_IMPULSE_CLR1(0); 	//정펄스 Clear
    } 
	
	if(_count_measure_impulse_voltage >= 40)   //rx: 40(2ms)     tx: 50(2.5ms)
    {
        GPIO_O_IMPULSE_CLR2(0);     //부펄스 Clear
    } 
	
    
    //-----------------------------------------------------------------------
    if(_count_measure_impulse_voltage>200)	//0.05ms * 200 = 10ms
	{		
		_active_measure_impulse_voltage = CX_TRUE;
		
		GPIO_O_IMPULSE_CLR1(1); 	
		GPIO_O_IMPULSE_CLR2(1);			
	}
	else 
	{
		_count_measure_impulse_voltage++;
		
		_active_measure_impulse_voltage = CX_FALSE;
	}
	
	get_input_data_impulse_voltage(_active_measure_impulse_voltage, _count_measure_impulse_voltage);
	
	//-----------------------------------------------------------------------
	if(_active_measure_impulse_voltage == CX_TRUE)
	{
		_count_measure_impulse_voltage = 0u;
		
		_flag_rising_edge_freq 		= CX_FALSE;
	}	
    
	_flag_get_impulse_voltage 	= CX_FALSE;
	//-----------------------------------------------------------------------
	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////  PWS modify//////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OnUart3_Recv (cx_uint8_t ch)
{
	static cx_uint32_t i=0;
	static cx_uint8_t buf[25];
//	ringbuf_write (&Trans1_rxringbuf, &ch, 1);
	
	buf[i] = ch;
	if(buf[i++] == 0x0A)
	{
		memcpy(Trans1_rxbuf_debug,buf,i);
		_flag_Trans1_rx_done = CX_TRUE;
		memset(buf,0x00,25);
		_count_Trans1_rx=i;
		i=0;
	}
	if(i>24) i=0;
}

void OnUart4_Recv (cx_uint8_t ch)
{
	static cx_uint32_t i=0;
	static cx_uint8_t buf[50];
//	ringbuf_write (&Trans2_rxringbuf, &ch, 1);
	
	buf[i] = ch;
	if(buf[i++] == 0x0A)
	{
		memcpy(Trans2_rxbuf_debug,buf,i);
		_flag_Trans2_rx_done = CX_TRUE;
		memset(buf,0x00,25);
		_count_Trans2_rx=i;
		i=0;
	}
	if(i>50) i=0;
}

uint8_t fcs(uint8_t* data, cx_uint16_t len)
{
	cx_uint16_t i;
	uint8_t ch;


	ch = 0;
	for (i = 0; i < len; i++)
	{
		ch += data[i];
	}
	
	ch |= 0x40;

	return ch;
}
uint8_t fcs_rx(uint8_t* data, cx_uint16_t len)
{
	cx_uint16_t i;
	uint8_t ch;


	ch = 0;
	for (i = 0; i < len; i++)
	{
		ch += data[i];
	}
	
	ch += 0x20;

	return ch;
}

uint8_t fcs_conv(uint8_t data)
{
	uint8_t value;
	uint8_t ch;

	ch = data&0x0F;
	//value = data;
	if(data>9)
	{
		value = 'A'+(ch%10);
	}
	else
	{
		value = '0'+ch;
	}		
	
	return value;
}

cx_uint16_t POW(cx_uint16_t a, cx_uint16_t b)
{
	cx_uint16_t i, pow = 1;
	for (i = 0; i < b; i++)
	
	pow *= a; 
	return pow;
} 

void dbg_put_char(uint8_t dat) {      // tx flag check(polling)

  USART3->DR = (dat & (u16)0x01FF);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);     // tx interrupt
}

void put_str(cx_byte_t *str) {

	cx_uint16_t i;
	for(i=0;i<strlen(str);i++) {
		dbg_put_char(str[i]);
	}
}
cx_uint8_t Packet_Verify (cx_uint8_t* bufptr, cx_uint32_t bufsize, cx_uint32_t max, cx_uint32_t* packetsize)
{
	cx_uint32_t  offset;
	cx_uint32_t  result;

	cx_uint8_t etx;

	cx_uint8_t chk;

	offset = 0;
	result = 0;


	if (bufsize>max)
	{
		goto cleanup;
	}

	etx = bufptr[bufsize-3];
	if (etx!=0x03)
	{
		offset+=2;
		goto cleanup;		
	}

	chk = fcs(&bufptr[bufsize-23], 18);
	
	if (bufptr[bufsize-5]!=fcs_conv(chk>>4))
	{
		offset+=2;
		goto cleanup;
	}
	if (bufptr[bufsize-4]!=fcs_conv(chk&0x0f))
	{
		offset+=2;
		goto cleanup;
	}
	
	offset = bufsize;
	result = 1;

cleanup:
	if (packetsize)
	{
		*packetsize = offset;
	}

	return result;
}

void Trans1_Input_Parse(cx_uint8_t * bufptr, cx_uint16_t bufsize)
{
	uint32_t volage_plus;
	_TX1_rx_timeout_count=0;
	volage_plus = ((bufptr[bufsize-17]-'0') *100) + ((bufptr[bufsize-16]-'0') * 10) + (bufptr[bufsize-15]-'0');
	if(volage_plus>=100)
	{
		memcpy(Tranmitt_active_data_buf,&bufptr[bufsize-23],18);
		memset(Trans1_rxbuf_debug,0x00, 25);
	}
}

void Trans2_Input_Parse(cx_uint8_t * bufptr, cx_uint16_t bufsize)
{
	uint32_t volage_plus;
	_TX2_rx_timeout_count=0;
	volage_plus = ((bufptr[bufsize-17]-'0') *100) + ((bufptr[bufsize-16]-'0') * 10) + (bufptr[bufsize-15]-'0');
	if(volage_plus>=100)
	{
		memcpy(Tranmitt_active_data_buf,&bufptr[bufsize-23],18);
		memset(Trans2_rxbuf_debug,0x00, 25);
	}
}

void Trans1_Input (void)
{
	if(CX_FALSE == _flag_Trans1_rx_done)
	{
		return;
	}
	const cx_uint32_t   max = 25;
	cx_uint32_t         size;
	cx_uint32_t         offset;
    size = _count_Trans1_rx;
	if (Packet_Verify(Trans1_rxbuf_debug, size, max, &offset))
	{
		Trans1_Input_Parse(Trans1_rxbuf_debug, offset);
	}
	_flag_Trans1_rx_done = CX_FALSE;
}

void Trans2_Input (void)
{
	if(CX_FALSE == _flag_Trans2_rx_done)
	{
		return;
	}
	const cx_uint32_t   max = 25;
	cx_uint32_t         size;
	cx_uint32_t         offset;

	size = _count_Trans2_rx; 
	if (Packet_Verify(Trans2_rxbuf_debug, size, max, &offset))
	{
		Trans2_Input_Parse(Trans2_rxbuf_debug, offset);
	}
	_flag_Trans2_rx_done = CX_FALSE;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////PWD modify end//////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------
void control_relay_voltage_input(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_get_relay_voltage)			//500us
	{
		return;
	}
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_rising_edge_freq_relay)	//rising edge
	{
		return;
	}
	//-----------------------------------------------------------------------
	static cx_uint_t _count_measure_relay_voltage  = 0u;
	static cx_bool_t _active_measure_relay_voltage = CX_FALSE;
	
	if(_count_measure_relay_voltage >= 660)    //500us * 660 = 330ms
    {
        _active_measure_relay_voltage = CX_TRUE;
    } 
	else 
    {
        _count_measure_relay_voltage++;
        
        _active_measure_relay_voltage = CX_FALSE;
    }

    //-----------------------------------------------------------------------
    get_input_data_relay_voltage( _active_measure_relay_voltage,  _count_measure_relay_voltage);

    //-----------------------------------------------------------------------
	if(_active_measure_relay_voltage == CX_TRUE)
	{
		_count_measure_relay_voltage = 0u;
		
		_flag_rising_edge_freq_relay = CX_FALSE;
	}	
    
	_flag_get_relay_voltage 	= CX_FALSE;
}

//-----------------------------------------------------------------------
void message_show (void)
{
	//-----------------------------------------------------------------------
	debug_printf("# MESSAGE\n");

	//-----------------------------------------------------------------------
	cx_byte_t*  payload;
	cx_uint8_t id;
	cx_uint32_t timestamp;
    
    memcpy(&id       , &_message[1], 1);
	memcpy(&timestamp, &_message[2], 4);
    payload = &_message[6];
	//-----------------------------------------------------------------------
}                                           

void io_control_show(void)
{
	debug_printf("# IO CONTROL\n");

//	debug_printf("\tGPIO INPUT: SwitchOver INPUT 1 = %d \n", GPIO_I_SwitchOver_INPUT_1());
//	debug_printf("\tGPIO INPUT: SwitchOver INPUT 2 = %d \n", GPIO_I_SwitchOver_INPUT_2());
													 
	debug_printf("\tGPIO INPUT: RELAY INPUT        = %d \n", (!GPIO_I_Track_Relay()));
													 
	debug_printf("\tGPIO INPUT: WATCHDOG INPUT     = %d \n", GPIO_I_EXTERNAL_WATCHDOG_INPUT());
													 
	debug_printf("\tGPIO INPUT: TRANS INPUT 1      = %d \n", (!GPIO_I_MCU_TRANS_1()));
	debug_printf("\tGPIO INPUT: TRANS INPUT 2      = %d \n", (!GPIO_I_MCU_TRANS_2()));
	debug_printf("\tGPIO INPUT: TRANS INPUT 3      = %d \n", (!GPIO_I_MCU_TRANS_3()));
	debug_printf("\tGPIO INPUT: TRANS INPUT 4      = %d \n", (!GPIO_I_MCU_TRANS_4()));
	
	debug_flush();
}
//-----------------------------------------------------------------------
cx_uint_t get_active_transmitter (void)
{
	return _pre_hotstandby;
}

//-----------------------------------------------------------------------
static void update_ostream (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_update_ostream)
	{
		return;
	}

	//-----------------------------------------------------------------------
	cx_byte_t message [NB_O_MESSAGE_MAX_SIZE];
	cx_uint_t message_size;
	cx_byte_t* payload;	
	//-----------------------------------------------------------------------
	cx_uint8_t	id;
	cx_uint32_t timestamp;
	//-----------------------------------------------------------------------
	static cx_byte_t _hotstandby = 0u;
	static cx_uint_t _count_simultaneous_active = 0u;
	
	cx_uint_t config_id;
	cx_uint_t input;
	cx_uint_t trans_tab;
	
	
	
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  relay_voltage_v1;
	cx_uint_t  relay_voltage_v2;
	cx_uint_t  rx_current;
	cx_uint_t  rx_frequency;
	
	
	cx_uint_t  manual_switch_input = CX_FALSE;
	cx_uint_t  simultaneous_activation = CX_FALSE;
	cx_uint_t  tx_fail_data = CX_FALSE;
	
	
	//-----------------------------------------------------------------------
	message_size = sizeof(message);
		
	memset (message, 0x00, message_size);
	
	payload = &message[6];  //
	//-----------------------------------------------------------------------			

	//-----------------------------------------------------------------------	
	if(_pre_hotstandby == _hotstandby);
	else 
	{
		_pre_hotstandby = _hotstandby;
		debug_printf("# ACTIVE Transmaitter = %d \n", _pre_hotstandby );
	}	
	
	if(_count_simultaneous_active >= 3) simultaneous_activation = CX_TRUE; 
	//-----------------------------------------------------------------------	
	//_hotstandby 있는 경우
	if(_hotstandby)
	{	
		memcpy(&payload[12], &_equipment.transmitter[0][_hotstandby-1].data, 14);	//
	}	
	
	//각 송신의 FAIL Byte, 절체스위치 입력정보 취합
	manual_switch_input = ((_equipment.transmitter[0][0].data[1])&0x40) | ((_equipment.transmitter[0][1].data[1])&0x80);
	
	//송신모듈 연결 여부를 확인하여 고장정보 비트를 추가	
	if(CX_TRUE == _equipment.transmitter[0][0].connection) 
	{
		tx_fail_data |= ((_equipment.transmitter[0][0].data[1])&0x03);
	}	
	else tx_fail_data |= 0x03;
	
	if(CX_TRUE == _equipment.transmitter[0][1].connection) 
	{
		tx_fail_data |= ((_equipment.transmitter[0][1].data[1])&0x0C);
	}	
	else tx_fail_data |= 0x0C; 
		
	payload[13] = (manual_switch_input|tx_fail_data);	//
	//-----------------------------------------------------------------------		
	config_id = _config.gpio_i_id;

	input =  
//		( GPIO_I_SwitchOver_INPUT_1()  ? 0x01u : 0x00u ) |	//절체 스위치 1 입력 정보
//		( GPIO_I_SwitchOver_INPUT_2()  ? 0x02u : 0x00u ) |	//절체 스위치 2 입력 정보
		( _application_halt 		   ? 0x00u : 0x01u ) |	//수신모듈 run/fail
		( _false_relay_drop 		   ? 0x00u : 0x02u ) |	//부정낙하 정상/고장
		( _false_relay_excitation 	   ? 0x00u : 0x04u ) |	//부정여자 정상/고장
		( simultaneous_activation 	   ? 0x08u : 0x00u ) |	//송신모듈 동시 주계
		//-----------------------------------------------------------------------	
		( _relay_input_value 		   ? 0x80u : 0x00u ) ;	//궤도계전기 입력 정보
		
	trans_tab = _trans_tab_input_value;	//
	
	input = (input | ((trans_tab&0x07)<<4));	//
	
	
	impulse_voltage_plus = get_imnpulse_voltage_plus_value();
	impulse_voltage_minus = get_imnpulse_voltage_minus_value();
	
	relay_voltage_v1 = get_relay_voltage_v1_value()/10;
	relay_voltage_v2 = get_relay_voltage_v2_value()/10;
	
	rx_current = get_rx_current_value()/10;
	rx_frequency = (cx_uint_t)(get_rx_3hz_frequency_value()*100);
	
	//-----------------------------------------------------------------------
	id = 0u;
	if (EQUIPMENT_TYPE_RECEIVER  ==_config.type) 
	{
		id = 2u;
	}
	timestamp = _operating_time_second;
		
	//-----------------------------------------------------------------------
    memcpy(&message[0], 0x00     , 1);
    memcpy(&message[1], &id       , 1);
	memcpy(&message[2], &timestamp, 4);

	//-----------------------------------------------------------------------
	payload[ 0] = config_id ;
	payload[ 1] = input     ;
	//정펄스
	payload[ 2] = ((impulse_voltage_plus &0xFF00)>>8);
	payload[ 3] = ( impulse_voltage_plus &0x00FF);
	//부펄스
	payload[ 4] = ((impulse_voltage_minus &0xFF00)>>8);
	payload[ 5] = ( impulse_voltage_minus &0x00FF);
	//V1
	payload[ 6] = relay_voltage_v1;
	//V2
	payload[ 7] = relay_voltage_v2;
	//트랜스 탭
//	payload[ 7] = trans_tab;	//이동
	//주파수
	payload[ 8] = ((rx_frequency &0xFF00)>>8);
	payload[ 9] = ( rx_frequency &0x00FF);
	//전류 
	payload[10] = ((rx_current &0xFF00)>>8);
	payload[11] = ( rx_current &0x00FF);

	//-----------------------------------------------------------------------
	memcpy(_message, message, sizeof(_message));

	//-----------------------------------------------------------------------
	message_queue_push(&_nb_o_message_queue, message, message_size);

	//-----------------------------------------------------------------------
	
	
	_flag_update_ostream = CX_FALSE;
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void transmit (void)
{
	//-----------------------------------------------------------------------
	if (_timer_count_transmit<380) //(_flag_transmit==CX_FALSE) 펄스 출력 후 190ms 마다 통신.. 시리얼 통신에 의한 ADC 입력 영향 방지
	{
		return;
	}

	//-----------------------------------------------------------------------
	cx_uint_t config_id;
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  relay_voltage_v1;
	cx_uint_t  relay_voltage_v2;
	cx_uint_t  rx_current;
	cx_uint_t  rx_frequency;
	
	cx_uint16_t temp_ReadID1 = 0u;
	cx_uint16_t temp_ReadID2 = 0u;
	
	cx_uint16_t rotary_sw_1 = 0u;
	cx_uint16_t rotary_sw_2 = 0u;
	
	cx_uint_t  i; // pws
	cx_uint8_t fcs_value; // pws
	cx_uint_t  size;  //pws
	cx_uint8_t relay_data;
	static cx_uint8_t _transmit_state_data;
	static cx_uint_t _count_relay_vol_off=0;
	
	temp_ReadID1 = GPIO_ReadInputData(GPIOE);
	temp_ReadID1 = ~(temp_ReadID1) & 0x3C;
	rotary_sw_1 = (temp_ReadID1 >> 2)&0x0F;  //SW 1 read   

	temp_ReadID2 = GPIO_ReadInputData(GPIOC);
	temp_ReadID2 = ~(temp_ReadID2) & 0x000F;		
	rotary_sw_2 = (temp_ReadID2 >> 0)&0x0F;	//SW 2 read
	rotary_sw_1 = reverse4(rotary_sw_1);	//reverse
	rotary_sw_2 = reverse4(rotary_sw_2);	//reverse
	
	config_id = _config.gpio_i_id;
	
	impulse_voltage_plus = get_imnpulse_voltage_plus_value();
	impulse_voltage_minus = get_imnpulse_voltage_minus_value();
	
	relay_voltage_v1 = get_relay_voltage_v1_value()/10;
	relay_voltage_v2 = get_relay_voltage_v2_value()/10;
	
	rx_current = get_rx_current_value()/10;
	rx_frequency = (cx_uint_t)(get_rx_3hz_frequency_value()*100);
	
	size=46;
	memset(&_DG_tx_buffer[4],0x30,18);
	_DG_tx_buffer[0] = 0x02;
	// for(i=0;i<2;i++)
	// {
		// _DG_tx_buffer[1+i]	=(config_id/POW(10,1-i))%10+'0';
	// }
	
	_DG_tx_buffer[1]	=fcs_conv(rotary_sw_1);
	_DG_tx_buffer[2]	=fcs_conv(rotary_sw_2);
	
	_state_trans1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
	_state_trans2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
	
	if(!_state_trans1)
	{
		_transmit_state_data = 0x31u;
	}
	else
	{
		if(_TX1_rx_timeout_count > 3)
		{
			if(_TX2_rx_timeout_count > 3) _transmit_state_data = 0x34;
			else _transmit_state_data = 0x33;
		}
		else
		{
			if(_TX2_rx_timeout_count > 3) _transmit_state_data = 0x32;
		}
	}
	
	if(!_state_trans2)
	{
		_transmit_state_data = 0x30;
	}

	_DG_tx_buffer[3] = _transmit_state_data;
	
	memcpy(&_DG_tx_buffer[4],Tranmitt_active_data_buf,18);
	memset(Tranmitt_active_data_buf, 0x30, 18);
	
	impulse_voltage_plus = impulse_voltage_plus/10;
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[22+i]	=(impulse_voltage_plus/POW(10,2-i))%10+'0';
	}
	
	impulse_voltage_minus = impulse_voltage_minus/10;
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[25+i]	=(impulse_voltage_minus/POW(10,2-i))%10+'0';
	}
	
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[28+i]	=(rx_frequency/POW(10,2-i))%10+'0';
	}
	rx_current/=10;
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[31+i]	=(rx_current/POW(10,2-i))%10+'0';
	}
	
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[34+i]	=(relay_voltage_v2/POW(10,2-i))%10+'0';
	}
	
	for(i=0;i<3;i++)
	{
		_DG_tx_buffer[37+i]	=(relay_voltage_v1/POW(10,2-i))%10+'0';
	}

	if(_relay_input_value == CX_TRUE)
	{
		relay_data = 0x31;
		_count_relay_vol_off = 0u;
	}
	else
	{
		
		if(relay_voltage_v1 >= 20 && relay_voltage_v2 >= 20)
		{
			
			{
				relay_data = 0x32;
			}
		}
		else
		{
			relay_data = 0x30;
			_count_relay_vol_off=0;
		}
		if(_TX1_rx_timeout_count > 5 && _TX2_rx_timeout_count > 5)
		{
			_TX1_rx_timeout_count = 10;
			_TX2_rx_timeout_count = 10;
			relay_data =0x34;
		}
		_count_relay_vol_off = 0u;
	}

	
	_DG_tx_buffer[40]=relay_data;
	fcs_value = fcs_rx(&_DG_tx_buffer[1], 40);
	_DG_tx_buffer[41]=fcs_conv(fcs_value>>4);
	_DG_tx_buffer[42]=fcs_conv(fcs_value&0x0f);
	_DG_tx_buffer[43]=0x03;
	_DG_tx_buffer[44]=0x0D;
	_DG_tx_buffer[45]=0x0A;
	
	// if ( CX_TRUE==_relay_input_value )  
	// {
	// 	if(_timer_count_transmit >=40 && _timer_count_transmit<=575)
	// 	{
	// 		put_str(_DG_tx_buffer);
	// 	}
	// }
	// else
	// {
	// 	put_str(_DG_tx_buffer);
	// 	_timer_count_transmit=0;
	// }
	
	//-----------------------------------------------------------------------
	put_str(_DG_tx_buffer);
	_timer_count_transmit = CX_FALSE;
	_flag_transmit=CX_FALSE;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void control_watch (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_watch)	//100ms 주기
	{
		return;
	}
	
	cx_uint_t fail_check_chattering_count = 30u;	//3초
	cx_uint_t relay_excitaion_condition;
	cx_uint_t relay_drop_condition;
	cx_uint_t get_relay_voltage_V1;
	cx_uint_t get_relay_voltage_V2;
	
	cx_bool_t fail_max_count = 15u;	//3초
		
	static cx_uint_t _fail_count_false_drop 	= 0u;
	static cx_uint_t _fail_count_false_excitation 	= 0u;
	//-----------------------------------------------------------------------
	if(_operating_time_second>=1)
	{
		_check_rising_edge_tim3++;
				
		if(_check_rising_edge_tim3>=fail_max_count)
		{
			clear_measured_data();
		}
		
		if(_check_rising_edge_tim3>=0xFF) _check_rising_edge_tim3 = fail_max_count+1;
	}	
	//-----------------------------------------------------------------------	
	cx_bool_t watchdog_fail;
	cx_bool_t watchdog_fail_max_count 		= 50u;
	static cx_uint_t watchdog_fail_count 	= 0u;

	watchdog_fail = CX_TRUE;
		
	if(GPIO_I_EXTERNAL_WATCHDOG_INPUT() == CX_TRUE)
	{
		watchdog_fail = CX_FALSE;
	}
	else watchdog_fail = CX_TRUE;
	
	
	if ( CX_FALSE==watchdog_fail )
	{
		_watchdog_fail = CX_FALSE;	
		
		watchdog_fail_count = 0u;
	}
	else
	{
		if (watchdog_fail_count<watchdog_fail_max_count)
		{
			watchdog_fail_count++;
		}
		else
		{
			_watchdog_fail = CX_TRUE;
		}
	}
	//-----------------------------------------------------------------------
	get_relay_voltage_V1  = get_relay_voltage_v1_value();
	get_relay_voltage_V2  = get_relay_voltage_v2_value();
	
	//계전기 여자되는 범위, V1전압 25/V2전압 35 이상
	if( (get_relay_voltage_V1 >= 250) && (get_relay_voltage_V2 >= 350) )	
	{
		relay_excitaion_condition = CX_TRUE;
	}	
	else relay_excitaion_condition = CX_FALSE;
	
	//계전기 낙하되는 범위, V1전압 10/V2전압 20 이하
	if( (get_relay_voltage_V1 < 100) && (get_relay_voltage_V2 < 200) )	
	{
		relay_drop_condition = CX_TRUE;
	}	
	else relay_drop_condition = CX_FALSE;
	
	
	//-----------------------------------------------------------------------
	//부정낙하.. 임펄스 전압 있는데 궤도계전기가 여자인 상태
	if( (relay_excitaion_condition == CX_TRUE) && (_relay_input_value == CX_FALSE) )
	{
		if (_fail_count_false_drop >= fail_check_chattering_count)
		{
			_false_relay_drop = CX_TRUE;
		}
		else
		{
			_fail_count_false_drop++;
		}
	}
	else 
	{
		_fail_count_false_drop = 0u;
		
		_false_relay_drop = CX_FALSE;
	}	
	
	//-----------------------------------------------------------------------
	//부정여자.. 임펄스 전압 없는데 궤도계전기가 여자인 상태
	if( (relay_drop_condition == CX_TRUE) && (_relay_input_value == CX_TRUE) )
	{
		if (_fail_count_false_excitation >= fail_check_chattering_count)
		{
			_false_relay_excitation = CX_TRUE;
		}
		else
		{
			_fail_count_false_excitation++;
		}
	}
	else
	{
		_fail_count_false_excitation = 0u;
		
		_false_relay_excitation = CX_FALSE;
	}	
	//-----------------------------------------------------------------------
	if( (CX_TRUE == _false_relay_drop) || (CX_TRUE == _false_relay_excitation) )
	{
		_inconsistency_fail = CX_TRUE;	//불일치 고장
	}	
	else 
	{
		_inconsistency_fail = CX_FALSE;
	}	
	//-----------------------------------------------------------------------
	_flag_control_watch = CX_FALSE;
}


static void control_input(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_input)	//50ms 주기
	{
		return;
	}
	//-----------------------------------------------------------------------
	cx_bool_t relay_input;
	cx_uint_t input_trans_tab;
	
	cx_uint_t input_chattering_count 	= 6u;
	static cx_uint_t _relay_input_count = 0u;

	//-----------------------------------------------------------------------
	relay_input = CX_FALSE;



    if( CX_FALSE == GPIO_I_Track_Relay())
	{
		relay_input = CX_TRUE;
	}
	
	if (CX_TRUE==relay_input)
	{
		if (_relay_input_count < input_chattering_count)
		{
			_relay_input_count++;
		}
		else
		{
			_relay_input_value = CX_TRUE;
		}
	}
	else
	{
		if (_relay_input_count>0u)
		{
			_relay_input_count--;
		}
		else
		{
			_relay_input_value = CX_FALSE;
		}
	}
	//-----------------------------------------------------------------------
	input_trans_tab = 0u;
	
	if		(GPIO_I_MCU_TRANS_1() == CX_FALSE) input_trans_tab = 1u;
	else if (GPIO_I_MCU_TRANS_2() == CX_FALSE) input_trans_tab = 2u;
	else if (GPIO_I_MCU_TRANS_3() == CX_FALSE) input_trans_tab = 3u;
	else if (GPIO_I_MCU_TRANS_4() == CX_FALSE) input_trans_tab = 4u;
	else;
	_trans_tab_input_value = input_trans_tab;
        
	//-----------------------------------------------------------------------
	_flag_control_input = CX_FALSE;
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================

void application_show_version (void)
{
	static cx_char_t* APPLICATION_NAME    = "Intergrated Impulse Circuit Device - Receiver";
	static cx_char_t* APPLICATION_VERSION = "0.0.0.1";


	debug_printf("\n");
	debug_printf("%s \n", APPLICATION_NAME);
	debug_printf("Version %s (compiled: %s, %s)\n", APPLICATION_VERSION, __DATE__, __TIME__);
	debug_printf("\n");
	debug_printf("Copyright (c) 2022, Yoo-Kyung Control co., ltd.\n");
	debug_printf("All rights reserved.\n");
	debug_printf("\n");
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void update_istream (void)//최신 업데이트 입력 버퍼
{
	peer_update();

	istream_check_timeout(&_istream2);
	istream_check_timeout(&_istream3);
	istream_check_timeout(&_istream4);
}

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void update_equipment (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_update_equipment)
	{
		return;
	}

	//-----------------------------------------------------------------------
	equipment_update();

	//-----------------------------------------------------------------------
	_flag_update_equipment = CX_FALSE;
}

static void reception (cx_bool_t enable)
{
	cx_uint_t rx_size;

	
	rx_size = com_recv_buffer(COM2, _nb_rx_buffer, sizeof(_nb_rx_buffer)); //Tx 2계
	if (rx_size > 0u)
	{
		if (CX_TRUE==enable)
		{
			istream_packet_push(&_istream2, _nb_rx_buffer, rx_size);
		}

	}

	rx_size = com_recv_buffer(COM4, _nb_rx_buffer, sizeof(_nb_rx_buffer));	//Tx 1계
	if (rx_size > 0u)
	{
		if (CX_TRUE==enable)
		{
			istream_packet_push(&_istream4, _nb_rx_buffer, rx_size);
		}

	}
#if 0
	rx_size = com_recv_buffer(COM3, _nb_rx_buffer, sizeof(_nb_rx_buffer));
	if (rx_size > 0u)
	{
		if (CX_TRUE==enable)
		{
			istream_packet_push(&_istream3, _nb_rx_buffer, rx_size);
		}

	}
#endif	

}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#if 0
static void check_watchdog_input_data (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_check_watchdog_input_data)
	{
		return;
	}
	//-----------------------------------------------------------------------
	cx_bool_t watchdog_fail;
	cx_bool_t fail_max_count 			 	= 5u;
	static cx_uint_t watchdog_fail_count 	= 0u;

	watchdog_fail = CX_TRUE;
	
	
	if(GPIO_I_EXTERNAL_WATCHDOG_INPUT() == CX_TRUE)
	{
		watchdog_fail = CX_FALSE;
	}
	else watchdog_fail = CX_TRUE;
	
	
	if ( CX_FALSE==watchdog_fail )
	{
		_watchdog_fail = CX_FALSE;	
		
		watchdog_fail_count = 0u;
	}
	else
	{
		if (watchdog_fail_count<fail_max_count)
		{
			watchdog_fail_count++;
		}
		else
		{
			_watchdog_fail = CX_TRUE;
		}
	}
	
	//-----------------------------------------------------------------------
	_flag_check_watchdog_input_data = CX_FALSE;
}
#endif
/////////////////////////////////////////////////////////////////////////////
//===========================================================================

//-----------------------------------------------------------------------
void display_maked_fnd_data (const char* numstring, cx_bool_t* bufptr, cx_uint_t bufsize)
{
	const cx_bool_t tbl[10] =
	{
		FND_CHAR_0,
		FND_CHAR_1,
		FND_CHAR_2,
		FND_CHAR_3,
		FND_CHAR_4,
		FND_CHAR_5,
		FND_CHAR_6,
		FND_CHAR_7,
		FND_CHAR_8,
		FND_CHAR_9
	};

	const cx_bool_t max = 6;

	cx_bool_t fnd[5];
	cx_bool_t dot;
	cx_bool_t ascii;
	
	cx_uint_t  i;
	cx_uint_t  position;
 
	cx_uint_t  len;
    len    = strlen(numstring);
    
	if(len == 5)
    {		
		position = 0;     	 //첫번째 자리
    }
	else if(len == 4)
    {
	 	dot    = *(numstring+1);
	 	if(dot == 0x2E)  position    = 1;	//두번째 자리
	 	else position = 0;     				//첫번째 자리
       
    }
    else if(len == 3)
    {
		position = 1;
    }
    else if(len == 2)
    {
	 	position = 2;
    }
    else if(len == 1)
    {
	 	position = 3;
    }
    else position = 0;
	//-----------------------------------------------------------------------
	
	fnd[0] = FND_CHAR_BLANK;
	fnd[1] = FND_CHAR_BLANK;
	fnd[2] = FND_CHAR_BLANK;
	fnd[3] = FND_CHAR_BLANK;
	fnd[4] = FND_CHAR_BLANK;	

	for (i=0; i<len && position<max; i++)
	{
		ascii = numstring[i];
		switch (ascii)
		{
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				fnd[position++] = tbl[ascii-'0'];
				break;
			case '-':
				fnd[position++] = FND_CHAR_NEG;
				break;
			case 'H':
				fnd[position++] = FND_CHAR_H;
				break;
			case 'V':
				fnd[position++] = FND_CHAR_V;
				break;	
			case 'A':
				fnd[position++] = FND_CHAR_A;
				break;		
			case '.':
				if (position>0)
				{
					fnd[position-1] |= FND_CHAR_DOT;
				}

				break;
			default:
				fnd[position++] = FND_CHAR_BLANK;
				break;
		}
	}
	//-----------------------------------------------------------------------
	if (bufptr!=0 && bufsize>0)
	{
		memcpy (bufptr, fnd, min(bufsize, max));
	}
}

//-----------------------------------------------------------------------
void fnd_write (cx_uint_t select, cx_uint8_t fndvalue)
{
	cx_uint16_t portd_value = 0; //FND9~FND15 //PD8~PE15
	cx_uint16_t porte_value = 0; //FND1~FND8  //PE7~PE14
	volatile cx_uint_t i=0;
	cx_uint16_t ReadValue_portD;
	cx_uint16_t ReadValue_portE;
	
	ReadValue_portD = GPIO_ReadInputData(GPIOD);   // 쓰기 전에 기존값 읽기 
	
	portd_value |= select;    //PD8~PD15
	GPIO_Write(GPIOD,(ReadValue_portD & 0x00FF) | portd_value);
	
	ReadValue_portE = GPIO_ReadInputData(GPIOE);   // 쓰기 전에 기존값 읽기 
	
	porte_value |= fndvalue<<7; //PE7~PE14
	GPIO_Write(GPIOE,(ReadValue_portE & 0x807F) | porte_value);
	
	for (i=0;i<0x10;i++);
//	GpioD->DATA = GpioD->DATA & 0xff00;  
	GPIO_Write(GPIOD,(ReadValue_portD & 0x00FF));	//FND9~FND15 clear
	
//	GpioE->DATA = GpioE->DATA & 0x807f;  
	GPIO_Write(GPIOE,(ReadValue_portE & 0x807F));	//FND0~FND8 clear

}

//-----------------------------------------------------------------------
void fnd_output_data(const cx_uint_t id, const cx_bool_t* bufptr, cx_uint_t bufsize)
{
	cx_uint_t  i;
        
	switch (id)
	{
		case  0: for (i=0;i<bufsize;i++) { fnd_write(FND_CH_00(1<<i),bufptr[i]); } break;
		case  1: for (i=0;i<bufsize;i++) { fnd_write(FND_CH_01(1<<i),bufptr[i]); } break;
	}

}

//-----------------------------------------------------------------------
void output_fnd_display(void)
{
	_flag_active_display = CX_TRUE;
		
	fnd_output_data (0, _fnd0, 4);
	fnd_output_data (1, _fnd1, 4);	
}

//-----------------------------------------------------------------------
void input_front_mode_button(void)
{	
	if(CX_TRUE == _status_mode_switch)
	{
		_flag_active_mode_switch = CX_TRUE;
		
		_status_mode_switch = CX_FALSE;
		
		_count_active_fnd_display = 0u;	//누를때마다 FND off를 위한 카운트 클리어, 시간지나서 off시키기 위한 변수
	}
}

void check_front_button(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE == _flag_mode_sw_sequence)	//front FND 버튼 누르는 경우
	{
		return; 
	}
	//-----------------------------------------------------------------------	
	static cx_uint_t _input_mode_sw = 0u;
	
	_flag_active_display = CX_TRUE;
	
	_input_mode_sw = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);
	if(_input_mode_sw == 1)
	{
		_status_mode_switch = CX_TRUE;
		_display_sequence++;
		if(_display_sequence>=5) _display_sequence = 1u;
		_flag_active_mode_switch = CX_FALSE;
	}	
	
	//-----------------------------------------------------------------------
	_flag_mode_sw_sequence = CX_FALSE;
}

void make_fnd_data (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_make_display_data)	//500ms 주기
	{
		return;
	}

	//-----------------------------------------------------------------------
	char first_line   [5]  = {0,};	
	char second_line  [5]  = {0,};	
//	cx_uint_t  fnd_active_max_count 	  = 100u;	
	cx_uint_t  fnd_change_max_count 	  = 10u;	//500ms*10 = 5초
	
	cx_uint_t  first_line_display_length  = 0u;
	cx_uint_t  second_line_display_length = 0u;
	
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  relay_voltage_v1;
	cx_uint_t  relay_voltage_v2;
	cx_uint_t  int_rx_current;

	cx_float_t rx_current;
	cx_float_t rx_frequency;
    cx_float_t fnd_rx_current;
	
	cx_uint_t  id;
	cx_uint_t  trans_tab;
	
	//-----------------------------------------------------------------------
	impulse_voltage_plus  = get_imnpulse_voltage_plus_value()/10;	//3자리
	impulse_voltage_minus = get_imnpulse_voltage_minus_value()/10;	//3자리
	
	relay_voltage_v1 = get_relay_voltage_v1_value()/10;			//3자리
	relay_voltage_v2 = get_relay_voltage_v2_value()/10;			//3자리
	
	rx_current 		= get_rx_current_value();
	int_rx_current  = (cx_uint_t)(rx_current/10);
	
	fnd_rx_current = (cx_float_t)int_rx_current/100;
	
	rx_frequency 	= get_rx_3hz_frequency_value();
	
	id = _config.gpio_i_id;
	
	trans_tab = _trans_tab_input_value;

	switch (_display_sequence)
	{
		case 1u:
		
			sprintf(first_line ,"%d", impulse_voltage_plus);
			first_line_display_length = strlen(first_line);
			first_line[first_line_display_length] = 'V';
			
			sprintf(second_line,"%d", impulse_voltage_minus);
			second_line_display_length = strlen(second_line);
			second_line[second_line_display_length] = 'V';
			
			break;

		case 2u:
			
			sprintf(first_line ,"%d", relay_voltage_v2);
			first_line_display_length = strlen(first_line);
			first_line[first_line_display_length] = 'V';
			
			sprintf(second_line,"%d", relay_voltage_v1);
			second_line_display_length = strlen(second_line);
			second_line[second_line_display_length] = 'V';
			
			break;
		case 3u:
		
			sprintf(first_line,"%d", id);
			first_line_display_length = strlen(first_line);
				
			sprintf(second_line,"%.2f",rx_frequency);
			second_line_display_length = strlen(second_line);
			second_line[second_line_display_length] = 'H';
		
			break;
			
		case 4u:
	
			sprintf(first_line,"%.2f",fnd_rx_current);
			first_line_display_length = strlen(first_line);
			first_line[first_line_display_length] = 'A';

			sprintf(second_line,"%d",trans_tab);
			second_line_display_length = strlen(second_line);
			
			break;

		default:
			break;
	}
	
	//-----------------------------------------------------------------------
	
	display_maked_fnd_data   (first_line,   _fnd0, 5); 		
	display_maked_fnd_data   (second_line,  _fnd1, 5); 
	
	//-----------------------------------------------------------------------
	
	if(CX_TRUE == _flag_active_display)
	{
		if(_display_sequence != 1u)	_count_active_fnd_display++;
		else _count_active_fnd_display = 0u;

		if (_count_active_fnd_display > fnd_change_max_count)	//5초 후  
		{
			_display_sequence = 1u;
		}

#if 0
		//시간 지나면 FND OFF시킬 때 사용	
		if (_count_active_fnd_display > fnd_active_max_count)	
		{
			_count_active_fnd_display = 0;
			_flag_active_display = CX_FALSE;
			
			if( _count_active_fnd_display >= 0xFFFF) _count_active_fnd_display = (fnd_active_max_count+1);
			
			_display_sequence 			= 0u;
			_flag_active_mode_switch 	= 0u;
			_status_mode_switch 		= CX_TRUE;
		}
#endif		
	}
	else _count_active_fnd_display = 0u;
	
	if(_display_sequence == 0u) _display_sequence = 1u;
	
	//-----------------------------------------------------------------------
	_flag_make_display_data = CX_FALSE;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void check_fault (void)
{		
//---------------------------------------------------------------------------
// TODO: RELEASE version
//---------------------------------------------------------------------------		
	_application_halt = CX_FALSE;	
    
    // GO HALT?	
	if(CX_TRUE == _watchdog_fail)
	{
		_application_halt = CX_TRUE;	
	}	

	if(_operating_time_second>3) //run 이후 3초 이후 체크
	{
#if 0		
		//점유되면 주파수도 안들어오나?? 0옴은 안들어오네..fail처리 할 필요 없을듯
		if (CX_FALSE==_analog_data.pulse_frequency.health)	
		{
//			_application_halt = CX_TRUE;	 
		}
#endif		
		if(CX_TRUE == _inconsistency_fail)
		{
			_application_halt = CX_TRUE;	
		}		
	}
	//---------------------------------------------------------------------------
	//RUN,FAIL LED 제어
	if(_application_halt == CX_TRUE)	GPIO_O_CPU_LED_STATUS(0);
	else 								GPIO_O_CPU_LED_STATUS(1);
}


//---------------------------------------------------------------------------
void application_show_memory (void)
{
	cx_uint_t global_variable_size = 0u;

	debug_printf("# GLOBAL VARIABLE SIZE\n");
	debug_printf("\tsizeof(_com1_fifo_tx_buffer      )=%d\n",sizeof(_com1_fifo_tx_buffer      )); global_variable_size += sizeof(_com1_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com1_fifo_rx_buffer      )=%d\n",sizeof(_com1_fifo_rx_buffer      )); global_variable_size += sizeof(_com1_fifo_rx_buffer      );
	debug_printf("\tsizeof(_com2_fifo_tx_buffer      )=%d\n",sizeof(_com2_fifo_tx_buffer      )); global_variable_size += sizeof(_com2_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com2_fifo_rx_buffer      )=%d\n",sizeof(_com2_fifo_rx_buffer      )); global_variable_size += sizeof(_com2_fifo_rx_buffer      );
	debug_printf("\tsizeof(_com3_fifo_tx_buffer      )=%d\n",sizeof(_com3_fifo_tx_buffer      )); global_variable_size += sizeof(_com3_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com3_fifo_rx_buffer      )=%d\n",sizeof(_com3_fifo_rx_buffer      )); global_variable_size += sizeof(_com3_fifo_rx_buffer      );
	debug_printf("\tsizeof(_com4_fifo_tx_buffer      )=%d\n",sizeof(_com4_fifo_tx_buffer      )); global_variable_size += sizeof(_com4_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com4_fifo_rx_buffer      )=%d\n",sizeof(_com4_fifo_rx_buffer      )); global_variable_size += sizeof(_com4_fifo_rx_buffer      );
	
	debug_printf("\tsizeof(_com2_stream_rx_bsb_buffer)=%d\n",sizeof(_com2_stream_rx_bsb_buffer)); global_variable_size += sizeof(_com2_stream_rx_bsb_buffer);
	debug_printf("\tsizeof(_com3_stream_rx_bsb_buffer)=%d\n",sizeof(_com3_stream_rx_bsb_buffer)); global_variable_size += sizeof(_com3_stream_rx_bsb_buffer);
	debug_printf("\tsizeof(_com4_stream_rx_bsb_buffer)=%d\n",sizeof(_com4_stream_rx_bsb_buffer)); global_variable_size += sizeof(_com4_stream_rx_bsb_buffer);
	
	debug_printf("\tsizeof(_analog_data          	 )=%d\n",sizeof(_analog_data          	  )); global_variable_size += sizeof(_analog_data          	   );		
	debug_printf("\tsizeof(_peer_connection          )=%d\n",sizeof(_peer_connection          )); global_variable_size += sizeof(_peer_connection          );		
	debug_printf("\tsizeof(_equipment                )=%d\n",sizeof(_equipment                )); global_variable_size += sizeof(_equipment                );
	debug_printf("\tsizeof(_config                   )=%d\n",sizeof(_config                   )); global_variable_size += sizeof(_config                   );
	
	debug_printf("\tTOTAL = %d \n", global_variable_size);
}


//---------------------------------------------------------------------------
cx_bool_t application_initialize (void)
{
	//-----------------------------------------------------------------------
	application_show_version ();

	//-----------------------------------------------------------------------
	debug_printf("# INITIALIZE CONFIGURATION \n");

	if (CX_FALSE==config_initialize())
	{
		return CX_FALSE;
	}

	//-----------------------------------------------------------------------
	debug_printf("# ENABLE HARDWARE TIMER \n");
	TIM_Cmd(TIM4, ENABLE);
	
	//-----------------------------------------------------------------------
	debug_printf("# INITIALIZE DATA \n");
	peer_initialize();
	equipment_initialize();
	frequency_data_initialize();
		
	//-----------------------------------------------------------------------
	debug_printf("# ENABLE EXTERNAL-WATCHDOG TIMER \n");
	_watchdog_run = CX_TRUE;
	
	//-----------------------------------------------------------------------
	debug_printf("# ENABLE FND DISPLAY \n");	
	TIM_Cmd(TIM5, ENABLE);
	display_maked_fnd_data   ("8888",  _fnd0, 5); 
	display_maked_fnd_data   ("8888",  _fnd1, 5); 
	
	//-----------------------------------------------------------------------
//	ringbuf_init (&Trans1_rxringbuf, Trans1_rxbuf, Trans1_rxbufsize);
//	ringbuf_init (&Trans2_rxringbuf, Trans2_rxbuf, Trans2_rxbufsize);
	return CX_TRUE;
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void hw_driver_initialize (void)
{
	// hardware 초기화 전
	io_fifo_initialize(&_com_fifo[COM2], _com2_fifo_rx_buffer, sizeof(_com2_fifo_rx_buffer), _com2_fifo_tx_buffer, sizeof(_com2_fifo_tx_buffer));
	io_fifo_initialize(&_com_fifo[COM3], _com3_fifo_rx_buffer, sizeof(_com3_fifo_rx_buffer), _com3_fifo_tx_buffer, sizeof(_com3_fifo_tx_buffer));
	io_fifo_initialize(&_com_fifo[COM4], _com4_fifo_rx_buffer, sizeof(_com4_fifo_rx_buffer), _com4_fifo_tx_buffer, sizeof(_com4_fifo_tx_buffer));

	istream_initialize(&_istream2, 2, _com2_stream_rx_bsb_buffer, sizeof(_com2_stream_rx_bsb_buffer), 30u);
	istream_initialize(&_istream3, 3, _com3_stream_rx_bsb_buffer, sizeof(_com3_stream_rx_bsb_buffer), 30u);
	istream_initialize(&_istream4, 4, _com4_stream_rx_bsb_buffer, sizeof(_com4_stream_rx_bsb_buffer), 30u);
	
	io_fifo_initialize(&_com_fifo[COM1], _com1_fifo_rx_buffer, sizeof(_com1_fifo_rx_buffer), _com1_fifo_tx_buffer, sizeof(_com1_fifo_tx_buffer));
		
	debug_shell_initialize();
}


void hw_gpio_initialize (void)
{
	//-----------------------------------------------------------------------
	GPIO_O_EXTERNAL_WATCHDOG_CLOCK (0);
	
	GPIO_O_CPU_LED_STATUS(0);
	//-----------------------------------------------------------------------
		
	GPIO_O_IMPULSE_CLR1 (1); 
    GPIO_O_IMPULSE_CLR2 (1); 	
	
	//-----------------------------------------------------------------------
}

void application_halt (void)
{
	debug_printf("# HALT \n");
#if 0 
	//----------------------------------------------------------------------
	//고장 시 동작 추가
	clear_measured_data();
	
	//---------------------------------------------------------------------------

	while(1)
	{		
		// 디버깅 쉘
		debug_shell();
		
		control_watch();
		check_fault();
		
		if(CX_FALSE == _application_halt)
		{
			__NVIC_SystemReset();		
		}	
	}
#endif	
}

void application_run (void)
{
	cx_uint_t t1;
	cx_uint_t t2;
	
	message_queue_initialize(
		&_nb_o_message_queue, 
		sizeof(_nb_o_message)/sizeof(message_t), 
		sizeof(_nb_o_message_buffer)/(sizeof(_nb_o_message)/sizeof(message_t)), 
		_nb_o_message,
		_nb_o_message_buffer);
	
	
	debug_printf("# START (boot-time = %d second) \n", _operating_time_second);
	
	_timer_run = CX_TRUE;
	
	debug_flush();	
	t1 = _operating_time_second;
	t2 = t1;
	
	memset(Tranmitt_active_data_buf, 0x30, 18);
	
	while (1)
  	{
    	t2 = _operating_time_second;
		
		//recrption
		reception (CX_TRUE);
		update_istream();
		update_equipment();
		Trans1_Input();
		Trans2_Input();
		
		//아날로그 데이터 감시, 궤도계전기 입력 체크
		control_watch();
		control_input();
		
		//Measure ADC
		control_impulse_voltage_input();
        control_relay_voltage_input();
		
		// fail 체크
//		check_watchdog_input_data();	// 외부 와치독 입력 체크
		check_fault ();
		
		// 통신 송신
//		update_ostream();
		transmit ();
		
		//check mode switch
		check_front_button();
		
		//make Display Data
		make_fnd_data();
		
		// 디버깅 쉘
		debug_shell();
				

//---------------------------------------------------------------------------
// TODO: RELEASE version
//---------------------------------------------------------------------------
#if 1
		if(CX_TRUE == _application_halt) 
		{
			if(CX_TRUE == _worker_run)
			{
				_worker_run = CX_FALSE;
				debug_printf("# HALT \n");	
			}
		}		
		else
		{
			if(CX_FALSE == _worker_run)
			{
				_worker_run = CX_TRUE;
				debug_printf("# WORKER RUN \n");
			}	
		}
		
#else
//---------------------------------------------------------------------------
// TODO: DEBUG version
//---------------------------------------------------------------------------
		if(CX_TRUE == _application_halt)
		{
			if (t2!=t1)
			{
				t1 = t2;

				debug_printf("# HALT : %d \n", _operating_time_second);
			}
		}
#endif
  	}
}