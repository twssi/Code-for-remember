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
#define F_CLK 2000000  

volatile uint16_t ic_value1 = 0;
volatile uint16_t ic_value2 = 0;
volatile uint16_t capture_diff = 0;
volatile uint8_t  is_captured = 0;

volatile uint32_t measured_freq_hz = 0;  
#define DEBUG_BUF_SIZE 100
static cx_uint8_t debug_rx_buf[DEBUG_BUF_SIZE];  // 디버깅용 수신 버퍼
static cx_uint32_t debug_rx_len = 0;             // 실제 저장된 길이

static cx_bool_t Track_relay;
static cx_uint_t _timer_count_0010_msec  	= 0u;
static cx_uint_t _timer_count_0020_msec  	= 0u;
static cx_uint_t _timer_count_0100_msec  	= 0u;
static cx_uint_t _timer_count_0200_msec  	= 0u;
static cx_uint_t _timer_count_0300_msec  	= 0u;
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
static cx_bool_t _impulse_voltage_measure_run   = CX_FALSE; //timer2 rising edge(freq)/ for Impulse voltage
static cx_bool_t _flag_control_watch		= CX_FALSE;
static cx_bool_t _flag_update_equipment 	= CX_FALSE;
static cx_bool_t _flag_update_ostream   	= CX_FALSE;
static cx_bool_t _flag_transmit         	= CX_FALSE;

static cx_bool_t _flag_get_impulse_voltage  = CX_FALSE;
static cx_bool_t _flag_get_relay_voltage 	= CX_FALSE;

cx_uint_t _relay_off_duration = 0;
cx_bool_t _prev_relay_input_value;
cx_bool_t _relay_flicker_detected;

static cx_uint_t transmit_relay_input_value 		= 0u;
static cx_bool_t flag_debug;
//static cx_bool_t _flag_check_watchdog_input_data = CX_FALSE;
static cx_bool_t	_flag_trans_state      	= CX_FALSE;
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

static cx_uint_t _flag_TR1_parsedone		= CX_FALSE;
static cx_uint_t _flag_TR2_parsedone		= CX_FALSE;

static cx_uint_t _pre_hotstandby 			= 0u;
static cx_uint_t _check_rising_edge_tim3	= 0u;

cx_uint_t rx_frequency;
cx_uint_t de_rx_frequency;
//===========================================================================
// reg -> bfifo -> bsb -> stream
//debug
static cx_byte_t _com1_fifo_tx_buffer      [4096];
static cx_byte_t _com1_fifo_rx_buffer      [  64];


//static cx_byte_t Tranmitt_active_data_buf  [  18];

static cx_uint_t TR1_active		= 0;
static cx_uint_t TR2_active		= 0;

static cx_uint_t TR1_state		= 0;
static cx_uint_t TR2_state		= 0;
static cx_uint8_t _transmit_state_data;
static cx_uint8_t transmit_data1;
static cx_uint8_t transmit_data2;
static cx_uint8_t transmit_data = CX_TRUE;

static cx_uint_t _freq_count			  			= 0u;
static cx_bool_t _freq_state			  			= IDLE;
static cx_bool_t _freq_fail_state			  		= CX_FALSE;
static cx_uint_t _freq_fail_count		  			= 0u;
static cx_bool_t _freq_health_fail		  			= CX_FALSE;
static cx_uint_t _freq_data				  			= 0u;

static cx_uint_t cur_debounce_count = 0u;
static cx_uint_t pre_debounce_count = 0u;

//To Tx 2
static cx_byte_t _com2_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com2_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com2_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer �ι������� Ŀ�� ��

//To D.G
static cx_byte_t _com3_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com3_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com3_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer �ι������� Ŀ�� ��

//To TX 1
static cx_byte_t _com4_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com4_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com4_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer �ι������� Ŀ�� ��

// static RINGBUF               Trans1_rxringbuf;
// static cx_uint8_t        Trans1_rxbuf[128];
// static const cx_uint32_t    Trans1_rxbufsize = 128;

// static RINGBUF               Trans2_rxringbuf;
// static cx_uint8_t         Trans2_rxbuf[128];
// static const cx_uint32_t    Trans2_rxbufsize = 128;

static cx_uint8_t         Trans1_rxbuf_debug[25];
static cx_uint8_t         Trans2_rxbuf_debug[25];
static cx_uint8_t		  Trans1_rx_data[18];
static cx_uint8_t		  Trans2_rx_data[18];



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

static cx_uint8_t         Trans2_debugbuf[25];
//--------------fnd ǥ�� ���� ����-----------------------------------------
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

void control_impulse_voltage_input (void);

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
	_timer_count_0300_msec++;
	if(_timer_count_0300_msec>300u*2)
	{
		_timer_count_0300_msec = 0u;
		
	}
	//-----------------------------------------------------------------------
	_timer_count_0200_msec++;
	if (_timer_count_0200_msec>200u*2)
	{
		_TX1_rx_timeout_count++;
		_TX2_rx_timeout_count++;
		_timer_count_0200_msec = 0u;
		//---------------------------------------------------------------------
		_flag_transmit = CX_TRUE;	
		//_flag_control_input = CX_TRUE;
	}

	//-----------------------------------------------------------------------
	_timer_count_0100_msec++;
	if (_timer_count_0100_msec>100u*2)
	{
		_timer_count_0100_msec = 0u;
		_flag_control_watch = CX_TRUE;
		//-------------------------------------------------------------------
		peer_timer_irq_handler   ();	//peer
		istream_timer_irq_handler();	//stream
		
		
	}
	
	//-----------------------------------------------------------------------
	_timer_count_0020_msec++;

	if (_timer_count_0020_msec>20u*2)
	{
	
		_timer_count_0020_msec = 0u;
		
	}
    _timer_count_0010_msec++;

	if (_timer_count_0010_msec>10u*2)
	{
		_timer_count_0010_msec = 0u;
		_flag_control_input = CX_TRUE;
		_flag_trans_state = CX_TRUE;
	}
	//-----------------------------------------------------------------------		
	cur_debounce_count ++;
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
	//_flag_get_impulse_voltage = CX_TRUE;	//
	control_impulse_voltage_input();
	get_impulse_voltage_irq_handler();
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
#if 1
void calculate_inputcapture_TIM3 (void)
{
	static cx_uint_t gu32_T1H	= 0;
	static cx_uint_t gu32_T2H	= 0;
	static cx_uint_t gu32_Ticks	= 0;
	static cx_uint_t gu32_Freq	= 0;

	if(_freq_state == IDLE)
	{
			_freq_count 	= 0;
			_freq_state 	= DONE;
	}
	else if(_freq_state == DONE)
	{
		if(_freq_count >= 100)
		{ 
			gu32_T1H 		= _freq_count;
			gu32_Freq 		= (uint32_t)(F_CLK/gu32_T1H);
			de_rx_frequency = gu32_Freq;
			_freq_state 	= IDLE;
			_freq_count		= 0;

			_freq_data		= gu32_Freq;
			
			if(_freq_data==301 || _freq_data == 299) _freq_data = 300; // 3Hz

		}
	}
}
#endif
//adc dma 
void measure_current_irq_handler (void)	// dma �����Ҷ����� ȣ��
{	
	//dma interrupt 
	//put_current_value();

	//-----------------------------------------------------------------------
}
void get_impulse_voltage_irq_handler (void)
{
//	_flag_get_impulse_voltage = CX_TRUE;	//50us
	_freq_count++;
	if(_freq_count > 30000)
	{
		_freq_state 	= IDLE;
		_freq_fail_state = CX_TRUE;
		if(_freq_count >= 30000)
		{
			_freq_data = 0;
			_freq_count 	= 30000;
		}
	}
	if(_freq_fail_state == CX_TRUE)
	{
		if(_freq_fail_count++ >= 40000)
		{
			_freq_health_fail = CX_TRUE;
			_freq_fail_count = 40000;
		}
	}
	else
	{
		_freq_fail_count	= 0;
		_freq_health_fail 	= CX_FALSE;
	}
	//control_impulse_voltage_input();
}

//-----------------------------------------------------------------------
void control_impulse_voltage_input(void)
{
	//-----------------------------------------------------------------------
	//if (CX_FALSE ==_flag_get_impulse_voltage)	//50us, 0.05ms
	//{
	//	return;
	//}
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
        GPIO_O_IMPULSE_CLR1(0); 	//���޽� Clear
    } 
	
	if(_count_measure_impulse_voltage >= 40)   //rx: 40(2ms)     tx: 50(2.5ms)
    {
        GPIO_O_IMPULSE_CLR2(0);     //���޽� Clear
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
	static cx_uint8_t buf[25];

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
# if 0
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
#endif 

uint8_t fcs_rx(uint8_t* data, uint8_t ID,cx_uint16_t len)
{
	cx_uint16_t i;
	cx_uint8_t ch;
	cx_uint8_t p_val;
	cx_uint8_t ID_val;

	ch = 0;

	ID_val = ID;
	if(ID_val >= 197)									p_val = 0x19;
	else if(ID_val >= 190 && ID_val < 195) 			 	p_val = 0x12;
	else if(ID_val >= 0XB4 && ID_val <= 0xB9) 			p_val = 0x19; //180~185
	else if(ID_val >= 0XB0 && ID_val <= 0XB3) 			p_val = 0x19; //176~179
	else if(ID_val >= 0xAA && ID_val <= 0XAF)			p_val = 0x12; //170~175
	else if(ID_val >  0x99 && ID_val <= 0xA9) 			p_val = 0x19; //154~ 169
	else p_val = 0x20;


	for (i = 0; i < len; i++)
	{

		ch += data[i];

	}

		ch += p_val;



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
	
	_TX1_rx_timeout_count=0;
	memcpy(Trans1_rx_data,&bufptr[bufsize-23],18);
	

}

void Trans2_Input_Parse(cx_uint8_t * bufptr, cx_uint16_t bufsize)
{
	
	_TX2_rx_timeout_count=0;
	memcpy(Trans2_rx_data,&bufptr[bufsize-23],18);
	

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


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void check_state_Transmitter(void)
{
	if(CX_FALSE == _flag_trans_state)
	{
		return;
	}
	
	TR1_active = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
	TR2_active = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
	
	TR1_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
	TR2_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);

	if(TR1_state == LOW && TR2_state == LOW && TR1_active == HIGH && TR2_active == LOW) _transmit_state_data = 0x00;
	else if(TR1_state == LOW && TR2_state == LOW && TR1_active == LOW && TR2_active == HIGH) _transmit_state_data = 0x01;
	else if(TR1_state == LOW && TR2_state == HIGH && TR1_active == HIGH && TR2_active == HIGH) _transmit_state_data = 0x02;
	else if(TR1_state == HIGH && TR2_state == LOW && TR1_active == HIGH && TR2_active == HIGH) _transmit_state_data = 0x03;
	else if(TR1_state == HIGH && TR2_state == HIGH && TR1_active == HIGH && TR2_active == HIGH) _transmit_state_data = 0x04;
	
	if(_TX1_rx_timeout_count > 5)
	{
		memset(Trans1_rx_data,0x00,18);
		_TX1_rx_timeout_count = 5;
	}


	if(_TX2_rx_timeout_count > 5)
	{
		memset(Trans2_rx_data,0x00,18);
		_TX2_rx_timeout_count = 5;
	}


	_flag_trans_state =CX_FALSE;
}

void put_data(cx_byte_t *str, cx_uint16_t len) {

	cx_uint16_t i;
	for(i=0;i<len;i++) {
		dbg_put_char(str[i]);
	}
}

static void transmit (void)
{
	if (_flag_transmit == CX_FALSE)
		return;

	cx_uint_t config_id;
	cx_uint_t impulse_voltage_plus;
	cx_uint_t impulse_voltage_minus;
	cx_uint_t relay_voltage_v1;
	cx_uint_t relay_voltage_v2;
	cx_uint_t rx_current;
	
	
	cx_uint16_t temp_ReadID1 = 0u;
	cx_uint16_t temp_ReadID2 = 0u;
	cx_uint16_t rotary_sw_1 = 0u;
	cx_uint16_t rotary_sw_2 = 0u;

	cx_uint_t i;
	cx_uint8_t fcs_value;
	cx_uint_t size;
	cx_uint8_t relay_data;
	cx_uint8_t ID_VAL;
	static cx_uint_t _count_relay_vol_off = 0;

	uint8_t local_data_copy[18];  // 로컬 복사본 생성

	memset(local_data_copy, 0x30, sizeof(local_data_copy)); 

	if(0x00 == _transmit_state_data || 0x02 == _transmit_state_data) memcpy(local_data_copy, Trans1_rx_data, 18);
	else if(0x01 == _transmit_state_data || 0x03 == _transmit_state_data) memcpy(local_data_copy, Trans2_rx_data, 18);
	else if(0x04 == _transmit_state_data) memset(local_data_copy, 0x30, 18);

	temp_ReadID1 = GPIO_ReadInputData(GPIOE);
	temp_ReadID1 = ~(temp_ReadID1) & 0x3C;
	rotary_sw_1 = reverse4((temp_ReadID1 >> 2) & 0x0F);

	temp_ReadID2 = GPIO_ReadInputData(GPIOC);
	temp_ReadID2 = ~(temp_ReadID2) & 0x000F;
	rotary_sw_2 = reverse4((temp_ReadID2 >> 0) & 0x0F);

	ID_VAL = (rotary_sw_1<<4)| rotary_sw_2;


	impulse_voltage_plus  = get_imnpulse_voltage_plus_value() / 10;
	impulse_voltage_minus = get_imnpulse_voltage_minus_value() / 10;
	relay_voltage_v1      = get_relay_voltage_v1_value() / 10;
	relay_voltage_v2      = get_relay_voltage_v2_value() / 10;
	rx_current            = get_rx_current_value() / 10;
	rx_frequency          = _freq_data;//(cx_uint_t)(get_rx_3hz_frequency_value() * 100);



	_DG_tx_buffer[0] = 0x02;
	_DG_tx_buffer[1] = fcs_conv(rotary_sw_1);
	_DG_tx_buffer[2] = fcs_conv(rotary_sw_2);
	_DG_tx_buffer[3] ='0'+ (_transmit_state_data & 0x0F);
	
	memcpy(&_DG_tx_buffer[4], local_data_copy, 18);  // 복사본을 사용
	
	
	for (i = 0; i < 3; i++)
		_DG_tx_buffer[22 + i] = (impulse_voltage_plus / POW(10, 2 - i)) % 10 + '0';

	for (i = 0; i < 3; i++)
		_DG_tx_buffer[25 + i] = (impulse_voltage_minus / POW(10, 2 - i)) % 10 + '0';

	for (i = 0; i < 3; i++)
		_DG_tx_buffer[28 + i] = (rx_frequency / POW(10, 2 - i)) % 10 + '0';

	for (i = 0; i < 3; i++)
		_DG_tx_buffer[31 + i] = (rx_current / POW(10, 2 - i)) % 10 + '0';

	for (i = 0; i < 3; i++)
		_DG_tx_buffer[34 + i] = (relay_voltage_v2 / POW(10, 2 - i)) % 10 + '0';

	for (i = 0; i < 3; i++)
		_DG_tx_buffer[37 + i] = (relay_voltage_v1 / POW(10, 2 - i)) % 10 + '0';


	_DG_tx_buffer[40] = transmit_relay_input_value;
	fcs_value = fcs_rx(&_DG_tx_buffer[1],ID_VAL, 40);
	_DG_tx_buffer[41] = fcs_conv(fcs_value >> 4);
	_DG_tx_buffer[42] = fcs_conv(fcs_value & 0x0F);
	_DG_tx_buffer[43] = 0x03;
	_DG_tx_buffer[44] = 0x0D;
	_DG_tx_buffer[45] = 0x0A;

	//put_str(_DG_tx_buffer);  // 실제 송신
	put_data(_DG_tx_buffer, 46);  // 실제 송신
	_timer_count_transmit = CX_FALSE;
	memset(&_DG_tx_buffer[4], 0x30, 18);  // 다음 출력 초기화
	_flag_transmit = CX_FALSE;
}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void control_watch (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_watch)	//100ms �ֱ�
	{
		return;
	}
	
	cx_uint_t fail_check_chattering_count = 30u;	//3��
	
	cx_uint_t relay_drop_condition;
	cx_uint_t relay_excitaion_condition;

	cx_uint_t get_relay_voltage_V1;
	cx_uint_t get_relay_voltage_V2;
	
	cx_bool_t fail_max_count = 15u;	//3초
	cx_bool_t check_railrelay_drop = 8u; //2초

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
	
	

	//계전기 여자되는 범위, V1전압 20/V2전압 30 이상
	if( (get_relay_voltage_V1 >= 210) && (get_relay_voltage_V2 >= 300) )	
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
	//부정낙하: 임펄스 전압 있는데 궤도계전기가 낙하인 상태
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
	//부정여자: 임펄스 전압 없는데 궤도계전기가 여자인 상태
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
		flag_debug = _false_relay_drop;
	
	}	
	else 
	{
		_inconsistency_fail = CX_FALSE;
	}	
	//-----------------------------------------------------------------------
	//-----------------------------------------------------------------------
	_flag_control_watch = CX_FALSE;
}


static void control_input(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_input)	//each 10ms check. 
	{
		return;
	}
	//-----------------------------------------------------------------------
	cx_bool_t relay_input;
	
	cx_uint_t input_trans_tab;

	cx_uint_t input_chattering_count 	= 5u;
	static cx_uint_t _relay_input_count = 0u;
	static cx_uint_t _count_relay_vol_off = 0;

	cx_uint_t get_relay_voltage_V1 = 0;
	cx_uint_t get_relay_voltage_V2 = 0;
	
	//-----------------------------------------------------------------------
	

	get_relay_voltage_V1  = get_relay_voltage_v1_value();
	get_relay_voltage_V2  = get_relay_voltage_v2_value();
	
	if( LOW == GPIO_I_Track_Relay())
	{
		relay_input = CX_TRUE;
	}
	else if( HIGH == GPIO_I_Track_Relay())
	{
		relay_input = CX_FALSE;
	}
	else relay_input = CX_FALSE;

	if(CX_TRUE == relay_input )
	{
		transmit_relay_input_value = 0x31;
		_count_relay_vol_off = 0;
	} 
	else
	{
		if(CX_FALSE == relay_input)
		{

			if(get_relay_voltage_V1 > 250 && get_relay_voltage_V2 > 350)	
			{
				_count_relay_vol_off++;
				if(_count_relay_vol_off>=300) //부정낙하 시간 조정
				{
					transmit_relay_input_value = 0x32;
					_count_relay_vol_off = 300;
				}
				else // checking time should be 200ms //else if(_count_relay_vol_off > 20)
				{
					transmit_relay_input_value = 0x30;
				}
			}
			else
			{
				transmit_relay_input_value = 0x34;
				_count_relay_vol_off=0;
			}
			if(0x34 == _transmit_state_data)
			{
				transmit_relay_input_value = 0x34;

			}
		}
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
static void update_istream (void)//�ֽ� ������Ʈ �Է� ����
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

	
	rx_size = com_recv_buffer(COM2, _nb_rx_buffer, sizeof(_nb_rx_buffer)); //Tx 2��
	if (rx_size > 0u)
	{
		if (CX_TRUE==enable)
		{
			istream_packet_push(&_istream2, _nb_rx_buffer, rx_size);
		}

	}

	rx_size = com_recv_buffer(COM4, _nb_rx_buffer, sizeof(_nb_rx_buffer));	//Tx 1��
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
		position = 0;     	 //ù��° �ڸ�
    }
	else if(len == 4)
    {
	 	dot    = *(numstring+1);
	 	if(dot == 0x2E)  position    = 1;	//�ι�° �ڸ�
	 	else position = 0;     				//ù��° �ڸ�
       
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
	
	ReadValue_portD = GPIO_ReadInputData(GPIOD);   // ���� ���� ������ �б� 
	
	portd_value |= select;    //PD8~PD15
	GPIO_Write(GPIOD,(ReadValue_portD & 0x00FF) | portd_value);
	
	ReadValue_portE = GPIO_ReadInputData(GPIOE);   // ���� ���� ������ �б� 
	
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
		
		_count_active_fnd_display = 0u;	//���������� FND off�� ���� ī��Ʈ Ŭ����, �ð������� off��Ű�� ���� ����
	}
}

void check_front_button(void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE == _flag_mode_sw_sequence)	//front FND ��ư ������ ���
	{
		return; 
	}
	//-----------------------------------------------------------------------	
	static cx_uint_t _input_mode_sw = 0u;
	
	_flag_active_display = CX_TRUE;
	
	_input_mode_sw = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);
	
	if(_input_mode_sw == 1)
	{
		if(cur_debounce_count-pre_debounce_count > 200u)
		{
			_status_mode_switch = CX_TRUE;
			_display_sequence++;
			if(_display_sequence>=5) _display_sequence = 1u;
			_flag_active_mode_switch = CX_FALSE;

			pre_debounce_count = cur_debounce_count;
		}
	}	
	
	//-----------------------------------------------------------------------
	_flag_mode_sw_sequence = CX_FALSE;
}

void make_fnd_data (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_make_display_data)	//500ms �ֱ�
	{
		return;
	}

	//-----------------------------------------------------------------------
	char first_line   [5]  = {0,};	
	char second_line  [5]  = {0,};	
//	cx_uint_t  fnd_active_max_count 	  = 100u;	
	cx_uint_t  fnd_change_max_count 	  = 10u;	//500ms*10 = 5��
	
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
	impulse_voltage_plus  = get_imnpulse_voltage_plus_value()/10;	//3�ڸ�
	impulse_voltage_minus = get_imnpulse_voltage_minus_value()/10;	//3�ڸ�
	
	relay_voltage_v1 = get_relay_voltage_v1_value()/10;			//3�ڸ�
	relay_voltage_v2 = get_relay_voltage_v2_value()/10;			//3�ڸ�
	
	rx_current 		= get_rx_current_value();
	int_rx_current  = (cx_uint_t)(rx_current/10);
	
	fnd_rx_current = (cx_float_t)int_rx_current/100;
	
	rx_frequency 	= (cx_float_t)_freq_data/100; ////get_rx_3hz_frequency_value();
	
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

		if (_count_active_fnd_display > fnd_change_max_count)	//5�� ��  
		{
			_display_sequence = 1u;
		}

#if 0
		//�ð� ������ FND OFF��ų �� ���	
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

	if(_operating_time_second>3) //run ���� 3�� ���� üũ
	{
#if 0		
		//�����Ǹ� ���ļ��� �ȵ�����?? 0���� �ȵ�����..failó�� �� �ʿ� ������
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
	//RUN,FAIL LED ����
	if(_application_halt == CX_TRUE)	GPIO_O_CPU_LED_STATUS(0);
	else 								GPIO_O_CPU_LED_STATUS(1);
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



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void hw_driver_initialize (void)
{
	// hardware �ʱ�ȭ ��
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
	//���� �� ���� �߰�
	clear_measured_data();
	
	//---------------------------------------------------------------------------

	while(1)
	{		
		// ����� ��
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
	
	memset(Trans1_rx_data, 0x30, 18);
	memset(Trans2_rx_data, 0x30, 18);
        
	while (1)
  	{
    	t2 = _operating_time_second;
		
		//recrption
		//reception (CX_TRUE);
		//update_istream();
		//update_equipment();
		Trans1_Input();
		Trans2_Input();
		
		//�Ƴ��α� ������ ����, �˵������� �Է� üũ
		control_watch();
		control_input();
		
		//Measure ADC
		//control_impulse_voltage_input();
        control_relay_voltage_input();
		
		// fail üũ
//		check_watchdog_input_data();	// �ܺ� ��ġ�� �Է� üũ
		check_fault ();
		
		// ��� �۽�
//		update_ostream();
		check_state_Transmitter();
		transmit ();
		
		//check mode switch
		check_front_button();
		
		//make Display Data
		make_fnd_data();
		
		// ����� ��
		//debug_shell();
				

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