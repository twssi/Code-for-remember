/////////////////////////////////////////////////////////////////////////////
//
// Header Files
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "stm32f10x_init.h"
#include "mcu.h"

#include "mcu_redundant.h"
#include "mcu_gpio_alias.h"

/////////////////////////////////////////////////////////////////////////////
//
// Private Global Variables
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static cx_uint_t _timer_count_50_msec  			 	= 0u;
static cx_uint_t _timer_count_0050_msec  			= 0u;
static cx_uint_t _timer_count_0100_msec  			= 0u;
static cx_uint_t _timer_count_0300_msec  			= 0u;
static cx_uint_t _timer_count_0500_msec  			= 0u;
static cx_uint_t _timer_count_1000_msec  			= 0u;
static cx_uint_t _timer_count_5000_msec  			= 0u;
static cx_uint_t _timer_count_2000_msec  			= 0u;

static cx_uint_t _output_v_timer_count_0300_msec  	= 0u;
static cx_uint_t _watchdog_timer_count_0100_msec	= 0u;
static cx_uint_t _mode_sw_timer_count_0100_msec  	= 0u;

static cx_uint_t _freq_count			  			= 0u;
static cx_bool_t _freq_state			  			= IDLE;
static cx_bool_t _freq_fail_state			  		= CX_FALSE;
static cx_uint_t _freq_fail_count		  			= 0u;
static cx_bool_t _freq_health_fail		  			= CX_FALSE;
static cx_uint_t _freq_data				  			= 0u;

static cx_uint_t _operating_time_second 			= 0u;
static cx_uint_t _active_operating_time_second 		= 0u;

static cx_bool_t _flag_get_output_voltage 	 		= CX_FALSE;
//static cx_bool_t _flag_get_impulse_voltage  		= CX_FALSE;	

static cx_bool_t _external_watchdog_clock_enabled 	= CX_TRUE; //
static cx_uint_t _external_watchdog_clock_output  	= 0u;

static cx_bool_t _timer_run 			 	 	= CX_FALSE; 
static cx_bool_t _watchdog_run              	= CX_FALSE; 
static cx_bool_t _watchdog_fail             	= CX_FALSE;
static cx_bool_t _output_voltage_measure_run    = CX_FALSE;	//timer3 rising edge(clock pin)
static cx_bool_t _impulse_voltage_measure_run   = CX_FALSE; //timer2 rising edge(freq)/ for Impulse voltage
static cx_bool_t _boot_state 			 	 	= CX_FALSE; 

static cx_bool_t _flag_control_input    		= CX_FALSE;
static cx_bool_t _flag_control_output   		= CX_FALSE;
static cx_bool_t _flag_worker_state    		= CX_FALSE;
static cx_bool_t _flag_worker_health_state	= CX_TRUE;

static cx_bool_t _input_manual_switch    		= CX_FALSE;
static cx_bool_t _output_switchover_control     = CX_TRUE;
static cx_bool_t _output_smps_output_control    = CX_FALSE;
static cx_bool_t _health_output_voltage    		= CX_TRUE;
//static cx_bool_t _health_ac_voltage    		= CX_TRUE;
//static cx_bool_t _input_manual_switch_2    	= CX_FALSE;
//static cx_bool_t _flag_check_watchdog_input_data= CX_FALSE;

static cx_bool_t _flag_control_watch			= CX_FALSE;
static cx_bool_t _flag_update_equipment 		= CX_FALSE;
static cx_bool_t _flag_update_ostream   		= CX_FALSE;
static cx_bool_t _flag_transmit         		= CX_FALSE;
		
static cx_uint_t _count_active_fnd_display		= 0u;
static cx_uint_t _display_sequence				= 0u;
static cx_uint_t _status_mode_switch			= CX_TRUE;
static cx_uint_t _flag_active_mode_switch		= CX_FALSE;
			
static cx_uint_t _flag_make_display_data		= CX_FALSE;
static cx_uint_t _flag_mode_sw_sequence			= CX_FALSE;
static cx_uint_t _flag_active_display			= CX_TRUE;

static cx_uint_t _check_rising_edge_tim2		= 0u;
static cx_uint_t _check_rising_edge_tim3		= 0u;
static cx_uint_t _check_TIM2_freq_OVC			= 0;

static cx_uint_t _fail_condition_frequency		= CX_FALSE;
static cx_uint_t _fail_condition_outputvoltage 	= CX_FALSE;

cx_uint_t _application_halt				= CX_FALSE;				

static cx_uint_t cur_debounce_count = 0u;
static cx_uint_t pre_debounce_count = 0u;



static __IO uint32_t TimingDelay;
//-----------------------------------------------------------------------
// reg -> bfifo -> bsb -> stream
static cx_byte_t _com1_fifo_tx_buffer      [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _com1_fifo_rx_buffer      [NB_PACKET_RX_MAX_SIZE];
static cx_byte_t _com1_stream_rx_bsb_buffer[NB_PACKET_RX_MAX_SIZE*2u]; // fifo_rx_buffer 두배사이즈보다 커야 함

static cx_byte_t _com1_tx_buffer      [24];

static cx_byte_t _com3_fifo_tx_buffer      [4096];
static cx_byte_t _com3_fifo_rx_buffer      [64];

//--------------fnd 표시 변수 선언-----------------------------------------
cx_bool_t _fnd0[5];
cx_bool_t _fnd1[5];

#define	F_CLK			1000000

#define FND_CH_00(pos)  ((pos)&0x000f)<<0   // first line
#define FND_CH_01(pos)  ((pos)&0x000f)<<4   // second line

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


//===========================================================================
static message_queue_t _nb_o_message_queue  ;
static message_t       _nb_o_message        [NB_O_MESSAGE_MAX_COUNT];
static cx_byte_t       _nb_o_message_buffer [NB_O_MESSAGE_MAX_COUNT*NB_O_MESSAGE_MAX_SIZE];

static cx_byte_t _nb_tx_buffer [NB_PACKET_TX_MAX_SIZE];
static cx_byte_t _nb_rx_buffer [NB_PACKET_RX_MAX_SIZE];

static cx_byte_t _message [NB_O_MESSAGE_MAX_SIZE];

/////////////////////////////////////////////////////////////////////////////
//
// Global Variables
//
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
/* Private function prototypes -----------------------------------------------*/
void delay_msec (__IO cx_uint_t msec);
static void control_impulse_voltage_input (void);

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

//-----------------------------------------------------------------------
#if 0
//DMA for adc 
void calculate_adc_irq_handler (void)	
{	
	//DMA interrupt 
//	put_analog_value();
}
#endif

void get_impulse_voltage_irq_handler (void)
{
//	_flag_get_impulse_voltage = CX_TRUE;	//50us
	_freq_count++;
	if(_freq_count > 3436)
	{
		_freq_state 	= IDLE;
		_freq_fail_state = CX_TRUE;
		if(_freq_count >= 10000)
		{
			_freq_data = 0;
			_freq_count 	= 10000;
		}
	}
	if(_freq_fail_state == CX_TRUE)
	{
		if(_freq_fail_count++ >= 30000)	//3초   //if(_freq_fail_count++ >= 30000)//3초
		{
			_freq_health_fail = CX_TRUE;
			_freq_fail_count = 30000;
		}
	}
	else
	{
		_freq_fail_count	= 0;
		_freq_health_fail 	= CX_FALSE;
	}
	control_impulse_voltage_input();
}

//-----------------------------------------------------------------------
static void output_voltage_measure_irq_handler (void)
{
	_output_v_timer_count_0300_msec++;
    if (_output_v_timer_count_0300_msec>280u*2 && _output_v_timer_count_0300_msec<310u*2)    
	{
		//_output_v_timer_count_0300_msec = 0u;

		_flag_get_output_voltage 	= CX_TRUE;		
	}
	else
	{
		_flag_get_output_voltage 	= CX_FALSE;	
	}
}

static void output_voltage_measure_irq_handler1 (void)
{
	_output_v_timer_count_0300_msec++;
    if (_output_v_timer_count_0300_msec>300u*2)    
	{
		_output_v_timer_count_0300_msec = 0u;

		_flag_get_output_voltage 	= CX_TRUE;		
	}	
}

//-----------------------------------------------------------------------
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
    
  //  hotstandby_irq_handler();
	//-----------------------------------------------------------------------
	_flag_control_output = CX_TRUE;	

	//-----------------------------------------------------------------------
	_timer_count_5000_msec++;
	if (_timer_count_5000_msec>5000u*2)
	{
		_timer_count_5000_msec = 0u;
		
		//-------------------------------------------------------------------
		_flag_update_equipment = CX_TRUE;	//equipment		
	}

	//-----------------------------------------------------------------------
	_timer_count_2000_msec++;
	if (_timer_count_2000_msec>2000u*2)	
	{
		_timer_count_2000_msec = 0u;

		//-------------------------------------------------------------------
		
	}
	
	//-----------------------------------------------------------------------
	_timer_count_1000_msec++;
	if (_timer_count_1000_msec>1000u*2)
	{
		_timer_count_1000_msec = 0u;

		//-------------------------------------------------------------------
		_operating_time_second++;
		if(_operating_time_second >=6)
		{
			_boot_state  = CX_TRUE;
			_operating_time_second = 6u;
		}
		if(CX_TRUE == _flag_worker_state) // 수정필요
		{
			_active_operating_time_second++;	
			if(_active_operating_time_second>=0xFFFF) _active_operating_time_second = 10u; 
		}	
		else _active_operating_time_second = 0u;
		
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
	_timer_count_0300_msec++;
	if (_timer_count_0300_msec>300u*2)	
	{
		_timer_count_0300_msec = 0u;
		
		//-------------------------------------------------------------------
		_flag_control_watch = CX_TRUE;
			
	}
	
	//-----------------------------------------------------------------------
	_timer_count_0100_msec++;
	if (_timer_count_0100_msec>100u*2)
	{
		_timer_count_0100_msec = 0u;
		
		//-------------------------------------------------------------------
		peer_timer_irq_handler   ();	//peer
		istream_timer_irq_handler();	//stream	 

	
	}
	//-----------------------------------------------------------------------
	_timer_count_0050_msec++;
	cur_debounce_count++;

	if (_timer_count_0050_msec>50u*2)
	{
		
		_timer_count_0050_msec = 0u;
		
		//-------------------------------------------------------------------	
		_flag_control_input = CX_TRUE;	
	}
	
	//-----------------------------------------------------------------------
	//_timer_count_transmit++;
	
	_timer_count_50_msec++;
	if (_timer_count_50_msec>200u*2)
	{
		_timer_count_50_msec = 0u;
		
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
	
}

void timer_watchdog_irq_handler (void)
{
	if (CX_TRUE==_watchdog_run) watchdog_run_irq_handler();
}


void timer_500usec_irq_handler (void) // stm32f_it.c에서 호출(500us 주기)
{			
	if (CX_TRUE==_timer_run)
	{
		timer_run_irq_handler();
	}
	else
	{
		timer_boot_irq_handler();
	}
	
	// if (CX_TRUE==_output_voltage_measure_run) output_voltage_measure_irq_handler1();
	output_voltage_measure_irq_handler();
	
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



/* Private functions ---------------------------------------------------------*/

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

uint8_t fcs_conv(uint8_t data)
{
	uint8_t value;


	//value = data;
	if(data>9)
	{
		value = 'A'+(data%10);
	}
	else
	{
		value = '0'+data;
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

  USART1->DR = (dat & (u16)0x01FF);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);     // tx interrupt
}

/******************************************************************************************************/
/* put_str	:																														*/												
/******************************************************************************************************/
void put_str(cx_byte_t *str) {

	cx_uint16_t i;
	for(i=0;i<strlen(str);i++) {
		dbg_put_char(str[i]);
	}
}


//===========================================================================
void rising_edge_flag_TIM2 (void)
{	
	/*//20241211
	_impulse_voltage_measure_run  = CX_TRUE;
	set_flag_rising_edge_pin_freq();	
	
	
	_timer_count_transmit=0u;
	*/
	_impulse_voltage_measure_run  = CX_TRUE;//20241211
	_check_rising_edge_tim2 = 0u;
	_check_TIM2_freq_OVC = 0; ///PWS 추가
}

//===========================================================================

void rising_edge_flag_TIM3 (void) //stm32f_it.c에서 호출 input capture
{
	_output_voltage_measure_run = CX_TRUE;
	_output_v_timer_count_0300_msec = 0u;
	_check_rising_edge_tim3 = 0u;
////////////////////
	//TEST_O=!TEST_O;	
	set_flag_rising_edge_pin_freq();	//20241211
	
//	_timer_count_transmit=0u;//20241211
	//////////////////////
}

//===========================================================================
void check_impuls_freqout (void)
{
	_check_TIM2_freq_OVC++;
	if(_check_TIM2_freq_OVC>=3u)
	{
		_check_TIM2_freq_OVC = 10u;
		_flag_worker_state = CX_FALSE;
	}
	else
	{
		_flag_worker_state = CX_TRUE;
	}
}

void calculate_inputcapture_TIM2 (void)
{
	static cx_uint16_t _capture_number_TIM2	= 0u;
	static cx_uint16_t _readvalue1_TIM2		= 0u;
	static cx_uint16_t _readvalue2_TIM2		= 0u;
	
	cx_uint16_t capture_value			= 0u;
	cx_float32_t pulse_frequecy_TIM2 	= 0u;
			
	if(_capture_number_TIM2 == 0)
	{
		/* Get the Input Capture value */
		_readvalue1_TIM2 = TIM_GetCapture4(TIM2);

		_capture_number_TIM2 = 1;
	}
	else if(_capture_number_TIM2 == 1)
	{
		/* Get the Input Capture value */
		_readvalue2_TIM2 = TIM_GetCapture4(TIM2); 
			 
		/* Capture computation */
		if (_readvalue2_TIM2 > _readvalue1_TIM2)
		{
			capture_value = _readvalue2_TIM2 - _readvalue1_TIM2; 
		}
		else
		{
			capture_value = (0xFFFF - _readvalue1_TIM2) + _readvalue2_TIM2;	//0xFFFF : Timer2�� period
		}
		
		/* Frequency computation */ 
		pulse_frequecy_TIM2 = ((cx_float32_t)(72000000/2400) / capture_value);	//2400 : Timer2�� TIM_Prescaler
		
	//	put_pulse_frequency(pulse_frequecy_TIM2, 2); //20241211
	
		//_check_TIM2_freq_OVC = 0; ///PWS 추가
		_capture_number_TIM2 = 0;	
	}
}
#if 0
void calculate_inputcapture_TIM3 (void)
{
	static cx_uint16_t _capture_number_TIM3	= 0u;
	static cx_uint16_t _readvalue1_TIM3		= 0u;
	static cx_uint16_t _readvalue2_TIM3		= 0u;

	cx_uint16_t capture_value			= 0u;
	cx_float32_t pulse_frequecy_TIM3 	= 0.0;
	
	if(_capture_number_TIM3 == 0)
	{
		/* Get the Input Capture value */
		_readvalue1_TIM3 = TIM_GetCounter(TIM2);
		
		_capture_number_TIM3 = 1;
	}
	else if(_capture_number_TIM3 == 1)
	{
		/* Get the Input Capture value */
		_readvalue2_TIM3 = TIM_GetCounter(TIM2); 
			 
		/* Capture computation */
		if (_readvalue2_TIM3 > _readvalue1_TIM3)
		{
			capture_value = _readvalue2_TIM3 - _readvalue1_TIM3; 
		}
		else
		{
			capture_value = (0xFFFF - _readvalue1_TIM3) + _readvalue2_TIM3;	//0xFFFF : Timer2�� period
		}
		
		/* Frequency computation */ 
		pulse_frequecy_TIM3 = ((cx_float32_t)(72000000/2400) / capture_value);	//2400 : Timer2�� TIM_Prescaler
		
		put_pulse_frequency(pulse_frequecy_TIM3, 3);
		if(_flag_worker_state==CX_TRUE)	put_pulse_frequency(pulse_frequecy_TIM3, 2);//20241211
	
		_capture_number_TIM3 = 0;	
	}
}

#endif


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
			_freq_state 	= IDLE;
			_freq_count		= 0;

			_freq_data		= gu32_Freq;
			
			if(_freq_data==301 || _freq_data == 299) _freq_data = 300; // 3Hz

			if ( (291>_freq_data) || (309<_freq_data) )
			{
				_freq_fail_state = CX_TRUE;
			}
			else 
			{
				_freq_fail_state = CX_FALSE;
			}
		}
	}
}

static void control_impulse_voltage_input (void)
{
	//-----------------------------------------------------------------------
	//if (CX_FALSE ==_flag_get_impulse_voltage)	//190us, 0.1ms
	//{
	//	return;
	//}
	//-----------------------------------------------------------------------		
	if (CX_FALSE ==_impulse_voltage_measure_run)	//Frq 핀 rising edge 시
	{
		return;
	}
	//-----------------------------------------------------------------------		
	static cx_uint_t _impulse_measure_count          	= 0u;
	static cx_bool_t _active_measure_impulse_voltage	= 0u;
	
    //edge후 바로 정펄스 측 clear핀 제어
	GPIO_O_IMPULSE_CLR1(0); 	//정펄스
	//부펄스
	if(_impulse_measure_count>=25)   	//2.5ms
	{
		GPIO_O_IMPULSE_CLR2(0);	//부펄스
	} 


	if(_impulse_measure_count>=100)		//0.05ms * 200 = 10ms
	{		
		_active_measure_impulse_voltage = CX_TRUE;
		GPIO_O_IMPULSE_CLR1(1); 	
		GPIO_O_IMPULSE_CLR2(1);
	}
	else 
	{
		_impulse_measure_count++;
		_active_measure_impulse_voltage = CX_FALSE;
	}	
	
	get_input_data_impulse_voltage(_active_measure_impulse_voltage, _impulse_measure_count);
	
	//-----------------------------------------------------------------------
	if(_active_measure_impulse_voltage == CX_TRUE)
	{
		_impulse_measure_count = 0u;
				
		_impulse_voltage_measure_run = CX_FALSE;	//clear
	}	
//	_flag_get_impulse_voltage 	= CX_FALSE;	//clear
}


static void control_output_voltage_input(void)
{
	cx_bool_t flag_count_clear = CX_FALSE;

	
	//-----------------------------------------------------------------------
	if (CX_FALSE ==_flag_get_output_voltage)	
	{
		return;
	}
	//-----------------------------------------------------------------------
	
//	if(_boot_state == CX_FALSE)	//부팅된 후 6초 이후부터 watch // 6초
	{
//		flag_count_clear = CX_TRUE;
	}	
	
	get_input_data_output_voltage(CX_FALSE);

	//-----------------------------------------------------------------------
	_output_voltage_measure_run = CX_FALSE;
	
	_flag_get_output_voltage 	= CX_FALSE;
	
	flag_count_clear = CX_FALSE;
}

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
	debug_printf("\tid        = %d \n", id        );
	debug_printf("\ttimestamp = %d \n", timestamp );

	//-----------------------------------------------------------------------
	//debug_printf("\tpayload[ 0] Hotstandby              = 0x%02x \n", payload[ 0]);
	//debug_printf("\tpayload[ 1] FAIL                    = 0x%02x \n", payload[ 1]);
	debug_printf("\tpayload[ 2] Impulse_voltage_plus 1  = 0x%02x \n", payload[ 2]);
	debug_printf("\tpayload[ 3] Impulse_voltage_plus 2  = 0x%02x \n", payload[ 3]);
	debug_printf("\tpayload[ 4] Impulse_voltage_minus 1 = 0x%02x \n", payload[ 4]);
	debug_printf("\tpayload[ 5] Impulse_voltage_minus 2 = 0x%02x \n", payload[ 5]);
	debug_printf("\tpayload[ 6] Output_voltage 1        = 0x%02x \n", payload[ 6]);
	debug_printf("\tpayload[ 7] Output_voltage 2        = 0x%02x \n", payload[ 7]);
	debug_printf("\tpayload[ 8] AC_voltage 1            = 0x%02x \n", payload[ 8]);
	debug_printf("\tpayload[ 9] AC_voltage 2            = 0x%02x \n", payload[ 9]);
	debug_printf("\tpayload[10] Tx_Current 1            = 0x%02x \n", payload[10]);
	debug_printf("\tpayload[11] Tx_Current 2            = 0x%02x \n", payload[11]);
	debug_printf("\tpayload[12] Tx_frequency 1          = 0x%02x \n", payload[12]);
	debug_printf("\tpayload[13] Tx_frequency 2          = 0x%02x \n", payload[13]);
	debug_printf("\tpayload[14] SP                      = 0x%02x \n", payload[14]);
	debug_printf("\tpayload[15] SP                      = 0x%02x \n", payload[15]);
	debug_printf("\tpayload[16] SP                      = 0x%02x \n", payload[16]);
	debug_printf("\tpayload[17] SP                      = 0x%02x \n", payload[17]);
	debug_printf("\tpayload[18] SP                      = 0x%02x \n", payload[18]);
	debug_printf("\tpayload[19] SP                      = 0x%02x \n", payload[19]);
	debug_printf("\tpayload[20] SP                      = 0x%02x \n", payload[20]);
	debug_printf("\tpayload[21] SP                      = 0x%02x \n", payload[21]);
	debug_printf("\tpayload[22] SP                      = 0x%02x \n", payload[22]);
	debug_printf("\tpayload[23] SP                      = 0x%02x \n", payload[23]);
	debug_printf("\tpayload[24] SP                      = 0x%02x \n", payload[24]);
	debug_printf("\tpayload[25] SP                      = 0x%02x \n", payload[25]);
}                                           

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
	
	cx_byte_t hotstandby = 0;
	cx_byte_t fail = 0;
	//-----------------------------------------------------------------------
	
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  output_voltage;
	cx_uint_t  ac_voltage;
	cx_uint_t  tx_current;
	cx_uint_t  tx_frequency;

	
	
	impulse_voltage_plus  = get_imnpulse_voltage_plus_value();
	impulse_voltage_minus = get_imnpulse_voltage_minus_value();
	
	output_voltage = get_output_voltage_value();
	ac_voltage = get_ac_voltage_value();
	
	tx_current = (cx_uint_t)(get_tx_current_value())/10;	
	tx_frequency = (cx_uint_t)(get_3hz_frequency_FREQ_value()*100);
	
	//-----------------------------------------------------------------------
	message_size = sizeof(message);
		
	memset (message, 0x00, message_size);
	
	payload = &message[6];


	
		//-----------------------------------------------------------------------
	memcpy(&message[0], 0x00      , 1);
	memcpy(&message[1], &id       , 1);
	memcpy(&message[2], &timestamp, 4);

	//-----------------------------------------------------------------------
	payload[ 0] = hotstandby;
	payload[ 1] = fail;
	//정펄스
	payload[ 2] = ((impulse_voltage_plus &0xFF00)>>8);
	payload[ 3] = ( impulse_voltage_plus &0x00FF);
	//부펄스
	payload[ 4] = ((impulse_voltage_minus &0xFF00)>>8);
	payload[ 5] = ( impulse_voltage_minus &0x00FF);
	//출력전압
	payload[ 6] = ((output_voltage &0xFF00)>>8);
	payload[ 7] = ( output_voltage &0x00FF);
	//AC 전압
	payload[ 8] = ((ac_voltage &0xFF00)>>8);
	payload[ 9] = ( ac_voltage &0x00FF);
	//주파수
	payload[10] = ((tx_frequency &0xFF00)>>8);
	payload[11] = ( tx_frequency &0x00FF);
	//평균전류
	payload[12] = ((tx_current &0xFF00)>>8);
	payload[13] = ( tx_current &0x00FF);

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
	if (_flag_transmit==CX_FALSE) // 
	{
		return;
	}
		
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  output_voltage;
	cx_uint_t  ac_voltage;
	cx_uint_t  tx_current;
	cx_uint_t  tx_frequency;
	cx_uint_t  i; // pws
	uint8_t fcs_value; // pws

	impulse_voltage_plus  = get_imnpulse_voltage_plus_value();
	impulse_voltage_minus = get_imnpulse_voltage_minus_value();

	output_voltage = get_output_voltage_value();
	ac_voltage = get_ac_voltage_value();

	tx_current = (cx_uint_t)(get_tx_current_value())/10;	
	tx_frequency = _freq_data;
	//////////////////////////////////////////////////////////////
	/*
	if(_flag_worker_state==CX_FALSE)
	{
		impulse_voltage_plus=0;
		impulse_voltage_minus=0;
		tx_frequency=0;
		tx_current=0;
	}
	*/
	_com1_tx_buffer[0] = 0x02;
	ac_voltage = ac_voltage/10;
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[1+i]	=(ac_voltage/POW(10,2-i))%10+'0';
	}
	output_voltage = output_voltage/10;
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[4+i]	=(output_voltage/POW(10,2-i))%10+'0';
	}
	impulse_voltage_plus = impulse_voltage_plus/10;
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[7+i]	=(impulse_voltage_plus/POW(10,2-i))%10+'0';
	}
	
	impulse_voltage_minus = impulse_voltage_minus/10;
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[10+i]	=(impulse_voltage_minus/POW(10,2-i))%10+'0';
	}
	
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[13+i]	=(tx_frequency/POW(10,2-i))%10+'0';
	}
	
	for(i=0;i<3;i++)
	{
		_com1_tx_buffer[16+i]	=(tx_current/POW(10,2-i))%10+'0';
	}
	
	fcs_value = fcs(&_com1_tx_buffer[1], 18);
	_com1_tx_buffer[19]=fcs_conv(fcs_value>>4);
	_com1_tx_buffer[20]=fcs_conv(fcs_value&0x0f);
	_com1_tx_buffer[21]=0x03;
	_com1_tx_buffer[22]=0x0D;
	_com1_tx_buffer[23]=0x0A;
	
	
	put_str(_com1_tx_buffer);
	//_timer_count_transmit = CX_FALSE;
	_flag_transmit=CX_FALSE;

}


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void control_watch (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_watch)	//300ms 주기
	{
		return;
	}
	//-----------------------------------------------------------------------	
	cx_bool_t watchdog_fail;
	cx_bool_t fail_max_count = 10u;
	static cx_uint_t watchdog_fail_count 	= 0u;
	//-----------------------------------------------------------------------	
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
	if(_operating_time_second>=1)
	{
		_check_rising_edge_tim2++;
		_check_rising_edge_tim3++;
		
		if(_check_rising_edge_tim2>=fail_max_count)
		{
			standby_clear_measured_data();
		}	
		
		if(_check_rising_edge_tim3>=fail_max_count)
		{
			clear_measured_data();
		}
		
		if(_check_rising_edge_tim2>=0xFF) _check_rising_edge_tim2 = fail_max_count+1;
		if(_check_rising_edge_tim3>=0xFF) _check_rising_edge_tim3 = fail_max_count+1;
	}	
	//-----------------------------------------------------------------------
	if(_boot_state == CX_TRUE)	//전원 On 후 6초 이후부터 watch //6초
	{
		//analog_data_control_watch();
	
		//580V 출력전압 감시..
		_health_output_voltage = output_voltage_control_watch();
	}
			
/*	
	if(_operating_time_second > 3u)	//주계된 후 3초 이후 
	{
		//AC전압 감시
		_health_ac_voltage = ac_voltage_control_watch();
	}
*/
	//-----------------------------------------------------------------------
	_flag_control_watch = CX_FALSE;
}

static void control_input(void)
{
	//-----------------------------------------------------------------------	
	if (6u >= _operating_time_second)	//전원 ON 후 3초후 READ 시작
	{
		return;
	}	
	if (CX_FALSE==_flag_control_input)	//50ms 주기
	{
		return;
	}
	//-----------------------------------------------------------------------
	#if 0	
	cx_bool_t fail;

	fail = CX_FALSE;

	// 1번,2번 절체 스위치 둘다 입력 받아서 고장처리를 하게되면 사용
	if ( _input_manual_switch_1==_input_manual_switch_2 )
	{
		fail = CX_TRUE;
	}

	if ( CX_FALSE==fail )
	{
		io_module_port_di->fail.value = CX_FALSE;
		io_module_port_di->fail.count = 0u;
	}
	else
	{
		if (io_module_port_di->fail.count<CHATTERING_COUNT)
		{
			io_module_port_di->fail.count++;
		}
		else
		{
			// 절체 스위치 입력 불량
			io_module_port_di->fail.value = CX_TRUE;
		}
	}
	#endif
	//-----------------------------------------------------------------------
	cx_bool_t input;

	cx_uint_t input_chattering_count = 10u;
	static cx_uint_t _io_input_count = 0u;
	static cx_uint_t _io_input_value = 0u;


	input = CX_FALSE;
	if( CX_FALSE == GPIO_I_Manual_Switch_INPUT())   //반대로 들어옴, 평상시 high
	{
		input = CX_TRUE;
	}


	if (CX_TRUE==input)
	{
		if (_io_input_count <= input_chattering_count)
		{
			_io_input_count++;
		}
		else
		{
			_io_input_value = CX_TRUE;
		}
	}
	else
	{
		_io_input_count = 0u;
		_io_input_value = CX_FALSE;
			
	}

	//-----------------------------------------------------------------------
	// 입력정보 반영
	_input_manual_switch = _io_input_value;


	//-----------------------------------------------------------------------
	_flag_control_input = CX_FALSE;

}


static void control_output(void) //  주부계 Switch
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_control_output)
	{
		return; 
	}
	//-----------------------------------------------------------------------
	// 출력정보 갱신
	// 12V 계전기 출력은 OFF->ON 시 delay 추가
	//static cx_uint_t _count_active_smps_output = 0u;
	
//	cx_uint_t active_output_maxcount 		= 54u;	//27ms (500us*54)
//	cx_uint_t active_smps_output_maxcount 	= 10u;	//5ms (500us*10)
//	cx_uint_t active_output_maxcount 		= 40u;	//20ms
	//cx_uint_t active_smps_output_maxcount 	= 0u;

	if ( CX_TRUE==_flag_worker_state )  
	{
		_output_smps_output_control = CX_TRUE;
		if(_flag_worker_health_state == CX_FALSE)
		{
			_output_smps_output_control = CX_FALSE;
		}

	}
	else
	{
	//	_count_active_smps_output 	= 0u;	//count clear
	}
        
	if(_flag_worker_health_state == CX_FALSE)
	{
		_output_smps_output_control = CX_FALSE;
	}
	
	//-----------------------------------------------------------------------
	GPIO_O_CPU_LED_STATUS 	(CX_TRUE == _application_halt ? 0u : 1u);
	
	//GPIO_O_MCU_SMPS_Control		(_output_smps_output_control);
	if(GPIO_TEST_MODE() == CX_TRUE)
	{
		GPIO_O_SwitchOver_Control       (_output_smps_output_control); // PWS 절체 사용시 주석 제거
	}
	
	//GPIO_O_MCU_TIM_State		(_output_smps_output_control);
	
	//-----------------------------------------------------------------------	
	_flag_control_output = CX_FALSE;
}	

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void application_show_version (void)
{
	static cx_char_t* APPLICATION_NAME    = "Intergrated Impulse Circuit Device - Transmitter";
	static cx_char_t* APPLICATION_VERSION = "0.0.0.1";


	debug_printf("\n");
	debug_printf("%s \n", APPLICATION_NAME);
	debug_printf("Version %s (compiled: %s, %s)\n", APPLICATION_VERSION, __DATE__, __TIME__);
	debug_printf("\n");
	debug_printf("Copyright (c) 2022, Yoo-Kyung Control co., ltd.\n");
	debug_printf("All rights reserved.\n");
	debug_printf("\n");
}

//===========================================================================
void io_control_initialize (void)
{
	//-----------------------------------------------------------------------
	GPIO_O_SwitchOver_Control(1);	
	GPIO_O_MCU_TM_State(1);
	GPIO_O_MCU_TM_Data(1);
	
	GPIO_O_CPU_LED_STATUS (0);
	
}

void io_control_show(void)
{
	debug_printf("# IO CONTROL\n");

	debug_printf("\tGPIO INPUT: Manual Switch     	 = %d \n", _input_manual_switch);
	debug_printf("\tGPIO INPUT: WATCHDOG INPUT     	 = %d \n", !(_watchdog_fail));
	
	debug_printf("\tGPIO OUTPUT: SwitchOver Control	 = %d \n", _output_switchover_control);
	debug_printf("\tGPIO OUTPUT: SMPS Output Control = %d \n", _output_smps_output_control);
	
	debug_flush();
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
static void update_istream (void)//최신 업데이트 입력 버퍼
{
	peer_update();

	istream_check_timeout(&_istream1);
//	istream_check_timeout(&_istream2);
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

	
	rx_size = com_recv_buffer(COM1, _nb_rx_buffer, sizeof(_nb_rx_buffer));
	if (rx_size > 0u)
	{
		if (CX_TRUE==enable)
		{
			istream_packet_push(&_istream1, _nb_rx_buffer, rx_size);
		}
	}

	//===========================================================================		
}

#if 0
static void check_watchdog_input_data (void)
{
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_check_watchdog_input_data)	//500ms 주기
	{
		return;
	}
	
	//-----------------------------------------------------------------------
	cx_bool_t watchdog_fail;
	cx_bool_t fail_max_count 				= 5u;
	static cx_uint_t watchdog_fail_count 	= 0u;
	
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
	cx_uint16_t portd_value = 0; //FND9~FND15 //PD0~PE7
	cx_uint16_t porte_value = 0; //FND0~FND8  //PE7~PE14
	volatile cx_uint_t i=0;
	cx_uint16_t ReadValue_portD;
	cx_uint16_t ReadValue_portE;
	
	ReadValue_portD = GPIO_ReadInputData(GPIOD);   // 쓰기 전에 기존값 읽기 
	
	portd_value |= select;    //PD0~PD7
	GPIO_Write(GPIOD,(ReadValue_portD & 0xff00) | portd_value);
	
	ReadValue_portE = GPIO_ReadInputData(GPIOE);   // 쓰기 전에 기존값 읽기 
	
	porte_value |= fndvalue<<7; //PE7~PE14
	GPIO_Write(GPIOE,(ReadValue_portE & 0x807f) | porte_value);
	
	for (i=0;i<0x10;i++);
//	GpioD->DATA = GpioD->DATA & 0xff00;  
	GPIO_Write(GPIOD,(ReadValue_portD & 0xff00));	//FND9~FND15 clear
	
//	GpioE->DATA = GpioE->DATA & 0x807f;  
	GPIO_Write(GPIOE,(ReadValue_portE & 0x807f));	//FND0~FND8 clear

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

	if ( CX_TRUE==_flag_worker_state )	//주계만 ON
	{
		_flag_active_display = CX_TRUE;
		
		fnd_output_data (0, _fnd0, 5);
		fnd_output_data (1, _fnd1, 5);
	}
	else _flag_active_display = CX_FALSE;
}

//-----------------------------------------------------------------------
void input_front_mode_button(void)
{		

			
	if(CX_TRUE == _status_mode_switch)
	{
		_flag_active_mode_switch = CX_TRUE;
			
		_status_mode_switch = CX_FALSE;
		_count_active_fnd_display = 0u;	//누를때마다 FND off를 위한 카운트 클리어, 시간지나서 off시키기 
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
	_input_mode_sw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
	
	if(_input_mode_sw == 1)
	{
		if(cur_debounce_count-pre_debounce_count > 200u)
		{
	
			_status_mode_switch = CX_TRUE;
			_display_sequence++;
			if(_display_sequence>=4) _display_sequence = 1u;
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
	if (CX_FALSE ==_flag_make_display_data)	//500ms 주기
	{
		return;
	}

	//-----------------------------------------------------------------------
	char first_line   [5]  = {0,};	
	char second_line  [5]  = {0,};	

	cx_uint_t  fnd_change_max_count 	  = 10u;	//500ms*10 = 5초
	
	cx_uint_t  first_line_display_length  = 0u;
	cx_uint_t  second_line_display_length = 0u;
	
	cx_uint_t  impulse_voltage_plus;
	cx_uint_t  impulse_voltage_minus;
	cx_uint_t  output_voltage;
	cx_uint_t  ac_voltage;
	cx_uint_t  int_tx_current;
	cx_float_t  tx_current;
	cx_float_t  tx_frequency;
	
	impulse_voltage_plus  = get_imnpulse_voltage_plus_value()/10;	//3자리
	impulse_voltage_minus = get_imnpulse_voltage_minus_value()/10;	//3자리
	
	output_voltage = get_output_voltage_value()/10;	//3자리
	ac_voltage = get_ac_voltage_value()/10;			//3자리
	
	int_tx_current 		= get_tx_current_value()/10;	//??단위는 다시 확인
    tx_current = (cx_float_t)int_tx_current/10;
	
	tx_frequency 	= (cx_float_t)_freq_data/100;
    	
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
			
			sprintf(first_line ,"%d", output_voltage);
			first_line_display_length = strlen(first_line);
			first_line[first_line_display_length] = 'V';
			
			sprintf(second_line,"%d", ac_voltage);
			second_line_display_length = strlen(second_line);
			second_line[second_line_display_length] = 'V';
			
			break;			
		case 3u:
		
			sprintf(first_line,"%.1f",tx_current);
			first_line_display_length = strlen(first_line);
			first_line[first_line_display_length] = 'A';
			
			sprintf(second_line,"%.2f",tx_frequency);
			second_line_display_length = strlen(second_line);
			second_line[second_line_display_length] = 'H';
		
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
		_count_active_fnd_display++;

		if (_count_active_fnd_display > fnd_change_max_count)	//5초 후
		{
			_display_sequence = 1u;
		}
/*
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
*/				
	}
	else _count_active_fnd_display = 0u;
	
	if(_display_sequence == 0u) _display_sequence = 1u;
	
	//-----------------------------------------------------------------------
	_flag_make_display_data = CX_FALSE;
}


static void check_fault (void)
{	
	cx_bool_t worker_fail   = CX_FALSE; 

//---------------------------------------------------------------------------
// TODO: RELEASE version
//---------------------------------------------------------------------------    
    // GO HALT
    if(_operating_time_second >= 10u)
	{
		if(CX_TRUE == _watchdog_fail)
		{
			debug_printf("# WATCHDOG FAIL \n");
			_flag_worker_health_state=CX_FALSE;
			worker_fail = CX_TRUE; 
		}

	}			
	//---------------------------------------------------------------------------   
//	if(CX_TRUE == _flag_worker_state)
//	{
#if 1	
		if(_boot_state == CX_TRUE)	//주계된 후 6초 이후  // 6초
		{
			if (_freq_health_fail == CX_TRUE)	//주파수 고장
			{
				//debug_printf("# FREQUENCY FAIL \n");
				_flag_worker_health_state=CX_FALSE;	

				worker_fail = CX_TRUE;

				_fail_condition_frequency = CX_TRUE;
			}
            else _fail_condition_frequency = CX_FALSE;
			
			if(CX_FALSE == _health_output_voltage)
			{
				//debug_printf("# Output Voltage FAIL \n");				
				_flag_worker_health_state=CX_FALSE;
					
				worker_fail = CX_TRUE;
				
				_fail_condition_outputvoltage = CX_TRUE;
			}
            else _fail_condition_outputvoltage = CX_FALSE;
			
			//debug_flush();	//위쪽 디버그 플면 같이 주석 풀자
		}
#endif
	//}
	//---------------------------------------------------------------------------    
	debug_flush();
	if(_flag_worker_health_state == CX_FALSE) worker_fail = CX_TRUE;
        
	if(worker_fail == CX_FALSE) 
	{
		_application_halt = CX_FALSE;
	}
	else _application_halt = CX_TRUE;
		
}

void application_show_memory (void)
{
	cx_uint_t global_variable_size = 0u;

	debug_printf("# GLOBAL VARIABLE SIZE\n");
	debug_printf("\tsizeof(_com1_fifo_tx_buffer      )=%d\n",sizeof(_com1_fifo_tx_buffer      )); global_variable_size += sizeof(_com1_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com1_fifo_rx_buffer      )=%d\n",sizeof(_com1_fifo_rx_buffer      )); global_variable_size += sizeof(_com1_fifo_rx_buffer      );
//	debug_printf("\tsizeof(_com2_fifo_tx_buffer      )=%d\n",sizeof(_com2_fifo_tx_buffer      )); global_variable_size += sizeof(_com2_fifo_tx_buffer      );
//	debug_printf("\tsizeof(_com2_fifo_rx_buffer      )=%d\n",sizeof(_com2_fifo_rx_buffer      )); global_variable_size += sizeof(_com2_fifo_rx_buffer      );
	debug_printf("\tsizeof(_com3_fifo_tx_buffer      )=%d\n",sizeof(_com3_fifo_tx_buffer      )); global_variable_size += sizeof(_com3_fifo_tx_buffer      );
	debug_printf("\tsizeof(_com3_fifo_rx_buffer      )=%d\n",sizeof(_com3_fifo_rx_buffer      )); global_variable_size += sizeof(_com3_fifo_rx_buffer      );
	
	debug_printf("\tsizeof(_com1_stream_rx_bsb_buffer)=%d\n",sizeof(_com1_stream_rx_bsb_buffer)); global_variable_size += sizeof(_com1_stream_rx_bsb_buffer);
	
	debug_printf("\tsizeof(_analog_data          	 )=%d\n",sizeof(_analog_data          	  )); global_variable_size += sizeof(_analog_data          );		
	debug_printf("\tsizeof(_peer_connection          )=%d\n",sizeof(_peer_connection          )); global_variable_size += sizeof(_peer_connection          );		
	debug_printf("\tsizeof(_equipment                )=%d\n",sizeof(_equipment                )); global_variable_size += sizeof(_equipment                );
	debug_printf("\tsizeof(_config                   )=%d\n",sizeof(_config                   )); global_variable_size += sizeof(_config                   );
	
	debug_printf("\tTOTAL = %d \n", global_variable_size);
}

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
    TIM_Cmd(TIM5, ENABLE);
	//TIM_Cmd(TIM6, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	

	//-----------------------------------------------------------------------
	debug_printf("# INITIALIZE DATA \n");
	peer_initialize();
	equipment_initialize();
	frequency_data_initialize();
		

	//-----------------------------------------------------------------------
	debug_printf("# ENABLE EXTERNAL-WATCHDOG TIMER \n");
	_watchdog_run = CX_TRUE;
	
	
	//-----------------------------------------------------------------------
	debug_printf("# INITIALIZE IO-CONTROL \n");
	io_control_initialize();

	
	//-----------------------------------------------------------------------
	debug_printf("# INITIALIZE HOTSTANDBY \n");
	hotstandby_initialize();
	
	
	//-----------------------------------------------------------------------
	debug_printf("# ENABLE FND DISPLAY \n");	
	TIM_Cmd(TIM7, ENABLE);
	display_maked_fnd_data   ("8888",  _fnd0, 5); 
	display_maked_fnd_data   ("8888",  _fnd1, 5); 

	
	//-----------------------------------------------------------------------
	return CX_TRUE;
}



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
void hw_driver_initialize (void)
{
	// hardware 초기화 전
	io_fifo_initialize(&_com_fifo[COM1], _com1_fifo_rx_buffer, sizeof(_com1_fifo_rx_buffer), _com1_fifo_tx_buffer, sizeof(_com1_fifo_tx_buffer));
	io_fifo_initialize(&_com_fifo[COM3], _com3_fifo_rx_buffer, sizeof(_com3_fifo_rx_buffer), _com3_fifo_tx_buffer, sizeof(_com3_fifo_tx_buffer));
	
	istream_initialize(&_istream1, 1, _com1_stream_rx_bsb_buffer, sizeof(_com1_stream_rx_bsb_buffer), 30u);
	
	debug_shell_initialize();
}


void hw_gpio_initialize (void)
{
	//-----------------------------------------------------------------------
	GPIO_O_EXTERNAL_WATCHDOG_CLOCK (0);

	GPIO_O_CPU_LED_STATUS(0);
	//-----------------------------------------------------------------------
	GPIO_O_COWORKER_DO0 (0);
	GPIO_O_COWORKER_DO1 (0);
	GPIO_O_COWORKER_DO2 (0);
	GPIO_O_COWORKER_DO3 (0);
	GPIO_O_COWORKER_DO4 (0);
	GPIO_O_COWORKER_DO5 (0);
	GPIO_O_COWORKER_DO6 (0);
	//GPIO_O_COWORKER_DO7 (0);
	//-----------------------------------------------------------------------
	//GPIO_O_MCU_SMPS_Control(1);	
	GPIO_O_SwitchOver_Control (1);
	GPIO_O_MCU_TM_State(1);
	GPIO_O_MCU_TM_Data(1);
	//-----------------------------------------------------------------------
	GPIO_O_IMPULSE_CLR1 (1); 
    GPIO_O_IMPULSE_CLR2 (1); 

	
	//-----------------------------------------------------------------------
}


void application_halt (void)
{
	//---------------------------------------------------------------------------
#if 1
	debug_printf("# HALT \n");
	
	_application_halt = CX_TRUE;
    
    //FND OFF
    TIM_Cmd(TIM7, DISABLE);
    
    clear_measured_data();
    standby_clear_measured_data();
    
    hw_gpio_initialize();
 
	while(1)
	{
		// 입력 
		control_input ();
		
		// Transmit
		update_ostream();
		transmit ();
	
		// 디버깅 쉘
		debug_shell();	
		
		
		
/*		
		//부팅 시 DIDO pair 비교 실패로 HALT 발생하는 경우 MCU 리셋
		if(_operating_time_second < 10) //부팅 후 10초 이내라면
		{
			if (CX_TRUE==pair_dido_heartbeat_fault_update())
			{
				debug_printf("# BOOT - Pair DIDO heartbeat fault RESET \n");
				delay_msec(100);
				__NVIC_SystemReset();	
			}
			else;
		}
*/			
	}
#endif	
}

//main loop for Output PULSE 
void application_run_ouptut_pulse (void)
{
	_watchdog_run = CX_TRUE;
	
	while (1)
  	{	
		//WatchDog
//		check_watchdog_input_data();	// 외부 와치독 입력 체크
	}
}

//---------------------------------------------------------------------------
//main loop for SUPERVISOR
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

	
	while (1)
  	{
    	t2 = _operating_time_second;
		
		//recrption
		//reception (CX_TRUE);
		//update_istream();
		//update_equipment();
		//3hz Freq핀 주파수 감시
		//Feq_health_check();
		
		//아날로그 데이터 감시

		control_watch();
		
		// 입력 
		//control_input ();
		
		//Measure ADC
		control_output_voltage_input();
	
		
		// fail 체크, 이중계
		//check_watchdog_input_data();	// 외부 와치독 입력 체크
		check_fault ();
//		hotstandby_update();
		
		//출력
		control_output();
		
		// Transmit
		//update_ostream();
		transmit ();
		
		//check mode switch
		check_front_button();
		
		//make Display Data
		make_fnd_data();
		
		// 디버깅 쉘
//		debug_shell();
       		
#if 1
//---------------------------------------------------------------------------
// TODO: RELEASE version
//---------------------------------------------------------------------------
		if (CX_TRUE==get_worker_halt())
		{
			return;
		}
#else
//---------------------------------------------------------------------------
// TODO: DEBUG version
//---------------------------------------------------------------------------
		if (CX_TRUE==get_worker_halt())
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



