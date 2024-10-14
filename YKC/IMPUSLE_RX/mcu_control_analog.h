#ifndef INCLUDED__MCU_CONTROL_ANALOG__H
#define INCLUDED__MCU_CONTROL_ANALOG__H


typedef struct _sampling_data_t
{
	cx_bool_t	health;
	
	cx_bool_t  fail_value;
	cx_uint8_t fail_count;
	cx_bool_t  period_end;
	cx_uint8_t period_count;

	cx_float_t sampling[5];
}
sampling_data_t;

typedef struct _pulse_frequency_t
{
	cx_bool_t				health;
	
	sampling_data_t			sampling_pulse_frequency;
}
pulse_frequency_t;

typedef struct _analog_data_t
{
	pulse_frequency_t	pulse_frequency;
}
analog_data_t;



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API analog_data_t _analog_data;

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void frequency_data_initialize (void);
MCU_API void put_current_value(void);
MCU_API void get_input_data_impulse_voltage(cx_bool_t active_measure, cx_uint_t count_value);
MCU_API void get_input_data_relay_voltage(cx_bool_t active_relay_measure, cx_uint_t relay_count_value);

MCU_API void put_pulse_frequency (cx_float_t freq);
MCU_API void get_pulse_frequency (sampling_data_t* sampling_data, cx_uint8_t pulse_count);

MCU_API void DMA_initial(void);
MCU_API void active_flag_frequency_rising_edge(void);
MCU_API void analog_100us_tim5_irq(void);
MCU_API cx_uint_t get_imnpulse_voltage_plus_value(void);
MCU_API cx_uint_t get_imnpulse_voltage_minus_value(void);
MCU_API cx_uint_t get_relay_voltage_v1_value(void);
MCU_API cx_uint_t get_relay_voltage_v2_value(void);
MCU_API cx_uint_t get_rx_current_value(void);
MCU_API cx_float_t get_rx_3hz_frequency_value(void);

MCU_API void clear_measured_data(void);

MCU_API void analog_data_control_watch(void);
MCU_API void input_frequency_show(void);
MCU_API void measured_analog_data_show(void);

#endif




