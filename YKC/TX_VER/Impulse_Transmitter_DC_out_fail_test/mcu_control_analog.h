#ifndef INCLUDED__MCU_CONTROL_ANALOG__H
#define INCLUDED__MCU_CONTROL_ANALOG__H


typedef struct _sampling_data_t
{
	cx_bool_t	health;
	
	cx_bool_t  fail_value;
	cx_uint8_t fail_count;
	cx_bool_t  period_end;
	cx_uint8_t period_count;

	cx_float_t sampling[10];
}
sampling_data_t;

typedef struct _pulse_frequency_t
{
	cx_bool_t				health;
	
	sampling_data_t			sampling_pulse_frequency_CLOCK;
	sampling_data_t			sampling_pulse_frequency_FREQ;
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
MCU_API void DMA_initial(void);

MCU_API void analog_data_control_watch(void);

MCU_API void get_input_data_output_voltage(cx_bool_t clear_count);
MCU_API void get_input_data_impulse_voltage(cx_bool_t active_measure, cx_uint_t count_value);
MCU_API void get_AC_voltage_irq_handler (void);
MCU_API void get_tx_current_irq_handler (void);

MCU_API void put_pulse_frequency (cx_float_t freq, cx_bool_t timer);
MCU_API void get_pulse_frequency (sampling_data_t* sampling_data, cx_uint8_t pulse_count,cx_bool_t timer);

MCU_API void set_flag_rising_edge_pin_freq(void);
MCU_API void clear_impulse_data(void);

MCU_API cx_bool_t output_voltage_control_watch(void);
MCU_API cx_bool_t ac_voltage_control_watch(void);

MCU_API cx_uint16_t get_ac_voltage_value(void);
MCU_API cx_uint_t get_output_voltage_value(void);
MCU_API cx_uint_t get_imnpulse_voltage_plus_value(void);
MCU_API cx_uint_t get_imnpulse_voltage_minus_value(void);
MCU_API cx_uint_t get_tx_current_value(void);
MCU_API cx_float_t get_3hz_frequency_CLOCK_value(void);
MCU_API cx_float_t get_3hz_frequency_FREQ_value(void);

MCU_API void measured_analog_data_show(void);
MCU_API void input_frequency_show(void);
MCU_API void standby_clear_measured_data(void);
MCU_API void clear_measured_data(void);

#endif


