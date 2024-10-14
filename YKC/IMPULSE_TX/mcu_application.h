#ifndef INCLUDED__MCU_APPLICATION__H
#define INCLUDED__MCU_APPLICATION__H



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_uint_t _application_halt;

//===========================================================================
MCU_API void delay_msec (cx_uint_t msec);
MCU_API void delay_sec (cx_uint_t sec);

MCU_API void timer_500usec_irq_handler  (void);
MCU_API void timer_watchdog_irq_handler (void);

MCU_API void calculate_inputcapture_TIM2 (void);
MCU_API void calculate_inputcapture_TIM3 (void);

MCU_API void output_voltage_measure_irq_handler (void);

MCU_API void rising_edge_flag_TIM2 (void);
MCU_API void rising_edge_flag_TIM3 (void);

MCU_API void hw_driver_initialize   	(void);
MCU_API void hw_gpio_initialize     	(void);
MCU_API void application_run         	(void);
MCU_API void application_run_ouptut_pulse (void);

MCU_API void application_show_version (void);
MCU_API void application_show_memory  (void);
MCU_API void application_halt         (void);

MCU_API cx_bool_t application_initialize   (void);

MCU_API void message_show(void);
MCU_API void io_control_show(void);

MCU_API void get_impulse_voltage_irq_handler (void);

MCU_API void check_watchdog_input_data(void);
MCU_API void check_front_button(void);
MCU_API void input_front_mode_button(void);

MCU_API void display_fnd (void);

MCU_API void output_fnd_display(void);
MCU_API void check_impuls_freqout (void);

#endif
