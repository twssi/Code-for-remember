#ifndef INCLUDED__MCU_APPLICATION__H
#define INCLUDED__MCU_APPLICATION__H

//===========================================================================
MCU_API void timer_500usec_irq_handler  (void);

MCU_API void timer_50usec_irq_handler (void);
MCU_API void measure_current_irq_handler (void);

MCU_API void control_input(void);

MCU_API void output_fnd_display(void);
MCU_API void check_input_data(void);
MCU_API void check_front_button(void);
MCU_API void input_front_mode_button(void);

MCU_API void hw_driver_initialize     (void);
MCU_API void hw_gpio_initialize       (void);
MCU_API void application_show_version (void);
MCU_API void application_show_memory  (void);
MCU_API void application_run          (void);
MCU_API void application_halt         (void);

MCU_API void message_show(void);
MCU_API void io_control_show(void);

MCU_API cx_bool_t application_initialize   (void);

MCU_API void rising_edge_flag_TIM3 (void);

MCU_API void calculate_inputcapture_TIM3 (void);

MCU_API cx_uint_t get_active_transmitter (void);

MCU_API void OnUart3_Recv (cx_uint8_t ch);
MCU_API void OnUart4_Recv (cx_uint8_t ch);

MCU_API void check_state_Transmitter(void);

MCU_API void get_impulse_voltage_irq_handler (void);
//MCU_API void put_data(cx_byte_t *str, cx_uint16_t len);

#endif
