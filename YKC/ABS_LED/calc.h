#ifndef CALC_H
#define CALC_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Global sensor/measurement variables (defined in calc.c) */
extern uint64_t measured_PWM_Cur;
extern uint16_t measured_TEMP_Val;
extern uint16_t measured_CDS_Val;
extern uint16_t measured_DC_Val;


extern volatile uint32_t adc_dma_values[NUM_ADC_CHANNELS];


extern uint16_t count_measure_temper;
extern uint32_t	temper_measure_sum;
extern uint32_t	_averageTemp_max_sampling;
extern float 	_voltage_Level_temp;
extern uint16_t count_measure_CDS;
extern uint32_t	CDS_measure_sum;
extern uint32_t	CDS_measure_avg;
extern uint32_t	_averageCds_max_sampling;
extern uint8_t current_ready;

extern uint16_t count_measure_DC;
extern uint32_t	DC_measure_sum;

extern uint32_t converted_value_cds;
extern uint32_t converted_value_temper;
extern uint32_t converted_value_tx_currnet;
extern uint32_t converted_value_DC_volatage;

//extern uint32_t test1 = 0;
//extern uint32_t test2 = 0;
//extern uint32_t test3 = 0;
extern uint32_t test4;
extern uint32_t test5;
extern uint32_t test6;
//extern uint32_t test7 = 0;
//extern uint32_t test8 = 0;



/* Calculation helpers */
float interpolate_mA(float avg_adc, float base_adc, float base_mA, float gain);
float calculate_70_duty_mA(float avg_adc);
float calculate_50_duty_mA(float avg_adc);
float calculate_40_duty_mA(float avg_adc);

/* Accessors */
uint64_t get_PWM_cur_Value(void);
uint16_t get_TEMP_Value(void);
uint16_t get_CDS_Value(void);
uint16_t get_DC_Value(void);

void put_PWM_current_Value(void);
void put_Temp_Value(void);
void put_DC_value(void);
void put_CDS_value(void);


#endif /* CALC_H */

