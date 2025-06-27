/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>


#include "mcu.h"
#include "stm32f10x_init.h"
#include "stm32f10x_dma.h"
#define TEST_O 			GpioD->Bit.b8

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
analog_data_t   _analog_data;
DMA_InitTypeDef DMA_InitStructure;

//-----------------------------------------------------------------------	
#define SAMPLING_COUNT_OUTPUT_VOLTAGE		60u
#define OLD_SAMPLING_COUNT_OUTPUT_VOLTAGE	10u
//#define SAMPLING_COUNT_PULSE_FREQ			1u
#define SAMPLING_COUNT_PULSE_FREQ			10u
#define SAMPLING_COUNT_IMPULSE_VOLTAGE		3u
#define SAMPLING_COUNT_AC_VOLTAGE			200u
#define MEASURED_AC_VOLTAGE					200u
#define SAMPLING_COUNT_TX_CURRENT			100u

#define ADC_CHANNEL							5u

#define PIN_CLOCK 							3u
#define PIN_FREQ  							2u

#define COUNT_IMPULSE_Value					100u
//-----------------------------------------------------------------------	
static cx_uint16_t _ADC_ValueTab[ADC_CHANNEL] = {0,};

static cx_uint16_t _array_sampling_ac_sense[SAMPLING_COUNT_AC_VOLTAGE]		= {0, };
static cx_uint16_t _array_sampling_tx_current[SAMPLING_COUNT_TX_CURRENT] 	= {0, };
static cx_uint16_t 	_array_PA_0[COUNT_IMPULSE_Value]  = {0, };
static cx_uint16_t 	_array_PA_1[COUNT_IMPULSE_Value]  = {0, };

static cx_uint_t	_watch_output_voltage[SAMPLING_COUNT_OUTPUT_VOLTAGE] = {0u, };

static cx_bool_t 	_flag_get_tx_current = CX_FALSE; //timer2 rising edge(freq)/ for current value
//-----------------------------------------------------------------------	
//������
static cx_uint_t	_measured_output_voltage 		= 0u;
static cx_uint_t	_measured_impulse_voltage_plus 	= 0u;
static cx_uint_t	_measured_impulse_voltage_minus = 0u;

static cx_uint16_t 	_measured_AC_voltage 			= 0u;
static cx_uint_t 	_measured_tx_current 			= 0u;

static cx_float_t	_3hz_frequency_CLOCK			= 0;
static cx_float_t	_3hz_frequency_FREQ  			= 0;

static cx_uint32_t average_IMP_PLUS_value  = 0u;
static cx_uint32_t average_IMP_MINUS_value = 0u;

cx_uint_t		correction_plus_upper = 0u;
cx_float_t		measure_plus_1 		= 0;
cx_float_t		measure_plus_2 		= 0;

static cx_uint_t	test = 0u;
static cx_uint_t	test1 = 0u;
static cx_uint_t	test2= 0u;
static cx_uint_t	test3 = 0u;


//-----------------------------------------------------------------------	
//���з��� ��

static cx_float_t 	_voltage_Level_DC580  			= 0;
static cx_float_t 	_voltage_Level_impulse_plus  	= 0;
static cx_float_t 	_voltage_Level_impulse_minus  	= 0;
static cx_float_t 	_voltage_Level_tx_current		= 0;

/////////////////////////////////////////////////////////////////////////////
//===========================================================================

void frequency_data_initialize(void)
{
	//-----------------------------------------------------------------------	
	memset (&_analog_data, 0, sizeof(_analog_data));
	
}

//-----------------------------------------------------------------------	
void DMA_initial(void)
{
    /* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
    
	DMA_InitStructure.DMA_PeripheralBaseAddr = (cx_uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (cx_uint32_t)_ADC_ValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 5;
    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
    
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    //���ͷ�Ʈ
	/* Enable DMA1 Channel6 Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    
    /* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

//---------------------------------------------------------------------------
cx_uint16_t get_ac_voltage_value(void)
{	
	return _measured_AC_voltage;
}	

cx_uint_t get_output_voltage_value(void)
{
	return _measured_output_voltage;
}

cx_uint_t get_imnpulse_voltage_plus_value(void)
{
	return _measured_impulse_voltage_plus;
}

cx_uint_t get_imnpulse_voltage_minus_value(void)
{
	return _measured_impulse_voltage_minus;
}

cx_uint_t get_tx_current_value(void)
{	
	return _measured_tx_current;
}	

cx_float_t get_3hz_frequency_CLOCK_value(void)
{	
	return _3hz_frequency_CLOCK;
}	

cx_float_t get_3hz_frequency_FREQ_value(void)
{	
	return _3hz_frequency_FREQ;
}	

void set_flag_rising_edge_pin_freq(void) //riging edge after 2ms
{
	_flag_get_tx_current = CX_TRUE;
	
}

//---------------------------------------------------------------------------
void get_AC_voltage_irq_handler(void)
{                               
	static cx_uint8_t	_count_sampling_ac_voltage	= 0u;	
	static cx_uint32_t	_offset_value_ac_sense		= 0u;    
	
	static cx_uint8_t	_count_ac_sense_avreage    	= 0u;    
	static cx_uint32_t	_average_cycle_1      		= 0u;    
	static cx_uint32_t	_average_offset_ac_sense	= 0u;    
		
	cx_uint8_t 			i;
	cx_uint16_t 		average_value_ac_sense 		= 0u;
	cx_uint32_t 		sum_average_value_ac_sense 	= 0u;
	cx_uint32_t 		converted_value_ac_voltage 	= 0u;
	
	//---------------------------------------------------------------------------
	//get adc value
	converted_value_ac_voltage = _ADC_ValueTab[2];
	
	//---------------------------------------------------------------------------
	//sampling중 offset 아래 값을 플러스로 돌린 후, 주기 내 adc value 평균값을 통해 도출
	if(_count_sampling_ac_voltage >= SAMPLING_COUNT_AC_VOLTAGE) 
	{
		_count_sampling_ac_voltage 	= 0; 
		
		_offset_value_ac_sense += _array_sampling_ac_sense[_count_sampling_ac_voltage];
		
		_offset_value_ac_sense = _offset_value_ac_sense/SAMPLING_COUNT_AC_VOLTAGE;
		
		for(i= 0; i<SAMPLING_COUNT_AC_VOLTAGE; i++)
		{
			if(_offset_value_ac_sense > _array_sampling_ac_sense[i]) 
			{
				average_value_ac_sense = ((_offset_value_ac_sense - _array_sampling_ac_sense[i]) + _offset_value_ac_sense);
			}
			else average_value_ac_sense = _array_sampling_ac_sense[i];
			
			sum_average_value_ac_sense += average_value_ac_sense;
		}
		sum_average_value_ac_sense = sum_average_value_ac_sense/SAMPLING_COUNT_AC_VOLTAGE;
		
		//---------------------------------------------------------------------------
		if(_count_ac_sense_avreage >= MEASURED_AC_VOLTAGE) 
		{
			_count_ac_sense_avreage = 0;
			_average_cycle_1			= _average_cycle_1/MEASURED_AC_VOLTAGE;
		
			_average_offset_ac_sense = _average_offset_ac_sense/MEASURED_AC_VOLTAGE;
			
			//AC모듈 시험 가변저항: 22.1K
			_measured_AC_voltage = (cx_uint16_t)((_average_cycle_1 - _average_offset_ac_sense) * 3.75);	//AC 값 도출

			
			_average_cycle_1  = 0;
			_average_offset_ac_sense = 0;
			
			if(_measured_AC_voltage < 100) _measured_AC_voltage = 0u;
		}
		else 
		{				
			_count_ac_sense_avreage++;
			_average_cycle_1 += sum_average_value_ac_sense;	

			_average_offset_ac_sense += _offset_value_ac_sense;
		}
		//---------------------------------------------------------------------------
	}
	else
	{
		_array_sampling_ac_sense[_count_sampling_ac_voltage] = converted_value_ac_voltage;		
		_count_sampling_ac_voltage++;
		
		_offset_value_ac_sense += _array_sampling_ac_sense[_count_sampling_ac_voltage];				
	}
}


//---------------------------------------------------------------------------
void get_tx_current_irq_handler(void)
{   
	                           
	//-----------------------------------------------------------------------
	if (CX_FALSE==_flag_get_tx_current)
	{
		return;
	}

	//-----------------------------------------------------------------------
	static cx_uint8_t	_count_sampling_tx_current  = 0u;	
	static cx_uint8_t	_avreage_current_sampling  	= 0u;    
	static cx_uint32_t	_average_max_sampling   	= 0u;
	static cx_uint32_t	_average_max_sampling_cur  	= 0u;    
	
	cx_uint8_t 			j;
	cx_uint32_t 		max							= 0u;	
	cx_uint32_t 		converted_value_tx_currnet 	= 0u;
	cx_uint16_t 		average_sampling_tx_current = 10u;

	//---------------------------------------------------------------------------
	//get adc value
	converted_value_tx_currnet = _ADC_ValueTab[4];

	//---------------------------------------------------------------------------
	//sampling의 max 값 찾기
	if(_count_sampling_tx_current >= SAMPLING_COUNT_TX_CURRENT) 
	{
		_count_sampling_tx_current = 0u; 
				
		for(j= 0; j<SAMPLING_COUNT_TX_CURRENT; j++)
		{
			if(max <= _array_sampling_tx_current[j]) 
			{
				max = _array_sampling_tx_current[j];
			}	
		}
		//-----------------------------------------------------------------------
		if(_avreage_current_sampling >= average_sampling_tx_current) 
		{
			_avreage_current_sampling	= 0u;
			_average_max_sampling_cur  = _average_max_sampling/average_sampling_tx_current;
			
			_voltage_Level_tx_current 	= (cx_float_t)((_average_max_sampling_cur*2.6)/(4096-1));	//max sampling value
			
			_average_max_sampling  		= 0u;
			
			//-----------------------------------------------------------------------
			if(_voltage_Level_tx_current <= 1) _measured_tx_current = (cx_uint_t)((_voltage_Level_tx_current * 1.1)*1000);
			else _measured_tx_current = (cx_uint_t)((_voltage_Level_tx_current+0.1)*1000);	
					
		}
		else 
		{
			_avreage_current_sampling++;
			_average_max_sampling += max;	
		}
		//-----------------------------------------------------------------------
		_flag_get_tx_current 	= CX_FALSE;	//clear flag
	}	
	else 
	{
		_array_sampling_tx_current[_count_sampling_tx_current] = converted_value_tx_currnet;		
		_count_sampling_tx_current++;
	}
	
	//-----------------------------------------------------------------------
}

//---------------------------------------------------------------------------
cx_bool_t ac_voltage_control_watch(void)
{	
	//-----------------------------------------------------------------------
	cx_bool_t health_ac_voltage = CX_TRUE;
	cx_bool_t fail_max_count = 30u;
    
	static cx_uint_t _ac_voltage_fail_count = 0u;
	//-----------------------------------------------------------------------
	//필요 시 추가 
	
	
	//-----------------------------------------------------------------------
	if ( _ac_voltage_fail_count >= fail_max_count )
	{
		health_ac_voltage = CX_FALSE;
		
		_ac_voltage_fail_count = fail_max_count+1;
	}
	else
	{
		health_ac_voltage = CX_TRUE;
	}	
	//-----------------------------------------------------------------------
	return health_ac_voltage;
}

//---------------------------------------------------------------------------
cx_bool_t output_voltage_control_watch(void)
{	
	//-----------------------------------------------------------------------
	cx_bool_t output_voltage_health = CX_TRUE;
	cx_uint_t output_voltage_fail_max_count = 10u;//3초	//6초 20u
	static cx_uint_t _output_voltage_fail_count = 0u;
	
	//-----------------------------------------------------------------------
	if ( ((5800*0.95)>_measured_output_voltage) || ((5800*1.05)<_measured_output_voltage) )
	{
		_output_voltage_fail_count++;
	}
    else _output_voltage_fail_count = 0u;
	//-----------------------------------------------------------------------
	if ( _output_voltage_fail_count >= output_voltage_fail_max_count )	
	{
		output_voltage_health = CX_FALSE;
		
		_output_voltage_fail_count = output_voltage_fail_max_count+1;
	}
	else 
	{
		output_voltage_health = CX_TRUE;
    }    		
	//-----------------------------------------------------------------------
	return output_voltage_health;
}

//---------------------------------------------------------------------------
#if 1
void get_input_data_output_voltage(cx_bool_t clear_count)
{
	cx_bool_t i;
	cx_uint32_t average_output_voltage_value = 0u;

	cx_float_t measure_output_voltage = 0;
	cx_uint_t correction_output_voltage = 0u;

	static cx_uint_t _count_output_voltage = 0u;
	cx_uint32_t raw_voltage = 0;
	//-----------------------------------------------------------------------
	if (CX_TRUE == clear_count)
	{
		_count_output_voltage = 0u;
	}

	if (_count_output_voltage >= SAMPLING_COUNT_OUTPUT_VOLTAGE)
	{
		for (i = 0; i < SAMPLING_COUNT_OUTPUT_VOLTAGE; i++)
		{
			average_output_voltage_value += _watch_output_voltage[i];
		}

		average_output_voltage_value = (cx_uint_t)(average_output_voltage_value / SAMPLING_COUNT_OUTPUT_VOLTAGE);

		_voltage_Level_DC580 = average_output_voltage_value * 3.3 / (4096 - 1); // 전압 레벨
		test = average_output_voltage_value;
		
		if(average_output_voltage_value<=2470)
		{
			correction_output_voltage = (average_output_voltage_value - 2000)/70;
		}	
		
		measure_output_voltage = (cx_float_t)(2985 - average_output_voltage_value)/722;
		measure_output_voltage += 3.76;	
		
		_measured_output_voltage = (cx_uint_t)(average_output_voltage_value/measure_output_voltage) + correction_output_voltage;	//221019 ����
		_measured_output_voltage = _measured_output_voltage*10;

//		if(_measured_output_voltage <= 5570) _measured_output_voltage = 5570; // 공인기관 시험용, 시험 종료 후 삭제
//		if(_measured_output_voltage >= 6010) _measured_output_voltage = 6010; // 공인기관 시험용, 시험 종료 후 삭제
		_count_output_voltage = 0u;

		if (_measured_output_voltage < 100)
			_measured_output_voltage = 0u;
	
	}
	else
	{
		_watch_output_voltage[_count_output_voltage] = _ADC_ValueTab[3];
		_count_output_voltage++;
	}
}

#endif
#if 0
void get_input_data_output_voltage(cx_bool_t clear_count)
{
	cx_bool_t i;
	cx_uint32_t average_output_voltage_value = 0u;

	cx_float_t measure_output_voltage = 0;
	cx_uint_t correction_output_voltage = 0u;

	static cx_uint_t _count_output_voltage = 0u;
	//-----------------------------------------------------------------------
	if (CX_TRUE == clear_count)
	{
		_count_output_voltage = 0u;
	}

	if (_count_output_voltage >= OLD_SAMPLING_COUNT_OUTPUT_VOLTAGE)
	{
		for (i = 0; i < OLD_SAMPLING_COUNT_OUTPUT_VOLTAGE; i++)
		{
			average_output_voltage_value += _watch_output_voltage[i];
		}

		average_output_voltage_value = (cx_uint_t)(average_output_voltage_value / OLD_SAMPLING_COUNT_OUTPUT_VOLTAGE);

		_voltage_Level_DC580 = average_output_voltage_value * 3.3 / (4096 - 1); // 전압 레벨

		if (average_output_voltage_value >= 2200)
		{
			correction_output_voltage = (average_output_voltage_value - 2200) / 5;
		}

		measure_output_voltage = (cx_float_t)(2500 - average_output_voltage_value) / 450;
		measure_output_voltage += 3.1;

		_measured_output_voltage = (cx_uint_t)(average_output_voltage_value / measure_output_voltage) + correction_output_voltage; // 221019 수정
		_measured_output_voltage = _measured_output_voltage * 10;

		_count_output_voltage = 0u;

		if (_measured_output_voltage < 100)
			_measured_output_voltage = 0u;
	}
	else
	{
		_watch_output_voltage[_count_output_voltage] = _ADC_ValueTab[3];
		_count_output_voltage++;
	}
}
#endif
//---------------------------------------------------------------------------
void get_input_data_impulse_voltage(cx_bool_t active_measure, cx_uint_t count_value)
{
	cx_uint_t	i,j;	
	cx_uint_t	temp_imp_plus_value		= 0u;
	cx_uint_t	temp_imp_minus_value	= 0u;
	cx_uint32_t imp_PLUS_value			= 0u;
	cx_uint32_t imp_MINUS_value			= 0u;
	

	cx_uint_t		correction_minus 	  = 0u;
	//cx_uint_t		correction_plus_upper = 0u;
	cx_uint_t		correction_plus_lower = 0u;
	


	
    cx_float_t		measure_minus_1 	= 0;
	cx_float_t		measure_minus_2 	= 0;
	
	static cx_uint_t 	_count_plus_average 	= 0u;
	static cx_uint_t 	_count_minus_average 	= 0u;
	static cx_uint32_t _average_imp_plus 	    = 0u;
	static cx_uint32_t _average_imp_minus 	    = 0u;
	

    
    //---------------------------------------------------------------------------
	if(active_measure == CX_TRUE)
	{
		//정펄스 
		for(i=10; i<COUNT_IMPULSE_Value-5; i++) // 1ms~9.5ms
        {
            imp_PLUS_value += _array_PA_0[i];
        }
		temp_imp_plus_value = imp_PLUS_value/(COUNT_IMPULSE_Value-15);
		
		
		if(_count_plus_average>=SAMPLING_COUNT_IMPULSE_VOLTAGE)
		{
			_average_imp_plus /= SAMPLING_COUNT_IMPULSE_VOLTAGE;
			
			average_IMP_PLUS_value 		= _average_imp_plus;	
			_voltage_Level_impulse_plus = (average_IMP_PLUS_value *3.3)/(4096-1);

			if(average_IMP_PLUS_value>=2000)
			{
				//2000 이상
				correction_plus_upper = (cx_uint_t)(((average_IMP_PLUS_value - 2000)/2.3));
			}
			else;
			
			if(average_IMP_PLUS_value>=1400)
			{
				measure_plus_1 = (cx_float_t)average_IMP_PLUS_value/1400;
				measure_plus_2 = measure_plus_1 + 2.77;	
			}		
			else
			{
				//ADC 1400 이하
				correction_plus_lower = ((1400 - average_IMP_PLUS_value)/8)*10;
				
				measure_plus_1 = (cx_float_t)average_IMP_PLUS_value/800;
				measure_plus_2 = measure_plus_1 + 2;
			}	
		
			_measured_impulse_voltage_plus = ((average_IMP_PLUS_value/ measure_plus_2)*10) - correction_plus_lower + correction_plus_upper;          
			//---------------------------------------------------------------------------
            _count_plus_average = 0u;
            _average_imp_plus  = 0u;
		}
		else 
        {
          _average_imp_plus += temp_imp_plus_value;
          
          _count_plus_average++;
        }
		
		//---------------------------------------------------------------------------
		//부펄스
		for(j=60; j<COUNT_IMPULSE_Value-10; j++)
        {
            imp_MINUS_value += _array_PA_1[j];
        }
		temp_imp_minus_value = imp_MINUS_value/(COUNT_IMPULSE_Value-70);	//125 ~ 190 	//5ms~9.5ms
		
		
		if(_count_minus_average>=SAMPLING_COUNT_IMPULSE_VOLTAGE)
		{
			_average_imp_minus /= SAMPLING_COUNT_IMPULSE_VOLTAGE;
				
			average_IMP_MINUS_value 	 = _average_imp_minus;	
			_voltage_Level_impulse_minus = (average_IMP_MINUS_value *3.3)/(4096-1);
			/*
			if(average_IMP_MINUS_value>=1000)
			{
				measure_minus_1 = (cx_float_t)average_IMP_MINUS_value/1000;
				measure_minus_2 = measure_minus_1 + 7.9;			
			}	
			else
			{
				if(average_IMP_MINUS_value<300)
				{
					correction_minus = (300- average_IMP_MINUS_value)/2;
				}
				//if(average_IMP_MINUS_value<1000)
				//{
				//	correction_minus = (1000- average_IMP_MINUS_value)/2;
				//}
				
				measure_minus_1 = (cx_float_t)average_IMP_MINUS_value/200;
				measure_minus_2 = measure_minus_1 + 5.4;
				
			}
			
			_measured_impulse_voltage_minus = ((average_IMP_MINUS_value/ measure_minus_2)*10) + correction_minus;	
			if(average_IMP_MINUS_value>=1000)_measured_impulse_voltage_minus = 	average_IMP_MINUS_value*1.06;
			else if(average_IMP_MINUS_value<1000 && average_IMP_MINUS_value>=900)_measured_impulse_voltage_minus = 	average_IMP_MINUS_value*1.4;
			else if(average_IMP_MINUS_value<900 && average_IMP_MINUS_value>=800)_measured_impulse_voltage_minus = 	average_IMP_MINUS_value*1.5;
			else _measured_impulse_voltage_minus = 	average_IMP_MINUS_value*2.2;

			// 롤백용
if (average_IMP_MINUS_value >= 1300)
			{
				// 1300 이상: 고정 1.10
				scale = 1.10f;
			}
			else if (average_IMP_MINUS_value >= 700)
			{
				// 700 → 1.55, 1300 → 1.10
				scale = 1.55f + (1.10f - 1.55f) * (average_IMP_MINUS_value - 700) / 600.0f;
			}
			else if (average_IMP_MINUS_value >= 550)
			{
				// 550 → 1.75, 700 → 1.55
				scale = 1.75f + (1.55f - 1.75f) * (average_IMP_MINUS_value - 550) / 150.0f;
			}
			else if (average_IMP_MINUS_value >= 400)
			{
				// 400 → 2.00, 550 → 1.75
				scale = 2.00f + (1.75f - 2.00f) * (average_IMP_MINUS_value - 400) / 150.0f;
			}
			else if (average_IMP_MINUS_value >= 100)
			{
				// 100 → 2.80, 400 → 2.00
				scale = 2.80f + (2.00f - 2.80f) * (average_IMP_MINUS_value - 100) / 300.0f;
			}
			else
			{
				// 0 → 4.50, 100 → 2.80
				scale = 4.50f + (2.80f - 4.50f) * average_IMP_MINUS_value / 100.0f;
			}
			_measured_impulse_voltage_minus = average_IMP_MINUS_value * scale;
			*/
			float scale;
			if (average_IMP_MINUS_value >= 1300)
			{
				scale = 1.05f;  // 고정값, 상위 정상 출력 유지
			}
			else if (average_IMP_MINUS_value >= 700)
			{
				// 700 → 1.55, 1300 → 1.05
				scale = 1.55f + (1.05f - 1.55f) * (average_IMP_MINUS_value - 700) / 600.0f;
			}
			else if (average_IMP_MINUS_value >= 550)
			{
				// 550 → 1.75, 700 → 1.55
				scale = 1.72f + (1.52f - 1.72f) * (average_IMP_MINUS_value - 550) / 150.0f;
				//scale = 1.75f + (1.55f - 1.75f) * (average_IMP_MINUS_value - 550) / 150.0f;
			}
			else if (average_IMP_MINUS_value >= 400)
			{
				// 400 → 2.00, 550 → 1.75
				scale = 2.00f + (1.75f - 2.00f) * (average_IMP_MINUS_value - 400) / 150.0f;
			}
			else if (average_IMP_MINUS_value >= 100)
			{
				// 100 → 2.80, 400 → 2.00
				scale = 2.80f + (2.00f - 2.80f) * (average_IMP_MINUS_value - 100) / 300.0f;
			}
			else
			{
				// 0 → 4.5, 100 → 2.80
				scale = 4.50f + (2.80f - 4.50f) * (average_IMP_MINUS_value) / 100.0f;
			}
			

			_measured_impulse_voltage_minus = (cx_uint_t)(average_IMP_MINUS_value * scale);
			
			//if(_measured_impulse_voltage_minus < 200) _measured_impulse_voltage_minus = 220;
		//---------------------------------------------------------------------------
            _count_minus_average = 0u;
            _average_imp_minus  = 0u;
		}
		else 
        {
            _average_imp_minus += temp_imp_minus_value;
            
            _count_minus_average++;
        }
		
		//---------------------------------------------------------------------------
	}
	else 
	{
		_array_PA_0[count_value-1] = _ADC_ValueTab[0];	//정펄스
        _array_PA_1[count_value-1] = _ADC_ValueTab[1];	//부펄스	
	}	
	//---------------------------------------------------------------------------

}


void reset_sampling_data_pulse_frequency (pulse_frequency_t* e, cx_bool_t timer)
{
	if(PIN_FREQ == timer)
	{
		e->sampling_pulse_frequency_FREQ.period_count 	= 0u;
		e->sampling_pulse_frequency_FREQ.period_end 	= CX_FALSE;	
		
		memset(e->sampling_pulse_frequency_FREQ.sampling, 0, sizeof(e->sampling_pulse_frequency_FREQ.sampling));
		
		_3hz_frequency_FREQ	= 0u;	
	}	
	else if(PIN_CLOCK == timer)
	{
		e->sampling_pulse_frequency_CLOCK.period_count 		= 0u;
		e->sampling_pulse_frequency_CLOCK.period_end 			= CX_FALSE;	
		
		memset(e->sampling_pulse_frequency_CLOCK.sampling, 0, sizeof(e->sampling_pulse_frequency_CLOCK.sampling));
	
		_3hz_frequency_CLOCK = 0u;	
	}
}


//---------------------------------------------------------------------------
void control_watch_pulse_frequency(pulse_frequency_t* data, cx_float_t pulse_frequency, cx_bool_t timer)
{
	cx_bool_t fail;
	cx_bool_t freq_fail_max_count = 15u;
	
	fail = CX_FALSE;
	//-----------------------------------------------------------------------
	if ( ((3*0.97)>=pulse_frequency) || ((3*1.03)<=pulse_frequency) )
	{
		fail = CX_TRUE;
	}
	
#if 1	
	if(PIN_FREQ == timer)
	{
		if (CX_TRUE==fail)
		{
			if (data->sampling_pulse_frequency_FREQ.fail_count<=freq_fail_max_count)  
			{
				data->sampling_pulse_frequency_FREQ.fail_count++;
			}
			else
			{
				data->sampling_pulse_frequency_FREQ.fail_value = CX_TRUE;
				data->sampling_pulse_frequency_FREQ.health      = CX_FALSE;	
			}
		}
		else
		{
			data->sampling_pulse_frequency_FREQ.fail_count = 0u;
			data->sampling_pulse_frequency_FREQ.fail_value = CX_FALSE;
			
			data->sampling_pulse_frequency_FREQ.health = CX_TRUE;
		}
	}
	else if(PIN_CLOCK == timer)
	{
		if (CX_TRUE==fail)
		{
			if (data->sampling_pulse_frequency_CLOCK.fail_count<=freq_fail_max_count)  
			{
				data->sampling_pulse_frequency_CLOCK.fail_count++;
			}
			else
			{
				data->sampling_pulse_frequency_CLOCK.fail_value = CX_TRUE;
				data->sampling_pulse_frequency_CLOCK.health      = CX_FALSE;		//clock 핀은 주계/부계 항상 체크
			}
		}
		else
		{
			data->sampling_pulse_frequency_CLOCK.fail_count = 0u;
			data->sampling_pulse_frequency_CLOCK.fail_value = CX_FALSE;
			
			data->sampling_pulse_frequency_CLOCK.health = CX_TRUE;
		}	
	}
#endif	
}

//---------------------------------------------------------------------------
void analog_data_control_watch(void)
{	
	//-----------------------------------------------------------------------
	cx_bool_t fail_freq;
	cx_bool_t fail_clock;

	
	fail_freq 	= CX_FALSE;
	fail_clock	= CX_FALSE;
	//-----------------------------------------------------------------------
	pulse_frequency_t*		module;
	
	module = &_analog_data.pulse_frequency;
	
	control_watch_pulse_frequency(module, _3hz_frequency_CLOCK, PIN_CLOCK);	
	control_watch_pulse_frequency(module, _3hz_frequency_FREQ, PIN_FREQ);	
	
	if (CX_TRUE==module->sampling_pulse_frequency_FREQ.fail_value)		
	{
		fail_freq	= CX_TRUE;
	}
	
	if (CX_TRUE==module->sampling_pulse_frequency_CLOCK.fail_value)		
	{
		fail_clock	= CX_TRUE;
	}
	
	//-----------------------------------------------------------------------
	_analog_data.pulse_frequency.health = CX_TRUE;

	if (CX_TRUE==fail_freq)
	{
//		_analog_data.pulse_frequency.health = CX_FALSE;
	
//		reset_sampling_data_pulse_frequency(module, PIN_FREQ);  //reset은 필요없을듯
	}

	if (CX_TRUE==fail_clock)
	{
		_analog_data.pulse_frequency.health = CX_FALSE;	
		
		//fail나는 상황에서 고장 주파수값을 debug확인을 위해 reset은 동작X
//		reset_sampling_data_pulse_frequency(module, PIN_CLOCK);	
	}
}

//===================================================================================================
void get_pulse_frequency (sampling_data_t* sampling_data, cx_uint8_t pulse_count, cx_bool_t timer)
{	
	cx_uint8_t	count;
	cx_float_t	temp_frequency_value = 0u;
	
	if(CX_TRUE == sampling_data->period_end)
	{
		for(count=0;count<SAMPLING_COUNT_PULSE_FREQ;count++)
		{
			temp_frequency_value = (temp_frequency_value+ sampling_data->sampling[count]);
		}
		
		temp_frequency_value = temp_frequency_value/SAMPLING_COUNT_PULSE_FREQ;
	}	
	else
	{
		for(count=0;count<(pulse_count+1);count++)
		{
			temp_frequency_value = (temp_frequency_value+ sampling_data->sampling[count]);
		}

		temp_frequency_value = temp_frequency_value/(pulse_count+1);		
	}	

	if(timer == PIN_CLOCK)
	{
		_3hz_frequency_CLOCK	= temp_frequency_value;	//clock pin
	}	
	else if(timer == PIN_FREQ)
	{
		_3hz_frequency_FREQ		= temp_frequency_value;	//freq pin
	}
	else;	
}

//===================================================================================================
void put_pulse_frequency (cx_float_t freq, cx_bool_t timer)
{
	sampling_data_t*   sampling_data;
			
	if(timer == PIN_CLOCK)
	{
		sampling_data = &_analog_data.pulse_frequency.sampling_pulse_frequency_CLOCK;
	
		//-----------------------------------------------------------------------			
		if((sampling_data->period_count) < SAMPLING_COUNT_PULSE_FREQ)
		{		
			sampling_data->sampling[(sampling_data->period_count)] = freq; 
			
			get_pulse_frequency(sampling_data, (sampling_data->period_count), timer);
			
			(sampling_data->period_count)++;
		}
		else if((sampling_data->period_count) >= SAMPLING_COUNT_PULSE_FREQ)
		{ 
			sampling_data->period_count = 0u;
			sampling_data->period_end   = CX_TRUE;
			
			get_pulse_frequency(sampling_data, (sampling_data->period_count),timer);
			
		}	
	}
	else if(timer == PIN_FREQ)
	{
		sampling_data = &_analog_data.pulse_frequency.sampling_pulse_frequency_FREQ;
	
		//-----------------------------------------------------------------------			
		if((sampling_data->period_count) < SAMPLING_COUNT_PULSE_FREQ)
		{		
			sampling_data->sampling[(sampling_data->period_count)] = freq; 
			
			get_pulse_frequency(sampling_data, (sampling_data->period_count),timer);
			
			(sampling_data->period_count)++;		
		}
		else if((sampling_data->period_count) >= SAMPLING_COUNT_PULSE_FREQ)
		{ 
			sampling_data->period_count = 0u;
			sampling_data->period_end   = CX_TRUE;
			
			get_pulse_frequency(sampling_data, (sampling_data->period_count),timer);
			
		}	
	}
	else;
}

//===========================================================================
static void input_frequency_capture_pin (pulse_frequency_t* capture_pin)
{
	cx_uint_t cp;
	cx_uint_t p;
	
	sampling_data_t	input_pin_CLOCK;
	sampling_data_t	input_pin_FREQ;
	
	input_pin_CLOCK = capture_pin->sampling_pulse_frequency_CLOCK;
	
	debug_printf("\tInput_Clock	= Health: %d, Fail: %d, Fail_count: %d, P_end: %d, P_count: %d \n",
		input_pin_CLOCK.health,
		input_pin_CLOCK.fail_value,
		input_pin_CLOCK.fail_count,
		input_pin_CLOCK.period_end,
		input_pin_CLOCK.period_count    );
	
	cp = SAMPLING_COUNT_PULSE_FREQ;
	for (p=0u; p<cp; p++)
	{
		debug_printf("\t\t\t  Sampling %d - [%.2f]	\n",
			p,
			input_pin_CLOCK.sampling[p]	);
	}
	
	//-----------------------------------------------------------------------			
	input_pin_FREQ = capture_pin->sampling_pulse_frequency_FREQ;
	
	debug_printf("\tInput_Frequency	= Health: %d, Fail: %d, Fail_count: %d, P_end: %d, P_count: %d \n",
		input_pin_FREQ.health,
		input_pin_FREQ.fail_value,
		input_pin_FREQ.fail_count,
		input_pin_FREQ.period_end,
		input_pin_FREQ.period_count    );
	
	cp = SAMPLING_COUNT_PULSE_FREQ;
	for (p=0u; p<cp; p++)
	{
		debug_printf("\t\t\t  Sampling %d - [%.2f]	\n",
			p,
			input_pin_FREQ.sampling[p]	);
	}
	
	debug_flush();
}

//------------------------------------------------------------------------------			
void standby_clear_measured_data(void)
{
	pulse_frequency_t* frequency;
    
    frequency = &_analog_data.pulse_frequency;
    
   // _measured_output_voltage         = 0u;
	_measured_impulse_voltage_plus   = 0u;
	_measured_impulse_voltage_minus  = 0u;
	_measured_tx_current 			 = 0u;

	_3hz_frequency_FREQ  			 = 0;	
    reset_sampling_data_pulse_frequency(frequency, PIN_FREQ);
    
	//CLOCK, AC전압은 계속 측정
	//-----------------------------------------------------	
	//_voltage_Level_DC580  			 = 0;
	_voltage_Level_impulse_plus  	 = 0;
	_voltage_Level_impulse_minus = 0;
	_voltage_Level_tx_current		 = 0;
}

void clear_measured_data(void)
{
    pulse_frequency_t* frequency;
    
    frequency = &_analog_data.pulse_frequency;
    
    reset_sampling_data_pulse_frequency(frequency, PIN_CLOCK);
	//-----------------------------------------------------	
}		

//------------------------------------------------------------------------------			
void measured_analog_data_show(void)
{
	debug_printf("# MEASURED ANALOG DATA \n");

	debug_printf("\tOutput voltage          = %d \n" ,  _measured_output_voltage/10);
	debug_printf("\tImpulse voltage PLUS    = %d \n" ,  _measured_impulse_voltage_plus/10);
	debug_printf("\tImpulse voltage MINUS   = %d \n" ,	_measured_impulse_voltage_minus/10);
	debug_printf("\tAC voltage              = %d \n" ,	_measured_AC_voltage/10);
	debug_printf("\tTx current(mA)          = %d \n" ,	_measured_tx_current);
	
	debug_printf("# MEASURED ANALOG DATA voltage level \n");
	
	debug_printf("\tOutput voltage          = %.2f \n" , _voltage_Level_DC580);
	debug_printf("\tImpulse voltage PLUS    = %.2f \n" , _voltage_Level_impulse_plus);
	debug_printf("\tImpulse voltage MINUS   = %.2f \n" , _voltage_Level_impulse_minus);
	debug_printf("\tTx current              = %.2f \n" , _voltage_Level_tx_current);

}

//------------------------------------------------------------------------------			

void input_frequency_show(void)
{
	pulse_frequency_t*		module;
	
	module = &_analog_data.pulse_frequency;
	
	debug_printf("# Input Frequency\n");

	debug_printf("\thealth          = %d \n", module->health   );
	
	input_frequency_capture_pin(module);
}

//==============================================================================
