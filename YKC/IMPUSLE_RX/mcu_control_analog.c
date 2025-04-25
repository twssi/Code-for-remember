/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mcu.h"
#include "stm32f10x_init.h"
#include "stm32f10x_dma.h"
/////////////////////////////////////////////////////////////////////////////
//===========================================================================
analog_data_t _analog_data;
DMA_InitTypeDef DMA_InitStructure;

//-----------------------------------------------------------------------	
#define CHATTERING_COUNT_PULSE_FREQ		5u 

#define SAMPLING_COUNT_IMPULSE_VOLTAGE	5u
#define SAMPLING_COUNT_RELAY_VOLTAGE	2u
#define SAMPLING_COUNT_RX_CURRENT		50u
#define SAMPLING_COUNT_AVERAGE_CURRENT	5u

#define COUNT_IMPULSE_Value				190u
#define COUNT_RELAY_Voltage_Value		660u

#define ADC_CHANNEL						5u
//-----------------------------------------------------------------------	
//������
static cx_uint_t	_measured_impulse_voltage_plus 	= 0u;
static cx_uint_t	_measured_impulse_voltage_minus = 0u;
static cx_uint_t	_measured_relay_voltage_V1 		= 0u;
static cx_uint_t	_measured_relay_voltage_V2 		= 0u;
static cx_uint_t	_measured_rx_current 			= 0u;
static cx_uint_t	_count_rx_current 			= 0u;

static cx_float_t	_3hz_frequency_FREQ				= 0;

cx_uint32_t _average_V1_value       = 0u;
cx_uint32_t _average_V2_value       = 0u;

	
cx_uint32_t _average_IMP_PLUS_value  = 0u;
cx_uint32_t _average_IMP_MINUS_value = 0u;
//-----------------------------------------------------------------------	
//���з��� ��
static cx_float_t 	_voltage_Level_V1  				= 0;
static cx_float_t 	_voltage_Level_V2  				= 0;
static cx_float_t 	_voltage_Level_impulse_plus  	= 0;
static cx_float_t 	_voltage_Level_impulse_minus  	= 0;
static cx_float_t 	_voltage_Level_rx_current		= 0;

//-----------------------------------------------------------------------	
static cx_uint16_t _ADC_ValueTab[ADC_CHANNEL] 		= {0,};

static cx_bool_t _flag_measure_current = CX_FALSE;
//-----------------------------------------------------------------------	

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
	DMA_InitStructure.DMA_MemoryBaseAddr = (cx_uint32_t)&_ADC_ValueTab;
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
cx_uint_t get_imnpulse_voltage_plus_value(void)
{
	return _measured_impulse_voltage_plus;
}

cx_uint_t get_imnpulse_voltage_minus_value(void)
{
	return _measured_impulse_voltage_minus;
}

cx_uint_t get_relay_voltage_v1_value(void)
{
	return _measured_relay_voltage_V1;
}

cx_uint_t get_relay_voltage_v2_value(void)
{
	return _measured_relay_voltage_V2;
}

cx_uint_t get_rx_current_value(void)
{	
	return _measured_rx_current;
}	

cx_float_t get_rx_3hz_frequency_value(void)
{	
	return _3hz_frequency_FREQ;
}	
	static cx_uint16_t 	_array_PA_0[COUNT_IMPULSE_Value]	= {0, };
    static cx_uint16_t 	_array_PA_1[COUNT_IMPULSE_Value]	= {0, };
//-----------------------------------------------------------------------	
//���޽�, ���޽�
void get_input_data_impulse_voltage(cx_bool_t active_measure, cx_uint_t count_value)
{
	cx_uint_t	i,j;
	cx_uint_t	temp_imp_plus_value		 = 0u;
	cx_uint_t	temp_imp_minus_value	 = 0u;
	cx_uint32_t imp_PLUS_value			 = 0u;
	cx_uint32_t imp_MINUS_value			 = 0u;
	

	
    cx_float_t	_measure_plus_1 		 = 0;
    cx_float_t	_measure_plus_2 		 = 0;
  
    cx_float_t	_measure_minus_1 		 = 0;
    cx_float_t	_measure_minus_2 		 = 0;

	cx_uint_t	correction_plus 	= 0u;
	cx_uint_t	correction_minus 	= 0u;
	
	static cx_uint_t	_count_plus_average 	 = 0u;
	static cx_uint_t	_count_minus_average	 = 0u;
	static cx_uint32_t 	_average_imp_plus 		 = 0u;
	static cx_uint32_t 	_average_imp_minus		 = 0u;
    
//	static cx_uint16_t 	_array_PA_0[COUNT_IMPULSE_Value]	= {0, };
//  static cx_uint16_t 	_array_PA_1[COUNT_IMPULSE_Value]	= {0, };
		
    //-----------------------------------------------------------------------	
	if(active_measure == CX_TRUE)
	{
		//????? 
		for(i=10; i<COUNT_IMPULSE_Value; i++)
        {
            imp_PLUS_value += _array_PA_0[i];
        }
		temp_imp_plus_value = imp_PLUS_value/(COUNT_IMPULSE_Value-10);
		
		
		if(_count_plus_average>=SAMPLING_COUNT_IMPULSE_VOLTAGE)
		{
			_average_imp_plus /= SAMPLING_COUNT_IMPULSE_VOLTAGE;
			
			_average_IMP_PLUS_value 	= _average_imp_plus;	
			_voltage_Level_impulse_plus = (_average_IMP_PLUS_value *3.3)/(4096-1);
            
			correction_plus = 0u;
			
			if(_average_IMP_PLUS_value>2100)
			{
				correction_plus = ((_average_IMP_PLUS_value-2100)/18)*10;
			}	 
			else if(_average_IMP_PLUS_value<=1200)
			{
				if(_average_IMP_PLUS_value<=600) correction_plus = (cx_uint_t)(((1000-_average_IMP_PLUS_value)/9)*10);
				else if(_average_IMP_PLUS_value<=900) correction_plus = (cx_uint_t)(((1000-_average_IMP_PLUS_value)/7)*10);
				else correction_plus = (cx_uint_t)(((1200-_average_IMP_PLUS_value)/7)*10);
				
			}
			else;

			_measure_plus_1 = (cx_float_t)_average_IMP_PLUS_value/1600;
			_measure_plus_2 = _measure_plus_1 + 2.7;
			
			_measured_impulse_voltage_plus  = (cx_uint_t)((_average_IMP_PLUS_value/ _measure_plus_2)*105/10) + correction_plus; 
			//-----------------------------------------------------------------------	
            _count_plus_average = 0u;
            _average_imp_plus  = 0u;
		}
		else 
		{
			_average_imp_plus += temp_imp_plus_value;
			
			_count_plus_average++;
		}
      
		
		//---------------------------------------------------------------------------
		//?????
		for(j=120; j<COUNT_IMPULSE_Value; j++)
        {
            imp_MINUS_value += _array_PA_1[j];
        }
		temp_imp_minus_value = imp_MINUS_value/(COUNT_IMPULSE_Value-120);	//190 - 120 
		
		
		if(_count_minus_average>=SAMPLING_COUNT_IMPULSE_VOLTAGE)
		{
			_average_imp_minus /= SAMPLING_COUNT_IMPULSE_VOLTAGE;
			
			
			_average_IMP_MINUS_value 	 = _average_imp_minus;	
			_voltage_Level_impulse_minus = (_average_IMP_MINUS_value *3.3)/(4096-1);
			
			if(_average_IMP_MINUS_value<1000)
			{
				correction_minus = ((1000-_average_IMP_MINUS_value)/32)*10;
			}
			else if(_average_IMP_MINUS_value>=2500)
			{
				correction_minus = ((_average_IMP_MINUS_value-2500)/28)*10;
			}
		
			_measure_minus_1 = (cx_float_t)_average_IMP_MINUS_value/450;
			_measure_minus_2 = _measure_minus_1 + 7.6;
				
			_measured_impulse_voltage_minus  = (cx_uint_t)((_average_IMP_MINUS_value/ _measure_minus_2)*10) + correction_minus; 	
			//-----------------------------------------------------------------------	
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
		_array_PA_0[count_value-1] = _ADC_ValueTab[0];	//?????
        _array_PA_1[count_value-1] = _ADC_ValueTab[1];	//?????
	}	
	//---------------------------------------------------------------------------
}

//---------------------------------------------------------------------------
//V1, V2 voltage
void get_input_data_relay_voltage(cx_bool_t active_relay_measure, cx_uint_t relay_count_value)
{
    cx_uint_t	i,j;
    cx_uint_t	temp_V1_value		    = 0u;
    cx_uint_t	temp_V2_value		    = 0u;
	cx_uint32_t relay_V1_value			= 0u;
    cx_uint32_t relay_V2_value			= 0u;
	

    
    static cx_uint_t	_count_V1_average 	= 0u;
    static cx_uint_t	_count_V2_average 	= 0u;
    static cx_uint32_t 	_average_relay_V1 	= 0u;
    static cx_uint32_t 	_average_relay_V2 	= 0u;
    
	static cx_uint16_t 	_array_PA_2[COUNT_RELAY_Voltage_Value]	= {0, };
    static cx_uint16_t 	_array_PA_3[COUNT_RELAY_Voltage_Value]	= {0, };
        
    //---------------------------------------------------------------------------
    if(active_relay_measure == CX_TRUE)
    {
        //V1
        for(i=60; i<COUNT_RELAY_Voltage_Value; i++)
        {
            relay_V1_value += _array_PA_2[i];
        }
        
        temp_V1_value = relay_V1_value/(COUNT_RELAY_Voltage_Value-60);
        
        if(_count_V1_average>=SAMPLING_COUNT_RELAY_VOLTAGE)
		{
			_average_relay_V1 /= SAMPLING_COUNT_RELAY_VOLTAGE;
			
			_average_V1_value = _average_relay_V1;	
			_voltage_Level_V1 = (_average_V1_value *3.3)/(4096-1);

             _measured_relay_voltage_V1 = (cx_uint_t)((_average_V1_value/11)*10);

			 
			if(_measured_relay_voltage_V1<130)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*110/100;
			else if(_measured_relay_voltage_V1>= 130 && _measured_relay_voltage_V1 < 160)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*106/100;
			else if(_measured_relay_voltage_V1>= 160 && _measured_relay_voltage_V1 < 200)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*104/100;
			else if(_measured_relay_voltage_V1>= 200 && _measured_relay_voltage_V1 < 240)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*103/100;
			else if(_measured_relay_voltage_V1>= 240 && _measured_relay_voltage_V1 < 280)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*102/100;
			else if(_measured_relay_voltage_V1>= 280 && _measured_relay_voltage_V1 < 320)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*101/100;
			else if(_measured_relay_voltage_V1>= 320 && _measured_relay_voltage_V1 < 380)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*100/100;
			else if(_measured_relay_voltage_V1>= 380 && _measured_relay_voltage_V1 < 430)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*98/100;
			else if(_measured_relay_voltage_V1>= 430 && _measured_relay_voltage_V1 < 480)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*96/100;
			else if(_measured_relay_voltage_V1>= 480 && _measured_relay_voltage_V1 < 530)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*95/100;
			else if(_measured_relay_voltage_V1>= 530 && _measured_relay_voltage_V1 < 580)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*94/100;
			else if(_measured_relay_voltage_V1>= 580 && _measured_relay_voltage_V1 < 630)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*93/100;
			else if(_measured_relay_voltage_V1>= 630 && _measured_relay_voltage_V1 < 680)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*92/100;
			else if(_measured_relay_voltage_V1>= 680)_measured_relay_voltage_V1 = _measured_relay_voltage_V1*91/100;
			

            //---------------------------------------------------------------------------
            _count_V1_average 	= 0u;	
            _average_relay_V1  	= 0u;
		}	
		else 
		{
			_average_relay_V1 += temp_V1_value;
			
			_count_V1_average++;
		}
        
        //---------------------------------------------------------------------------
        //V2
        for(j=0; j<COUNT_RELAY_Voltage_Value; j++)
        {
            relay_V2_value += _array_PA_3[j];
        }
        
        temp_V2_value = relay_V2_value/(COUNT_RELAY_Voltage_Value);
        
        if(_count_V2_average>=SAMPLING_COUNT_RELAY_VOLTAGE)
		{
			_average_relay_V2 /= SAMPLING_COUNT_RELAY_VOLTAGE;
			
			_average_V2_value = _average_relay_V2;	
			_voltage_Level_V2 = (_average_V2_value *3.3)/(4096-1);

			_measured_relay_voltage_V2 = (cx_uint_t)((_average_V2_value/34)*10);

			if(_measured_relay_voltage_V2<150)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*108/100;
			else if(_measured_relay_voltage_V2>= 150 && _measured_relay_voltage_V2 < 200)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*107/100;
			else if(_measured_relay_voltage_V2>= 200 && _measured_relay_voltage_V2 < 230)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*105/100;
			else if(_measured_relay_voltage_V2>= 230 && _measured_relay_voltage_V2 < 280)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*103/100;
			else if(_measured_relay_voltage_V2>= 280 && _measured_relay_voltage_V2 < 330)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*102/100;
			else if(_measured_relay_voltage_V2>= 330 && _measured_relay_voltage_V2 < 380)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*100/100;
			else if(_measured_relay_voltage_V2>= 380 && _measured_relay_voltage_V2 < 430)_measured_relay_voltage_V2 = _measured_relay_voltage_V2*98/100;
			else if(_measured_relay_voltage_V2>= 430 && _measured_relay_voltage_V2 < 480)_measured_relay_voltage_V2 =_measured_relay_voltage_V2*96/100;
			else if(_measured_relay_voltage_V2>= 480 && _measured_relay_voltage_V2 < 530)_measured_relay_voltage_V2 =_measured_relay_voltage_V2*95/100;
			else if(_measured_relay_voltage_V2>= 530 && _measured_relay_voltage_V2 < 580)_measured_relay_voltage_V2 =_measured_relay_voltage_V2*94/100;
			else if(_measured_relay_voltage_V2>= 580 && _measured_relay_voltage_V2 < 630)_measured_relay_voltage_V2 =_measured_relay_voltage_V2*92/100;
			else if(_measured_relay_voltage_V2>= 630 && _measured_relay_voltage_V2 < 680)_measured_relay_voltage_V2 =_measured_relay_voltage_V2*91/100;
			else if(_measured_relay_voltage_V2>= 680) _measured_relay_voltage_V2 = _measured_relay_voltage_V2*90/100;

			
 
            //---------------------------------------------------------------------------
            _count_V2_average 	= 0u;
            _average_relay_V2  	= 0u;
		}
        else 
		{
			_average_relay_V2 += temp_V2_value;
			
			_count_V2_average++;
		}
        
       //---------------------------------------------------------------------------
    }   
    else 
	{
		_array_PA_2[relay_count_value-1] = _ADC_ValueTab[2];	//V1 ����
        _array_PA_3[relay_count_value-1] = _ADC_ValueTab[3];	//V2 ����
	}	
}

//-----------------------------------------------------------------------	
void active_flag_frequency_rising_edge(void)
{   
    _flag_measure_current = CX_TRUE;
}

void analog_100us_tim5_irq(void) // ���� 50us ���� ���ͷ�Ʈ
{
	put_current_value();
}
static cx_uint16_t 	_array_current_value[SAMPLING_COUNT_RX_CURRENT] = {0,};
//-----------------------------------------------------------------------	
//rx current
void put_current_value(void) // dma �����Ҷ����� ȣ��
{
    //-----------------------------------------------------------------------
	if (CX_FALSE==_flag_measure_current)
	{
		return;
	}
    
	//-----------------------------------------------------------------------
    cx_uint_t 		j;	
	cx_uint_t 		temp_current_value = 0u;
	cx_uint16_t 	max	= 0u;	
	cx_float_t 		weight_measure_current 	= 0;
	cx_float_t 		measure_sqrt_value 		= 0;
	cx_uint_t 		correction_current 		= 0u;
	
	static cx_uint8_t	_count_average_current  	= 0u;    
	static cx_uint_t 	_count_measure_current 		= 0u; 
	static cx_uint32_t	_average_max_current_value  = 0u;
	
 
    //-----------------------------------------------------------------------
	if(_count_measure_current >= SAMPLING_COUNT_RX_CURRENT) 
    {
		_count_measure_current = 0;	
       
		_flag_measure_current = CX_FALSE; //clear flag 
       
		//----------------------------------------------------------------------- 
		for(j= 5; j<SAMPLING_COUNT_RX_CURRENT; j++)  
		{
			if(max <= _array_current_value[j]) 
			{
				max = _array_current_value[j];
			}	
		}

		//-----------------------------------------------------------------------
		if(_count_average_current >= SAMPLING_COUNT_AVERAGE_CURRENT) 
		{
			temp_current_value  = _average_max_current_value/SAMPLING_COUNT_AVERAGE_CURRENT;
			
			_voltage_Level_rx_current 	= (cx_float_t)((temp_current_value*3.3)/(4096-1));	//���� ���з���
			
			_count_average_current		= 0u;
			_average_max_current_value 	= 0u;

			if(temp_current_value>2500)	_measured_rx_current=temp_current_value*70/100;	
			else if(1500<temp_current_value && temp_current_value<=2500) _measured_rx_current=temp_current_value*74/100;
			else if(500<temp_current_value && temp_current_value<=1500) _measured_rx_current=temp_current_value*76/100;
			else _measured_rx_current=temp_current_value*79/100;	
		}
		else 
		{
			_average_max_current_value += max;	
			_count_average_current++;
		}

		_count_measure_current = 0;	
		//-----------------------------------------------------------------------
    }   
	else
	{
		_array_current_value[_count_measure_current]= _ADC_ValueTab[4];
		
		_count_measure_current++;	
	}	
    //-----------------------------------------------------------------------
}

//---------------------------------------------------------------------------
void reset_sampling_data_pulse_frequency (pulse_frequency_t* e)
{
	e->sampling_pulse_frequency.period_count 	= 0u;
	e->sampling_pulse_frequency.period_end 		= CX_FALSE;	
	
	memset(e->sampling_pulse_frequency.sampling, 0, sizeof(e->sampling_pulse_frequency.sampling));
		
	_3hz_frequency_FREQ = 0u;
}

//---------------------------------------------------------------------------
void control_watch_pulse_frequency(pulse_frequency_t* data, cx_float_t pulse_frequency)
{
	cx_bool_t fail;
	
	fail = CX_FALSE;	
	//-----------------------------------------------------------------------
	if ( ((3*0.97)>=pulse_frequency) || ((3*1.03)<=pulse_frequency) )
	{
		fail = CX_TRUE;
	}
	
    if (CX_TRUE==fail)
	{
        if (data->sampling_pulse_frequency.fail_count<=CHATTERING_COUNT_PULSE_FREQ) //5�� //펄스 fail시 기다리는 시간 기존 5초
		{
			data->sampling_pulse_frequency.fail_count++;
		}
        else
        {
            data->sampling_pulse_frequency.fail_value = CX_TRUE;
            data->sampling_pulse_frequency.health     = CX_FALSE;	
        }
    }  
    else
    {
        data->sampling_pulse_frequency.fail_count = 0u;
		data->sampling_pulse_frequency.fail_value = CX_FALSE;
			
		data->sampling_pulse_frequency.health = CX_TRUE;
    }

}

//---------------------------------------------------------------------------
void analog_data_control_watch(void)
{	
	//-----------------------------------------------------------------------
	cx_bool_t fail;

	fail = CX_FALSE;	
	//-----------------------------------------------------------------------
	pulse_frequency_t*		module;
	
	module = &_analog_data.pulse_frequency;
	
	control_watch_pulse_frequency(module, _3hz_frequency_FREQ);
	
	if (CX_TRUE==module->sampling_pulse_frequency.fail_value)		
	{
		fail = CX_TRUE;
	}
	
	//-----------------------------------------------------------------------
	_analog_data.pulse_frequency.health = CX_TRUE;
	
	if (CX_TRUE==fail)
	{
		_analog_data.pulse_frequency.health = CX_FALSE;
	
		reset_sampling_data_pulse_frequency(module);
	}	
}

void get_pulse_frequency (sampling_data_t* sampling_data, cx_uint8_t pulse_count)
{	
	cx_uint8_t	count;
	cx_float_t	temp_frequency_value = 0u;
	
	if(CX_TRUE == sampling_data->period_end)
	{
		for(count=0;count<CHATTERING_COUNT_PULSE_FREQ;count++)
		{
			temp_frequency_value = (temp_frequency_value+ sampling_data->sampling[count]);
		}
		
		temp_frequency_value = temp_frequency_value/CHATTERING_COUNT_PULSE_FREQ;
	}	
	else
	{
		for(count=0;count<(pulse_count+1);count++)
		{
			temp_frequency_value = (temp_frequency_value+ sampling_data->sampling[count]);
		}

		temp_frequency_value = temp_frequency_value/(pulse_count+1);		
	}	

	_3hz_frequency_FREQ 			= temp_frequency_value;
}

void put_pulse_frequency (cx_float_t freq)
{
	sampling_data_t*   sampling_data;
			
	sampling_data = &_analog_data.pulse_frequency.sampling_pulse_frequency;

	//-----------------------------------------------------------------------			
	if((sampling_data->period_count) < CHATTERING_COUNT_PULSE_FREQ)	
	{		
		sampling_data->sampling[(sampling_data->period_count)] = freq; 
		
		get_pulse_frequency(sampling_data, (sampling_data->period_count));
		
		(sampling_data->period_count)++;
	}	
	else if((sampling_data->period_count) >= CHATTERING_COUNT_PULSE_FREQ)
	{ 
		sampling_data->period_count = 0u;
		sampling_data->period_end   = CX_TRUE;
		
		get_pulse_frequency(sampling_data, (sampling_data->period_count));
		
	}	
}

//===========================================================================
static void input_frequency_capture_pin (pulse_frequency_t* capture_pin)
{
	cx_uint_t cp;
	cx_uint_t p;
	
	sampling_data_t	input_pin_FREQ;
	
	//-----------------------------------------------------------------------			
	input_pin_FREQ = capture_pin->sampling_pulse_frequency;
	
	debug_printf("\tInput_Frequency	= Fail: %d, Fail_count: %d, P_end: %d, P_count: %d \n",
		input_pin_FREQ.fail_value,
		input_pin_FREQ.fail_count,
		input_pin_FREQ.period_end,
		input_pin_FREQ.period_count    );
	
	cp = CHATTERING_COUNT_PULSE_FREQ;
	for (p=0u; p<cp; p++)
	{
		debug_printf("\t\t\t  Sampling %d - [%.2f]	\n",
			p,
			input_pin_FREQ.sampling[p]	);
	}
	
	debug_flush();
}


//------------------------------------------------------------------------------			
void clear_measured_data(void)
{
	pulse_frequency_t* frequency;
    
    frequency = &_analog_data.pulse_frequency;
	
	_measured_impulse_voltage_plus 	= 0u;
	_measured_impulse_voltage_minus = 0u;
	_measured_relay_voltage_V1 		= 0u;
	_measured_relay_voltage_V2 		= 0u;
	_measured_rx_current 			= 0u;
	
	_3hz_frequency_FREQ  			= 0;
	reset_sampling_data_pulse_frequency(frequency);
	//-----------------------------------------------------	
	_voltage_Level_V1  				= 0;	
	_voltage_Level_V2  				= 0;
	_voltage_Level_impulse_plus 	= 0;
	_voltage_Level_impulse_minus	= 0;
	_voltage_Level_rx_current		= 0;
}	


//------------------------------------------------------------------------------			
void measured_analog_data_show(void)
{
	debug_printf("# MEASURED ANALOG DATA \n");
	
	debug_printf("\tRelay voltage V1     	= %d \n" ,  _measured_relay_voltage_V1/10);
	debug_printf("\tRelay voltage V2     	= %d \n" ,  _measured_relay_voltage_V2/10);
	debug_printf("\tImpulse voltage PLUS    = %d \n" ,  _measured_impulse_voltage_plus/10);
	debug_printf("\tImpulse voltage MINUS   = %d \n" ,	_measured_impulse_voltage_minus/10);
	debug_printf("\tRx current(mA)     	= %d \n" ,	_measured_rx_current/10);
	
	debug_printf("# MEASURED ANALOG DATA voltage level \n");
	
	debug_printf("\tRelay voltage V1    	= %.2f \n" , _voltage_Level_V1);
	debug_printf("\tRelay voltage V2    	= %.2f \n" , _voltage_Level_V2);
	debug_printf("\tImpulse voltage PLUS    = %.2f \n" , _voltage_Level_impulse_plus);
	debug_printf("\tImpulse voltage MINUS   = %.2f \n" , _voltage_Level_impulse_minus);
	debug_printf("\tRx current     	        = %.2f \n" , _voltage_Level_rx_current);

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



//---------------------------------------------------------------------------
