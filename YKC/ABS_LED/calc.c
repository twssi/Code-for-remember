#include "calc.h"

/* Single definitions (storage) for measurement globals */
uint64_t measured_PWM_Cur = 0;
uint16_t measured_TEMP_Val = 0;
uint16_t measured_CDS_Val = 0;
uint16_t measured_DC_Val = 0;

#define CDS_SAMPLE_WINDOW       10u     // 10샘플 평균 (약 1초 @100ms 주기)
#define TEMP_SAMPLE_WINDOW       10u



volatile uint32_t adc_dma_values[NUM_ADC_CHANNELS];


uint16_t count_measure_temper		    = 0u;
uint32_t temper_measure_sum  		    = 0u;
uint32_t _averageTemp_max_sampling      = 0u;
float 	 _voltage_Level_temp			= 0;


uint16_t count_measure_CDS			    = 0u;
uint32_t CDS_measure_sum  			    = 0u;
uint32_t CDS_measure_avg  			    = 0u;
uint32_t _averageCds_max_sampling    	= 0u;



uint16_t count_measure_DC			    = 0u;
uint32_t DC_measure_sum  			    = 0u;

uint32_t converted_value_cds;
uint32_t converted_value_temper;
uint32_t converted_value_tx_currnet;
uint32_t converted_value_DC_volatage;

uint8_t current_ready = 0; 


//uint32_t test1 = 0;
//uint32_t test2 = 0;
//uint32_t test3 = 0;
uint32_t test4 = 0;
uint32_t test5 = 0;
uint32_t test6 = 0;
//uint32_t test7 = 0;
//uint32_t test8 = 0;



uint64_t get_PWM_cur_Value(void)
{
	return measured_PWM_Cur;
}

uint16_t get_TEMP_Value(void)
{
	return measured_TEMP_Val;
}

uint16_t get_CDS_Value(void)
{
	return measured_CDS_Val;
}

uint16_t get_DC_Value(void)
{
	return measured_DC_Val;
}




/* --- [Helper Function] 선형 보간 계산 --- */
inline float interpolate_mA(float avg_adc, float base_adc, float base_mA, float gain) 
{
    return base_mA + (avg_adc - base_adc) * gain;
}

/* --- [Calculation] 70% 듀티용 변환 테이블 --- */
float calculate_70_duty_mA(float avg_adc) 
{
    // --- [고전류 구간: 400mA ~ 580mA] ---
    if      (avg_adc >= 2876) return interpolate_mA(avg_adc, 2876, 580, 10.0f / 63.0f); 
    else if (avg_adc >= 2813) return interpolate_mA(avg_adc, 2813, 570, 10.0f / 58.0f); 
    else if (avg_adc >= 2755) return interpolate_mA(avg_adc, 2755, 560, 10.0f / 44.0f); 
    else if (avg_adc >= 2711) return interpolate_mA(avg_adc, 2711, 550, 10.0f / 53.0f); 
    else if (avg_adc >= 2657) return interpolate_mA(avg_adc, 2657, 540, 10.0f / 57.0f); 
    else if (avg_adc >= 2600) return interpolate_mA(avg_adc, 2600, 530, 10.0f / 59.0f); 
    else if (avg_adc >= 2541) return interpolate_mA(avg_adc, 2541, 520, 10.0f / 45.0f); 
    else if (avg_adc >= 2496) return interpolate_mA(avg_adc, 2496, 510, 10.0f / 52.0f); 
    else if (avg_adc >= 2444) return interpolate_mA(avg_adc, 2444, 500, 10.0f / 43.0f); 
    else if (avg_adc >= 2401) return interpolate_mA(avg_adc, 2401, 490, 10.0f / 67.0f); 
    else if (avg_adc >= 2334) return interpolate_mA(avg_adc, 2334, 480, 10.0f / 52.0f); 
    else if (avg_adc >= 2281) return interpolate_mA(avg_adc, 2281, 470, 10.0f / 48.0f); 
    else if (avg_adc >= 2233) return interpolate_mA(avg_adc, 2233, 460, 10.0f / 59.0f); 
    else if (avg_adc >= 2174) return interpolate_mA(avg_adc, 2174, 450, 10.0f / 42.0f); 
    else if (avg_adc >= 2132) return interpolate_mA(avg_adc, 2132, 440, 10.0f / 62.0f); 
    else if (avg_adc >= 2072) return interpolate_mA(avg_adc, 2072, 430, 10.0f / 52.0f); 
    else if (avg_adc >= 2020) return interpolate_mA(avg_adc, 2020, 420, 10.0f / 50.0f); 
    else if (avg_adc >= 1970) return interpolate_mA(avg_adc, 1970, 410, 10.0f / 59.0f); 
    else if (avg_adc >= 1911) return interpolate_mA(avg_adc, 1911, 400, 10.0f / 48.0f); 

    // --- [중전류 구간: 160mA ~ 390mA] ---
    else if (avg_adc >= 1863) return interpolate_mA(avg_adc, 1863, 390, 8.0f / 47.0f);  
    else if (avg_adc >= 1816) return interpolate_mA(avg_adc, 1816, 382, 12.0f / 64.0f); 
    else if (avg_adc >= 1752) return interpolate_mA(avg_adc, 1752, 370, 10.0f / 50.0f); 
    else if (avg_adc >= 1701) return interpolate_mA(avg_adc, 1701, 360, 10.0f / 46.0f); 
    else if (avg_adc >= 1655) return interpolate_mA(avg_adc, 1655, 350, 10.0f / 44.0f); 
    else if (avg_adc >= 1611) return interpolate_mA(avg_adc, 1611, 340, 10.0f / 55.0f); 
    else if (avg_adc >= 1556) return interpolate_mA(avg_adc, 1556, 330, 10.0f / 48.0f); 
    else if (avg_adc >= 1508) return interpolate_mA(avg_adc, 1508, 320, 10.0f / 45.0f); 
    else if (avg_adc >= 1463) return interpolate_mA(avg_adc, 1463, 310, 10.0f / 54.0f); 
    else if (avg_adc >= 1409) return interpolate_mA(avg_adc, 1409, 300, 10.0f / 55.0f); 
    else if (avg_adc >= 1354) return interpolate_mA(avg_adc, 1354, 290, 10.0f / 47.0f); 
    else if (avg_adc >= 1307) return interpolate_mA(avg_adc, 1307, 280, 10.0f / 48.0f); 
    else if (avg_adc >= 1259) return interpolate_mA(avg_adc, 1259, 270, 10.0f / 48.0f); 
    else if (avg_adc >= 1211) return interpolate_mA(avg_adc, 1211, 260, 10.0f / 53.0f); 
    else if (avg_adc >= 1158) return interpolate_mA(avg_adc, 1158, 250, 10.0f / 46.0f); 
    else if (avg_adc >= 1112) return interpolate_mA(avg_adc, 1112, 240, 10.0f / 51.0f); 
    else if (avg_adc >= 1061) return interpolate_mA(avg_adc, 1061, 230, 10.0f / 41.0f); 
    else if (avg_adc >= 1020) return interpolate_mA(avg_adc, 1020, 220, 10.0f / 50.0f); 
    else if (avg_adc >= 970)  return interpolate_mA(avg_adc, 970,  210, 10.0f / 50.0f); 
    else if (avg_adc >= 920)  return interpolate_mA(avg_adc, 920,  200, 10.0f / 50.0f); 
    else if (avg_adc >= 871)  return interpolate_mA(avg_adc, 871,  190, 10.0f / 49.0f); 
    else if (avg_adc >= 822)  return interpolate_mA(avg_adc, 822,  180, 10.0f / 47.0f); 
    else if (avg_adc >= 775)  return interpolate_mA(avg_adc, 775,  170, 10.0f / 51.0f); 

    // --- [저전류 집중 보정 구간: 160mA 이하, 베이스 값 +5mA 상향] ---
    else if (avg_adc >= 724)  return interpolate_mA(avg_adc, 724,  165, 10.0f / 45.0f); // 160 -> 165
    else if (avg_adc >= 679)  return interpolate_mA(avg_adc, 679,  155, 10.0f / 48.0f); // 150 -> 155
    else if (avg_adc >= 631)  return interpolate_mA(avg_adc, 631,  145, 10.0f / 43.0f); // 140 -> 145
    else if (avg_adc >= 588)  return interpolate_mA(avg_adc, 588,  135, 5.0f / 22.0f);  // 130 -> 135
    else if (avg_adc >= 566)  return interpolate_mA(avg_adc, 566,  130, 12.0f / 63.0f); // 125 -> 130
    else if (avg_adc >= 503)  return interpolate_mA(avg_adc, 503,  118, 12.0f / 59.0f); // 113 -> 118
    else if (avg_adc >= 444)  return interpolate_mA(avg_adc, 444,  106, 12.0f / 66.0f); // 101 -> 106
    else if (avg_adc >= 378)  return interpolate_mA(avg_adc, 378,  94,  12.0f / 60.0f);  // 89  -> 94
    else if (avg_adc >= 318)  return interpolate_mA(avg_adc, 318,  82,  12.0f / 64.0f);  // 77  -> 82
    else if (avg_adc >= 254)  return interpolate_mA(avg_adc, 254,  70,  12.0f / 68.0f);  // 65  -> 70
    else if (avg_adc >= 186)  return interpolate_mA(avg_adc, 186,  58,  13.0f / 68.0f);  // 53  -> 58
    else if (avg_adc >= 118)  return interpolate_mA(avg_adc, 118,  45,  13.0f / 65.0f);  // 40  -> 45
    else if (avg_adc >= 53)   return interpolate_mA(avg_adc, 53,   32,  13.0f / 32.0f);  // 27  -> 32
    else if (avg_adc >= 21)   return interpolate_mA(avg_adc, 21,   19,  13.0f / 32.0f);  // 14  -> 19
    else                      return (avg_adc * 0.9f); // 극저전류 보상 상향
}

/* --- [Calculation] 50% 듀티용 변환 테이블 --- */
float calculate_50_duty_mA(float avg_adc) 
{
    // --- [고전류 구간: 300mA ~ 410mA] ---
    if      (avg_adc >= 2761) return interpolate_mA(avg_adc, 2761, 410,  8.0f / 61.0f); // 410mA (ADC 2742~2780 중간)
    else if (avg_adc >= 2700) return interpolate_mA(avg_adc, 2700, 402,  8.0f / 72.0f); // 402mA (ADC 2682~2718 중간)
    else if (avg_adc >= 2628) return interpolate_mA(avg_adc, 2628, 391, 11.0f / 87.0f); 
    else if (avg_adc >= 2541) return interpolate_mA(avg_adc, 2541, 380,  8.0f / 49.0f); 
    else if (avg_adc >= 2492) return interpolate_mA(avg_adc, 2492, 372, 10.0f / 85.0f); 
    else if (avg_adc >= 2407) return interpolate_mA(avg_adc, 2407, 362, 12.0f / 85.0f); 
    else if (avg_adc >= 2322) return interpolate_mA(avg_adc, 2322, 350,  9.0f / 67.0f); 
    else if (avg_adc >= 2255) return interpolate_mA(avg_adc, 2255, 341,  9.0f / 64.0f); 
    else if (avg_adc >= 2191) return interpolate_mA(avg_adc, 2191, 332, 12.0f / 83.0f); 
    else if (avg_adc >= 2108) return interpolate_mA(avg_adc, 2108, 320, 10.0f / 69.0f); 
    else if (avg_adc >= 2039) return interpolate_mA(avg_adc, 2039, 310, 10.0f / 65.0f); 
    else if (avg_adc >= 1974) return interpolate_mA(avg_adc, 1974, 300,  9.0f / 66.0f); 

    // --- [중전류 구간: 150mA ~ 291mA] ---
    else if (avg_adc >= 1908) return interpolate_mA(avg_adc, 1908, 291, 11.0f / 71.0f); 
    else if (avg_adc >= 1837) return interpolate_mA(avg_adc, 1837, 280, 10.0f / 67.0f); 
    else if (avg_adc >= 1770) return interpolate_mA(avg_adc, 1770, 270, 10.0f / 74.0f); 
    else if (avg_adc >= 1696) return interpolate_mA(avg_adc, 1696, 260, 10.0f / 66.0f); 
    else if (avg_adc >= 1630) return interpolate_mA(avg_adc, 1630, 250, 10.0f / 69.0f); 
    else if (avg_adc >= 1561) return interpolate_mA(avg_adc, 1561, 240, 10.0f / 63.0f); 
    else if (avg_adc >= 1498) return interpolate_mA(avg_adc, 1498, 230, 10.0f / 62.0f); 
    else if (avg_adc >= 1436) return interpolate_mA(avg_adc, 1436, 220, 10.0f / 70.0f); 
    else if (avg_adc >= 1366) return interpolate_mA(avg_adc, 1366, 210, 10.0f / 66.0f); 
    else if (avg_adc >= 1300) return interpolate_mA(avg_adc, 1300, 200, 10.0f / 72.0f); 
    else if (avg_adc >= 1228) return interpolate_mA(avg_adc, 1228, 190, 10.0f / 66.0f); 
    else if (avg_adc >= 1162) return interpolate_mA(avg_adc, 1162, 180, 10.0f / 58.0f); 
    else if (avg_adc >= 1104) return interpolate_mA(avg_adc, 1104, 170, 10.0f / 71.0f); 
    else if (avg_adc >= 1033) return interpolate_mA(avg_adc, 1033, 160, 10.0f / 70.0f); 
    else if (avg_adc >= 963)  return interpolate_mA(avg_adc, 963,  150, 10.0f / 68.0f); 

    // --- [저전류 구간: 11mA ~ 140mA] ---
    else if (avg_adc >= 895)  return interpolate_mA(avg_adc, 895,  140, 10.0f / 61.0f); 
    else if (avg_adc >= 834)  return interpolate_mA(avg_adc, 834,  130, 10.0f / 67.0f); 
    else if (avg_adc >= 767)  return interpolate_mA(avg_adc, 767,  120, 10.0f / 60.0f); 
    else if (avg_adc >= 707)  return interpolate_mA(avg_adc, 707,  110, 10.0f / 66.0f); 
    else if (avg_adc >= 640)  return interpolate_mA(avg_adc, 640,  100, 10.0f / 68.0f); 
    else if (avg_adc >= 572)  return interpolate_mA(avg_adc, 572,  90,  10.0f / 98.0f); 
    else if (avg_adc >= 474)  return interpolate_mA(avg_adc, 474,  80,  10.0f / 63.0f); 
    else if (avg_adc >= 411)  return interpolate_mA(avg_adc, 411,  70,  10.0f / 81.0f); 
    else if (avg_adc >= 330)  return interpolate_mA(avg_adc, 330,  60,  10.0f / 65.0f); 
    else if (avg_adc >= 265)  return interpolate_mA(avg_adc, 265,  50,  10.0f / 76.0f); 
    else if (avg_adc >= 189)  return interpolate_mA(avg_adc, 189,  40,  10.0f / 79.0f); 
    else if (avg_adc >= 109)  return interpolate_mA(avg_adc, 109,  30,  10.0f / 46.0f); 
    else if (avg_adc >= 63)   return interpolate_mA(avg_adc, 63,   20,  9.0f / 6.0f);   // 20mA (ADC 61~66)
    else if (avg_adc >= 57)   return interpolate_mA(avg_adc, 57,   11,  9.0f / 6.0f);   // 11mA (ADC 57)
    
    else                      return (avg_adc * 11.0f / 57.0f); // 11mA 미만
}

/* --- [Calculation] 40% 듀티용 변환 테이블 --- */
float calculate_40_duty_mA(float avg_adc) 
{
    if      (avg_adc >= 2718) return interpolate_mA(avg_adc, 2718, 310, 0.102f);
    else if (avg_adc >= 2620) return interpolate_mA(avg_adc, 2620, 300, 10.0f / 98.0f);
    else if (avg_adc >= 2530) return interpolate_mA(avg_adc, 2530, 290, 10.0f / 90.0f);
    else if (avg_adc >= 2478) return interpolate_mA(avg_adc, 2478, 280, 10.0f / 52.0f);
    else if (avg_adc >= 2340) return interpolate_mA(avg_adc, 2340, 270, 10.0f / 138.0f);
    else if (avg_adc >= 2276) return interpolate_mA(avg_adc, 2276, 260, 10.0f / 64.0f);
    else if (avg_adc >= 2165) return interpolate_mA(avg_adc, 2165, 250, 10.0f / 111.0f);
    else if (avg_adc >= 2075) return interpolate_mA(avg_adc, 2075, 240, 10.0f / 90.0f);
    else if (avg_adc >= 2000) return interpolate_mA(avg_adc, 2000, 230, 10.0f / 75.0f);
    else if (avg_adc >= 1900) return interpolate_mA(avg_adc, 1900, 220, 10.0f / 100.0f);
    else if (avg_adc >= 1812) return interpolate_mA(avg_adc, 1812, 210, 10.0f / 88.0f);
    else if (avg_adc >= 1728) return interpolate_mA(avg_adc, 1728, 200, 10.0f / 84.0f);
    else if (avg_adc >= 1628) return interpolate_mA(avg_adc, 1628, 190, 10.0f / 100.0f);
    else if (avg_adc >= 1540) return interpolate_mA(avg_adc, 1540, 180, 10.0f / 88.0f);
    else if (avg_adc >= 1440) return interpolate_mA(avg_adc, 1440, 170, 10.0f / 100.0f);
    else if (avg_adc >= 1350) return interpolate_mA(avg_adc, 1350, 160, 10.0f / 90.0f);
    else if (avg_adc >= 1260) return interpolate_mA(avg_adc, 1260, 150, 10.0f / 90.0f);
    else if (avg_adc >= 1168) return interpolate_mA(avg_adc, 1168, 140, 10.0f / 92.0f);
    else if (avg_adc >= 1075) return interpolate_mA(avg_adc, 1075, 130, 10.0f / 93.0f);
    else if (avg_adc >= 994)  return interpolate_mA(avg_adc, 994,  120, 10.0f / 81.0f);
    else if (avg_adc >= 896)  return interpolate_mA(avg_adc, 896,  110, 10.0f / 98.0f);
    else if (avg_adc >= 800)  return interpolate_mA(avg_adc, 800,  100, 10.0f / 96.0f);
    else if (avg_adc >= 708)  return interpolate_mA(avg_adc, 708,  87,  10.0f / 92.0f);
    else if (avg_adc >= 612)  return interpolate_mA(avg_adc, 612,  77,  10.0f / 96.0f);
    else if (avg_adc >= 533)  return interpolate_mA(avg_adc, 533,  67,  10.0f / 79.0f);
    else if (avg_adc >= 440)  return interpolate_mA(avg_adc, 440,  57,  10.0f / 93.0f);
    else if (avg_adc >= 351)  return interpolate_mA(avg_adc, 351,  45.5f, 10.0f / 89.0f);
    else if (avg_adc >= 255)  return interpolate_mA(avg_adc, 255,  35.5f, 10.0f / 96.0f);
    else if (avg_adc >= 159)  return interpolate_mA(avg_adc, 159,  25.5f, 10.0f / 96.0f);
    else if (avg_adc >= 58)   return interpolate_mA(avg_adc, 58,   15.5f, 10.0f / 101.0f);
    else if (avg_adc >= 18)   return interpolate_mA(avg_adc, 18,   5.5f,  10.0f / 40.0f);
    else                      return 0.0f;
}


static inline int16_t roundi(float x) { return (int16_t)(x >= 0.f ? x + 0.5f : x - 0.5f); }

/////////////////////////////////////////////////////////////////////////
//ADC 변환 함수
//////////////////////////////////////////////////////////////////////////

void put_PWM_current_Value(void)
{
    static uint16_t _count_average_current = 0u;
    static uint32_t on_period_cnt = 0u;          
    static uint32_t acc_on_sum = 0u;      
    static float filtered_mA = 0.0f;
    static uint16_t last_stable_mA = 0u;  

    uint32_t cur_adc = (uint32_t)adc_dma_values[ADC_CH_C_IDX];

    // 1. 샘플링 및 데이터 누적
    if (cur_adc > 50) 
    {
        on_period_cnt++;
        acc_on_sum += cur_adc;
    }
    _count_average_current++;

    // 2. 계산 처리 
    if (_count_average_current >= TOTAL_SAMPLE_LIMIT)
    {
        if (on_period_cnt > 25) 
        {
            uint32_t duty_calc = (on_period_cnt * 100u) / _count_average_current;
            float avg_adc = (float)acc_on_sum / (float)on_period_cnt;
            float final_mA = 0.0f;

            test5 = (float)duty_calc;
            test6 = avg_adc;

            // 3. 듀티비 판별 및 mA 계산
            if        (duty_calc > 55)  final_mA = calculate_70_duty_mA(avg_adc);
            else if   (duty_calc > 43)  final_mA = calculate_50_duty_mA(avg_adc);
            else                        final_mA = calculate_40_duty_mA(avg_adc);
          
            // 4. 노이즈 제거를 위한 필터링
            if (filtered_mA < 1.0f) filtered_mA = final_mA; 
            filtered_mA = (final_mA * 0.08f) + (filtered_mA * 0.92f);
            
            // 반올림 처리
            uint16_t current_round_mA = (uint16_t)(filtered_mA + 0.5f);

            
            if (abs((int16_t)current_round_mA - (int16_t)last_stable_mA) >= 2) last_stable_mA = current_round_mA;
         
        }  
        else 
        {
            // 전류가 흐르지 않는 경우
            last_stable_mA = 0;
            filtered_mA = 0;
        }
        // 5. 최종 결과 저장
        measured_PWM_Cur = last_stable_mA;

        // 6. 변수 초기화
        on_period_cnt = 0;
        acc_on_sum = 0;
        _count_average_current = 0;
        current_ready = 1;

    }
}


void put_Temp_Value(void)
{

	converted_value_temper = (uint32_t)adc_dma_values[ADC_CH_TEMP_IDX];

	temper_measure_sum += converted_value_temper;
	count_measure_temper++;

	if(count_measure_temper >= TEMP_SAMPLE_WINDOW)
	{

		_averageTemp_max_sampling = (uint32_t)(temper_measure_sum/count_measure_temper);

		_voltage_Level_temp = (float)((_averageTemp_max_sampling * 3.3f) / 4095.0f);
		 int16_t t = roundi( (_voltage_Level_temp - 0.5f) * 100.0f + 5.0f );  // +5°C 오프셋 보정
     
		if (t < -40) t = -40;
		if (t > 150) t = 150;

		measured_TEMP_Val = (int16_t)t;


		temper_measure_sum 	 = 0;
		count_measure_temper = 0;

	}

}

void put_CDS_value(void)
{

	converted_value_cds = (uint32_t)adc_dma_values[ADC_CH_CDS_IDX];

	CDS_measure_sum += converted_value_cds;
	count_measure_CDS++;

  if(count_measure_CDS >= CDS_SAMPLE_WINDOW)
  {

    _averageCds_max_sampling = (uint32_t)(CDS_measure_sum / count_measure_CDS);

    CDS_measure_avg = 4095u-_averageCds_max_sampling;

    uint32_t scaled = (CDS_measure_avg * 1000u+2047) / 4095u; // 0~3.3V 스케일링
    uint16_t quant = (uint16_t)(((scaled + 5u) / 10u) * 10u);

    if(quant > 1000u) quant = 1000u;
    measured_CDS_Val = (uint16_t)quant;

    count_measure_CDS = 0;
    CDS_measure_sum = 0;

  }


}

void put_DC_value(void)
{
  
  uint32_t raw = (uint32_t)adc_dma_values[ADC_CH_DC_IDX];

  DC_measure_sum += raw;
  count_measure_DC++;

  // 10샘플 평균 후 전압 변환 (약 1초 주기)
  if (count_measure_DC >= 10u)
  {
      uint32_t avg_adc = DC_measure_sum / count_measure_DC;
      
      measured_DC_Val = (uint16_t)((avg_adc * 123 + 3800) / 1000);
      if(measured_DC_Val < 10)measured_DC_Val =0;
      
      // 누적값 초기화
      DC_measure_sum   = 0u;
      count_measure_DC = 0u;
  }
}
