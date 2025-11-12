/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * 
  * 
  * 2025.10.30
  * Made by Hyunil Song
  * ABS LED smart controller for railway signal lights
  * version 0.1V
  * 
  * todo : 
  * make adc table for PWM currnet control
  * implement uart communication protocol for monitoring and control
  * more setup values for setup pins
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stm32f1xx_ll_gpio.h"
#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct {
    GPIO_TypeDef * const port;
    const uint16_t       pin;
} PinDef;

enum
{
	Night =0,
	Day,
	Group,
	G_set,
	R_set,
	Unguide,
	Shunting,
	Signalling_Block,
	Pin_Count
};


static const PinDef kpins[Pin_Count] = {
		[Night] = {Night_GPIO_Port,Night_Pin},
		[Day]	= {Day_GPIO_Port, Day_Pin},
		[Group]	= {Group_GPIO_Port, Group_Pin},
		[G_set]	= {G_SET_GPIO_Port, G_SET_Pin},
		[R_set]	= {R_SET_GPIO_Port, R_SET_Pin},
		[Unguide]	= {Unguide_GPIO_Port, Unguide_Pin},
		[Shunting]	= {Shunting_GPIO_Port, Shunting_Pin},
		[Signalling_Block]	= {Signalling_Block_GPIO_Port, Signalling_Block_Pin},

};

typedef enum {
  MODE_OWNER_SWITCH = 0,   // 스위치가 세운 모드
  MODE_OWNER_AUTO   = 1,   // 고장으로 인한 자동절체가 세운 모드
} ModeOwner_t;

static ModeOwner_t mode_owner = MODE_OWNER_SWITCH;
static bool        fault_latched = false;
static uint32_t    hold_until_ms = 0;           // 자동절체 후 스위치 무시 기간
#define HOLD_MS_AFTER_FAILOVER  5000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RUN_LED_TIMEOUT_MS  900u
#define RUN_PWM_TIMEOUT_MS  100u
#define FAIL_TIMEOUT_MS     1000u
#define DATA_FRAME_SIZE		  0x17
#define STARTUP_BOOT_TIMEOUT_MS 3000u

#define PWM_full_duty_cycle  250u  // 100% duty for 250 max
#define PWM_seventy_duty_cycle  175u  // 70% duty for 250 max
#define PWM_sixty_duty_cycle  150u  // 60% duty for 250 max
#define PWM_half_duty_cycle  125u  // 50% duty for 250 max
#define PWM_fourty_duty_cycle  100u  // 40% duty for 250 max
#define PWM_zero_duty_cycle  0u    // 0% duty for 250 max


#define MV_ASSERT_MS   100u  
#define MV_RELEASE_MS  900u   // HIGH로 '해제'할 최소 지속시간
#define MV_RELEASEMAX_MS  1000u
#define IGNORE_HIGH_MS  220u  // 190ms보다 살짝 크게


static uint16_t mv_low_ms = 0;     // 연속 LOW 누적(ms)
static uint16_t mv_high_ms = 0;    // 연속 HIGH 누적(ms)
static uint8_t  mv_low_qualified = 0;   // 1=LOW로 인정됨(필터 후)
static uint8_t  mv_low_prev = 0;        // 에지 검출용


typedef enum { PO_IDLE=0, PO_ARMED, PO_ACTIVE } PoState_t;

// === 창 길이 선택 매크로 ============================================
// 250us 샘플링 주기 고정 가정
#define SAMPLE_PERIOD_US        250

// 한 개만 켜두세요.
#define AVERAGE_WINDOW_MS    100   // 100ms 창
//#define AVERAGE_WINDOW_MS       500   // 500ms 창 (기본)

// 창당 샘플 개수 = (창 길이 / 250us)
#define SAMPLES_PER_WINDOW  ((AVERAGE_WINDOW_MS * 1000) / SAMPLE_PERIOD_US)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Y_Pin_set			= 0;
uint8_t Y1_Pin_set			= 0;
static uint8_t transmit_buf[17]={0,};


uint8_t LED_setup;
uint8_t card_setup;
uint8_t LED_status;


static uint16_t watchdog_100ms_counter = 0;
static uint8_t  external_watchdog_enabled = TRUE;   // 필요하면 나중에 0으로 꺼도 됨
static uint8_t  external_watchdog_state   = 0;   

static uint16_t count_DI_100ms = 0;


static unsigned char  flag_RUNLED_ON   	  			= FALSE;
static unsigned char  flag_check_fault     			= FALSE;
static unsigned char  flag_transmit	   	  			= FALSE;
static unsigned char  flag_check_output	  			= FALSE;
static unsigned char  flag_Watchdog_start			= FALSE;
static unsigned char  flag_Fail	          			= FALSE;
static unsigned char  flag_check_input	   	 		= FALSE;
static unsigned char  flag_startup_boot   			= FALSE;
static unsigned char  flag_wating_100ms	   			= FALSE;

static unsigned char  flag_measure_temp				= FALSE;
static unsigned char  flag_measure_CDS				= FALSE;
static unsigned char  flag_measure_current			= FALSE;

static unsigned char  flag_Mainvolt_detected		= FALSE;


static uint16_t count_250us = 0;

static uint16_t count_5ms  = 0;
static uint16_t count_10ms = 0;
static uint16_t count_20ms = 0;
static uint16_t count_30ms = 0;
static uint16_t count_40ms = 0;
static uint16_t count_50ms = 0;

static uint16_t count_100ms = 0;
static uint16_t count_200ms = 0;
static uint16_t count_300ms = 0;
static uint16_t count_400ms = 0;
static uint16_t count_500ms = 0;

static uint16_t count_1000ms = 0;
static uint16_t count_1600ms = 0;
static uint16_t count_3000ms = 0;
static uint16_t count_fail   = 0;

static uint16_t DI_countHIGH_190ms = 0;


static uint16_t count_TX_timeout = 0;
static uint16_t count_RX_timeout = 0;


static uint8_t Switch_val		    = 0;


static uint8_t LED1_failcount = 0;
static uint8_t LED2_failcount = 0;




uint8_t test1 = 0;
uint8_t test2 = 0;
uint8_t test3 = 0;
uint8_t test4 = 0;
uint8_t test5 = 0;
uint8_t test6 = 0;
uint8_t test7 = 0;
uint8_t test8 = 0;

static RunMode_t      	run_mode;
static OutputResult_t	set_pinup;
///////////////////////ADC 변수////////////////////////
//New ADC
#define SAMPLING_COUNT_AC_VOLTAGE 167u	// AC 전압 1차 샘플링 회수 (1ms * 167 = 167ms)
#define MEASURED_AC_VOLTAGE 20u			// AC 전압 2차 샘플링 회수 (167ms * 20 = 3340ms)
#define MAX_CUR_ADC_CNT		50			// AC 전류 샘플링 회수 (1ms * 50 = 50ms)
#define SEQ_FAIL_MAX_CNT	5			// 시퀀스 에러 최대 횟수 (1초주기 체크)


#define THRESHOLD_FIRST_MIN   250  // ADC 값 일단 1,2계 동일 ADC 값 그대로 가져오기 (테스트용)
#define THRESHOLD_SECOND_MIN  250  // 필요하면 값 다르게 둘 수도 있음

// ADC 채널 정의 (V: 전압, C: 전류)
// ADC DMA 버퍼 인덱스와 매핑됩니다.
#define ADC_CH_C_IDX 0 // ADC_CHANNEL_0 (기존 전압 채널2)
#define ADC_CH_TEMP_IDX 1 // ADC_CHANNEL_2
#define ADC_CH_CDS_IDX 2 // ADC_CHANNEL_1
#define ADC_CH_DC_IDX 3 // ADC_CHANNEL_3
//#define ADC_CH_IDX 4 // ADC_CHANNEL_4 (기존 전류 채널3)
//#define ADC_CH_IDX 5 // ADC_CHANNEL_5
//#define ADC_CH_IDX 6 // ADC_CHANNEL_6
//#define ADC_CH_IDX 7 // ADC_CHANNEL_7

#define NUM_PWM_CURRENT_CHANNELS      1
#define NUM_DC_VOLATAGE_CHANNELS      1
#define NUM_CDS_CHANNELS              1
#define NUM_TEMP_CHANNELS             1
#define NUM_ADC_CHANNELS (NUM_PWM_CURRENT_CHANNELS + NUM_DC_VOLATAGE_CHANNELS + NUM_CDS_CHANNELS + NUM_TEMP_CHANNELS)

uint64_t measured_PWM_Cur  	= 0;
int16_t measured_TEMP_Val  = 0;
uint16_t measured_CDS_Val	= 0;
uint16_t measured_DC_Val	= 0;

//uint16_t adc_dma_values[NUM_ADC_CHANNELS];

volatile uint32_t adc_dma_values[NUM_ADC_CHANNELS];


static uint16_t	_count_sampling_tx_current  = 0u;
static uint32_t	_average_max_sampling   	= 0u;
static uint32_t	_average_max_sampling_cur  	= 0u;
static float 	_voltage_Level_tx_current		= 0;



static uint16_t count_measure_temper		= 0u;
static uint32_t	temper_measure_sum  		= 0u;
static uint32_t	_averageTemp_max_sampling   = 0u;
static float 	_voltage_Level_temp			= 0;

static uint16_t count_measure_CDS			= 0u;
static uint32_t	CDS_measure_sum  			= 0u;
static uint32_t	_averageCds_max_sampling   	= 0u;
static float 	_voltage_Level_CDS			= 0;


uint32_t converted_value_cds;
uint32_t converted_value_temper;
uint32_t converted_value_tx_currnet;

//--------------fnd ǥ�� ���� ����-----------------------------------------
uint8_t _fnd0[5];
uint8_t _fnd1[5];

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


static unsigned char _flag_make_display_data = FALSE;
///////////////////////UART 수신////////////////////////
static uint8_t COM1_data = 0; // UART1  1바이트 수신 데이터



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

static void GPIO_O_EXTERNAL_WATCHDOG_CLOCK(uint8_t v);
static void external_watchdog_clock_output_irq_handler (void);
static void watchdog_run_irq_handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	if(htim->Instance == TIM2)
	{
		count_5ms++;
		count_10ms++;
		count_50ms++;
		count_100ms++;
		count_500ms++;
		count_1000ms++;
		count_1600ms++;
		count_3000ms++;
		count_TX_timeout++;

		if(count_5ms >= 5)
		{

		  count_5ms = 0;
		}

		if(count_10ms >= 10)
		{

			count_10ms = 0;
		}


		if(count_50ms >=50)
		{
			flag_check_fault = TRUE;
			count_50ms = 0;
		}

		if(count_100ms >= 100)
		{
			flag_measure_CDS = TRUE;
			flag_measure_temp = TRUE;
		  flag_check_input = TRUE;
		  count_100ms = 0;
		}
		if(count_500ms >= 500)
		{

			_flag_make_display_data = TRUE;
			count_500ms = 0;
		}
		if(count_1000ms >= 1000)
		{

			flag_transmit = TRUE;
			count_1000ms = 0;
		}
		if(count_1600ms >= 1600)
		{

		}

		if(count_3000ms >= 3000)
		{


			count_3000ms = 0;
		}
		//watchdog_run_irq_handler();
	}

	if(htim->Instance == TIM3)
	{

	}

	if(htim->Instance == TIM4)
	{
		output_fnd_display();

	}

	if(htim->Instance == TIM5)
	{
		count_250us++;
		if(count_250us>=5)
		{
			flag_measure_current = TRUE;
			count_250us=0;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {

    	//PWM_current_accuarte();
	}
}

#if 1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{

	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{

	}
}
#endif

void HAL_SYSTICK_Callback(void)
{
	flag_Mainvolt_detected = TRUE;
    // 1) 메인전압 시간 필터 갱신(매 1ms)

}



/////////////////////////////////////////////////////////////////////////
//기본 설정 함수
/////////////////////////////////////////////////////////////////////////
Switchstate_t Get_switchState(uint8_t raw_bit)
{

	Switchstate_t state;

	state.Night_pin_set = (raw_bit >> Night)& 1u;
	state.Day_pin_set = (raw_bit >> Day)& 1u;
	state.Group_pin_set = (raw_bit >> Group)& 1u;
	state.G_Pin_set = (raw_bit >> G_set)& 1u;
	state.R_Pin_set = (raw_bit >> R_set)& 1u;
	state.Unguide_pin_set = (raw_bit >> Unguide)& 1u;
	state.Shunting_pin_set = (raw_bit >> Shunting)& 1u;
	state.Signalling_pin_set = (raw_bit >> Signalling_Block)& 1u;

	return state;

}

// 하드웨어 반영 + 논리결과를 반환
static OutputResult_t ApplyOutputsFromSwitch(Switchstate_t *s)
{
    OutputResult_t res;
    res.y_on        = 0u;
    res.applied_pwm = 0u;

    if (s->Night_pin_set == LOW)
    {
      res.night_active = HIGH;
    }
    else if (s -> Night_pin_set == HIGH)
    {
      res.night_active = LOW;
    }
  
      
    if (s->Day_pin_set == LOW)
    {
      res.day_active = HIGH;
    }
    else if(s ->Day_pin_set == HIGH)
    {
      res.day_active = LOW;
    }

    if (s->Group_pin_set  == LOW)
    {
      res.Group_active = HIGH;  // 그룹 모드
      res.Insert_active = LOW;
    }
    else if (s->Group_pin_set  == HIGH)
    {
      res.Insert_active = HIGH; // 삽입 모드
      res.Group_active = LOW;
    }


   
    if (s->G_Pin_set == LOW && s->R_Pin_set == LOW)
    {
        res.y_on = HIGH;
        Y_Pin_set = 1u;
    }
    else if(s->G_Pin_set == HIGH && s->R_Pin_set == HIGH)
    {
    	res.y1_on = HIGH;
    	Y1_Pin_set = HIGH;
    }

    if (s->G_Pin_set == LOW && s->R_Pin_set == HIGH)
    {
      res.G_active = HIGH;
      res.R_active = LOW;
    }
    else if (s->R_Pin_set == LOW && s->G_Pin_set == HIGH)
    {
    	res.R_active = HIGH;
    	res.G_active = LOW;
    }
 

    if (s->Unguide_pin_set == LOW) res.unguide_active = HIGH; 		// 무유도(Unguide) 모드
    else if(s->Unguide_pin_set == HIGH)  res.unguide_active = LOW;

    if (s->Signalling_pin_set == LOW) res.signalling_active = HIGH; // 폐색(Signalling) 모드
    else if (s->Signalling_pin_set == HIGH) res.signalling_active = LOW;
    
    if (s->Shunting_pin_set == LOW) res.shunting_active = HIGH;
    else if(s->Shunting_pin_set == HIGH) res.shunting_active = LOW;



    return res;
}


void app_setup(void)
{

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_values, NUM_ADC_CHANNELS);



    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart1, &COM1_data, 1);
    HAL_UART_Transmit_IT(&huart1, transmit_buf, 17);



	flag_RUNLED_ON = TRUE;

	HAL_GPIO_WritePin(CUR_CON0_GPIO_Port, CUR_CON0_Pin, GPIO_PIN_RESET);


	uint8_t raw_bits = 0; // 1) 스위치 전체 비트 읽기
	for (uint8_t i = 0; i < Pin_Count; i++)
	{
		if (HAL_GPIO_ReadPin(kpins[i].port, kpins[i].pin) == GPIO_PIN_SET)
		{
			raw_bits |= (1u << i);
		}
	}

	Switch_val = raw_bits;

	Switchstate_t sw = Get_switchState(Switch_val);// 2) 스위치 상태 구조체로 변환
	OutputResult_t result = ApplyOutputsFromSwitch(&sw);// 3) 하드웨어/카드 상태 반영
	set_pinup = result;

	//test 할때 주석 풀자
	//if(set_pinup.day_active == HIGH )
	//{
	//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 125);
	//}
	//else if (set_pinup.night_active == HIGH ) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 125);
   //else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_seventy_duty_cycle);


	HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_SET); // 3) 전원제어 라인 On
	HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_RESET);

	// 4) 1계/2계 LED 처리
	if (HAL_GPIO_ReadPin(LED1_INPUT_GPIO_Port, LED1_INPUT_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_RESET);
	}
	else if (HAL_GPIO_ReadPin(LED2_INPUT_GPIO_Port, LED2_INPUT_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_SET);
	}
  
}


/////////////////////////////////////////////////////////////////////////
//워치독
/////////////////////////////////////////////////////////////////////////
static void GPIO_O_EXTERNAL_WATCHDOG_CLOCK(uint8_t v)
{
    HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


static void external_watchdog_clock_output_irq_handler (void)
{
	if (external_watchdog_enabled)
	{
		external_watchdog_enabled = (0u == external_watchdog_enabled)?1u:0u;
		GPIO_O_EXTERNAL_WATCHDOG_CLOCK(external_watchdog_enabled);
	}

}

static void watchdog_run_irq_handler(void)
{
	if(run_mode !=MODE_FAULT)
	{
		 watchdog_100ms_counter++;

		if (watchdog_100ms_counter >= 100)
		{     // 100ms마다
			external_watchdog_clock_output_irq_handler();
			watchdog_100ms_counter = 0;
		}

	}

}

/////////////////////////////////////////////////////////////////////////
//PI, PO 함수
/////////////////////////////////////////////////////////////////////////


void mv_qualifier_tick(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(Main_Volt_detecto_GPIO_Port, Main_Volt_detecto_Pin);

    if (raw == GPIO_PIN_SET)
    {
        if (DI_countHIGH_190ms < 0xFFFF) DI_countHIGH_190ms++;

        // HIGH가 충분히 길면 해제
        if (DI_countHIGH_190ms >= IGNORE_HIGH_MS)
        {
            HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CUR_CON0_GPIO_Port, CUR_CON0_Pin, GPIO_PIN_RESET);
            count_DI_100ms = 0;
            DI_countHIGH_190ms = IGNORE_HIGH_MS;
        }
        return;   // 220ms 미만 HIGH는 완전히 무시
    }

    else if (raw == GPIO_PIN_RESET && flag_Mainvolt_detected == TRUE)
    {
        // LOW 진입: HIGH 카운터는 즉시 0으로
        DI_countHIGH_190ms = 0;

        count_DI_100ms++;
        if (count_DI_100ms >= MV_ASSERT_MS && count_DI_100ms < MV_RELEASE_MS)
        {
            if (set_pinup.Group_active == HIGH)
            {
                HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_RESET);
            }
            else if (set_pinup.Insert_active == HIGH)
            {
                HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_RESET);
            }

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 25);
        }

        if (count_DI_100ms >= MV_RELEASE_MS)
        {

            HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_RESET);


            if(set_pinup.day_active == HIGH )__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 125);
		    else if (set_pinup.night_active == HIGH ) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 125);
            //count_DI_100ms = MV_RELEASE_MS;

        }

        /*test용 주석 이후 주석 삭제.*/
        if (count_DI_100ms >= MV_RELEASEMAX_MS)
		{
            HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CUR_CON0_GPIO_Port, CUR_CON0_Pin, GPIO_PIN_SET);
			count_DI_100ms = MV_RELEASEMAX_MS;
		}
        flag_Mainvolt_detected=FALSE;
    }
}




/////////////////////////////////////////////////////////////////////////
//1,2계 절체 함수
/////////////////////////////////////////////////////////////////////////
RunMode_t read_setup_switch(void)
{
	uint8_t sw1_raw = HAL_GPIO_ReadPin(First_sw_in_GPIO_Port, First_sw_in_Pin);
	uint8_t sw2_raw = HAL_GPIO_ReadPin(Second_sw_in_GPIO_Port, Second_sw_in_Pin);

    if      (sw1_raw==HIGH && sw2_raw==LOW)  	return MODE_FIRST_ACTIVE;
    else if (sw1_raw==LOW  && sw2_raw==HIGH) 	return MODE_SECOND_ACTIVE;
    else if (sw1_raw==LOW  && sw2_raw==LOW)  	return MODE_SWITCH_OFF;
    else                           				return MODE_INVALID;

}

void control_logic_step(void)
{

    if ( run_mode == MODE_FAULT)  // 0) FAULT 확인
    {
        run_mode = MODE_FAULT;
        HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(First_LED_GPIO_Port,  First_LED_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_SET);
        return;
    }


    if (HAL_GetTick() >= hold_until_ms) // 1) 자동절체 직후 HOLD 종료 시 스위치 반영
    {
        RunMode_t req = read_setup_switch();

        if (req == MODE_FIRST_ACTIVE || req == MODE_SECOND_ACTIVE)
        {
            mode_owner = MODE_OWNER_SWITCH;   // 스위치 소유
            if (run_mode != req) run_mode = req;
        }
        else if (req == MODE_SWITCH_OFF) // 스위치 고장 시 2계 작동
        {
        	run_mode = MODE_SECOND_ACTIVE;
        }

    }

    switch (run_mode)  // 3) run_mode에 따른 출력
    {
        case MODE_FIRST_ACTIVE:
            HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_RESET);
            break;

        case MODE_SECOND_ACTIVE:
            HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////
//고장 검출 함수
/////////////////////////////////////////////////////////////////////////

int check_drop_PWMcurrent(uint32_t prev_val, uint32_t cur)
{

	return (cur * 100) <= (prev_val * 70);
}

void check_Fault(void)
{
	uint8_t Max_count_LED = 60;

	GPIO_PinState Read_Main_power = HAL_GPIO_ReadPin(Main_Volt_detecto_GPIO_Port, Main_Volt_detecto_Pin);

	static uint32_t prev_cur_first  = 0;
	static uint32_t prev_cur_second = 0;

	if(Read_Main_power == HIGH && flag_check_fault == FALSE) return;
	else if(Read_Main_power == HIGH && flag_check_fault == TRUE) return;


	flag_check_fault = FALSE;


	if(run_mode == MODE_FIRST_ACTIVE )
	{

		if(check_drop_PWMcurrent(prev_cur_first, measured_PWM_Cur))//(measured_PWM_Cur <= THRESHOLD_FIRST_MIN)
		{
			if(LED1_failcount<0xff) LED1_failcount++;

			if(LED1_failcount > Max_count_LED)
			{
				run_mode = MODE_SECOND_ACTIVE;
				mode_owner   = MODE_OWNER_AUTO;
				hold_until_ms = HAL_GetTick() + HOLD_MS_AFTER_FAILOVER;  // 스위치 무시 기간
				//flag_LED1_Fail = TRUE;
				LED1_failcount = 60;
			}

		}
		else LED1_failcount = 0;
		prev_cur_first = measured_PWM_Cur;
		return;

	}


	else if(run_mode == MODE_SECOND_ACTIVE)
	{

		if(measured_PWM_Cur <= check_drop_PWMcurrent(prev_cur_second, measured_PWM_Cur))//(measured_PWM_Cur <= THRESHOLD_SECOND_MIN)
		{
			if(LED2_failcount<0xff) LED2_failcount++;

			if(LED2_failcount > 60)
			{
				run_mode = MODE_FAULT;
				flag_Fail		= TRUE;
				//flag_LED2_Fail 	= TRUE;
				LED2_failcount 	= 60;
			}

		}
		else LED2_failcount = 0;
		prev_cur_second = measured_PWM_Cur;
		return;


	}

}


void Fail_setup(void)
{
	if(run_mode == MODE_FAULT)
	{
		HAL_GPIO_WritePin(Fault_Relay_Control_GPIO_Port, Fault_Relay_Control_Pin, GPIO_PIN_RESET);



	}

}




/////////////////////////////////////////////////////////////////////////
//ADC 변환 함수
//////////////////////////////////////////////////////////////////////////
uint16_t get_PWM_cur_Value(void)
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


void put_PWM_current_Value(void)
{

    if (flag_measure_current == FALSE)
    {
        return;  // 250us 주기로 TRUE로 세워줘야 함
    }


	converted_value_tx_currnet = adc_dma_values[ADC_CH_C_IDX];

	_average_max_sampling += converted_value_tx_currnet;
	_count_sampling_tx_current++;


    if (_count_sampling_tx_current >= 4000)
    {

        _average_max_sampling_cur = (uint32_t)(_average_max_sampling / _count_sampling_tx_current);

        // 전류 (12-bit, 0~4095)
        _voltage_Level_tx_current = (float)((_average_max_sampling_cur * 3.3f) / 4095.0f);
        // 2. 하드웨어 상수
        const float SHUNT_RESISTOR_OHM = 7.8f;
        const float OP_AMP_GAIN = 7.8f;
        const float SENSE_DIVIDER = OP_AMP_GAIN * SHUNT_RESISTOR_OHM; // 예: 20 * 0.1 = 2.0

        // 3. 전압(V)을 전류(A)로 변환
        float current_in_Amps = _voltage_Level_tx_current / SENSE_DIVIDER;

        // 4. 단위를 밀리암페어(mA)로 변환하여 저장
        if(current_in_Amps != 0)measured_PWM_Cur = (unsigned int)(current_in_Amps * 10000.0f)+75;
        else measured_PWM_Cur = 0;

        // 창 리셋
        _average_max_sampling = 0u;
		_count_sampling_tx_current = 0u;
    }

	flag_measure_current = FALSE;


}

static inline int16_t roundi(float x) { return (int16_t)(x >= 0.f ? x + 0.5f : x - 0.5f); }

void put_Temp_Value(void)
{

	if(FALSE == flag_measure_temp)
	{
		return;
	}

	flag_measure_temp = FALSE;


	converted_value_temper = (uint32_t)adc_dma_values[ADC_CH_TEMP_IDX];

	temper_measure_sum += converted_value_temper;
	count_measure_temper++;

	if(count_measure_temper >= 30)
	{

		_averageTemp_max_sampling = (uint32_t)(temper_measure_sum/count_measure_temper);

		_voltage_Level_temp = (float)((_averageTemp_max_sampling * 3.3f) / 4095.0f);
		 int16_t t = roundi( (_voltage_Level_temp - 0.5f) * 100.0f );
		if (t < -40) t = -40;
		if (t > 150) t = 150;
		measured_TEMP_Val = (int16_t)t;

		//measured_TEMP_Val = (_voltage_Level_temp * 40); // 섭씨 온도로 변환

		temper_measure_sum 	 = 0;
		count_measure_temper = 0;

	}

}

void put_CDS_value(void)
{

	if(FALSE == flag_measure_CDS)
	{
		return;
	}

	flag_measure_CDS = FALSE;

	converted_value_cds = (uint32_t)adc_dma_values[ADC_CH_CDS_IDX];

	CDS_measure_sum += converted_value_cds;
	count_measure_CDS++;

	if(count_measure_CDS >= 100)
	{
		_averageCds_max_sampling = (uint32_t)(CDS_measure_sum/count_measure_CDS);

		_voltage_Level_CDS = (float)(_averageCds_max_sampling * 3.3f)/4095.0f;

		measured_CDS_Val = (unsigned int)(_voltage_Level_CDS * 10000);

		count_measure_CDS = 0;
		CDS_measure_sum = 0;

	}


}

/////////////////////////////////////////////////////////////////////////
//통신 함수
//////////////////////////////////////////////////////////////////////////
uint16_t POW(uint16_t a, uint16_t b)
{
	uint16_t i, pow = 1;
	for (i = 0; i < b; i++)
	
	pow *= a; 
	return pow;
} 

void transmit(void)
{
	uint8_t i;

	if(flag_transmit == FALSE) return;
  
	flag_transmit = FALSE;
  

  if(run_mode == MODE_FIRST_ACTIVE)       LED_status = 0x01;
  else if(run_mode == MODE_SECOND_ACTIVE) LED_status = 0x02;
  else if(run_mode == MODE_FAULT)         LED_status = 0x03;

  if(set_pinup.shunting_active == HIGH)                 card_setup = 0x01;
  else if(set_pinup.unguide_active == HIGH)             card_setup = 0x02;
  else if(set_pinup.signalling_active == HIGH)          card_setup = 0x03;

  if(set_pinup.R_active == HIGH && set_pinup.G_active == LOW)             LED_setup = 0x01;
  else if(set_pinup.G_active == HIGH && set_pinup.R_active == LOW)        LED_setup = 0x02;
  else if(set_pinup.y_on == HIGH)                                         LED_setup = 0x03;
  else if(set_pinup.y1_on == HIGH)                                        LED_setup = 0x04;



	transmit_buf[0] = 0x02;
	transmit_buf[1]	= DATA_FRAME_SIZE;
	transmit_buf[2] = 0x11;
	transmit_buf[3] = 0x14;
	transmit_buf[4] = 0x45;

  
  for(i=0; i<3; i++)
  {
    transmit_buf[5+i] = (measured_PWM_Cur / POW(10, 2-i)) % 10 + '0';
  }
  for(i=0; i<3; i++)
  {
    transmit_buf[8+i] = (measured_TEMP_Val / POW(10, 2-i)) % 10 + '0';
  }
  transmit_buf[11] = (LED_status)%10 + '0';
  transmit_buf[12] = (card_setup)%10 + '0';
  transmit_buf[13] = (LED_setup) %10 + '0';

	transmit_buf[14]= 0x88;
	transmit_buf[15]= 0x88;
	transmit_buf[16]= 0x03;


	HAL_UART_Transmit_IT(&huart1, transmit_buf, 17);
	count_TX_timeout = 0;

}
/////////////////////////////////////////////////////////////////////////
//FND 함수
//////////////////////////////////////////////////////////////////////////
void display_maked_fnd_data (const char* numstring, uint8_t* bufptr, unsigned int bufsize)
{
	const uint8_t tbl[10] =
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

	const uint8_t max = 5;               // FIX: PD0~PD2만 사용 → 최대 3자리

	uint8_t fnd[5];                      // FIX: 실제 사용하는 자리수에 맞춰 3칸
	uint8_t ascii;

	unsigned int  i;
	unsigned int  position;

	unsigned int  len;
    len    = strlen(numstring);


	if (len >= 3)           position = 0;   // 3글자 이상은 왼쪽부터 꽉 채움
	else if (len == 2)      position = 1;   // 두 글자는 가운데부터 (우측 정렬)
	else if (len == 1)      position = 2;   // 한 글자는 맨 오른쪽
	else                    position = 0;
	// ---------------------

	fnd[0] = FND_CHAR_BLANK;
	fnd[1] = FND_CHAR_BLANK;
	fnd[2] = FND_CHAR_BLANK;

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
void fnd_write (unsigned int select, uint8_t fndvalue)
{
	uint16_t portd_value = 0; //FND9~FND15 //PD0~PE7
	uint16_t porte_value = 0; //FND0~FND8  //PE7~PE14
	volatile unsigned int i=0;
	uint16_t ReadValue_portD;
	uint16_t ReadValue_portE;

	ReadValue_portD = LL_GPIO_ReadInputPort(GPIOD);   // 쓰기 전에 기존값 읽기


	portd_value |= select;    //PD0~PD7
	LL_GPIO_WriteOutputPort(GPIOD, (ReadValue_portD & 0xff00) | portd_value);

	ReadValue_portE = LL_GPIO_ReadInputPort(GPIOE);   // 쓰기 전에 기존값 읽기

	porte_value  |= fndvalue << 8;             // ★ << 8
	LL_GPIO_WriteOutputPort(GPIOE, (ReadValue_portE & 0x00ff) | porte_value);

	for (i=0;i<0x10;i++);
//	GpioD->DATA = GpioD->DATA & 0xff00;
	LL_GPIO_WriteOutputPort(GPIOD,(ReadValue_portD & 0xff00));	//FND9~FND15 clear

//	GpioE->DATA = GpioE->DATA & 0x807f;
	LL_GPIO_WriteOutputPort(GPIOE,(ReadValue_portE & 0x00ff));	//FND0~FND8 clear

}
//-----------------------------------------------------------------------
void fnd_output_data(const unsigned int id, const uint8_t* bufptr, unsigned int bufsize)
{
	unsigned int  i;

	switch (id)
	{
		case  0: for (i=0;i<bufsize;i++) { fnd_write(FND_CH_00(1<<i),bufptr[i]); } break;

	}

}

//-----------------------------------------------------------------------
void output_fnd_display(void)
{
	fnd_output_data (0, _fnd0, 4);
	//fnd_output_data (1, _fnd1, 4);
}


void make_fnd_data (void)
{
	//-----------------------------------------------------------------------
	if (FALSE ==_flag_make_display_data)	//500ms �ֱ�
	{
		return;
	}

	_flag_make_display_data = FALSE;
	//-----------------------------------------------------------------------
	char first_line   [5]  = {0,};



	uint16_t  PWM_current;

	PWM_current = get_PWM_cur_Value();

	sprintf(first_line ,"%d", PWM_current);

	display_maked_fnd_data   (first_line,  _fnd0, 5);



}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  app_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if(run_mode != MODE_FAULT) HAL_GPIO_TogglePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin);
    else HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin,SET);

    put_PWM_current_Value();
    put_Temp_Value();

    make_fnd_data();
  
    //check_Fault();
    control_logic_step();
    //Fail_setup();
    mv_qualifier_tick();

    //transmit();




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 250-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 125;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, First_LED_Pin|Second_LED_Pin|Fault_Relay_Control_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin
                          |FND5_Pin|FND6_Pin|FND7_Pin|FND8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CUR_CON2_Pin|CUR_CON1_Pin|CUR_CON0_Pin|MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FNDA_Pin|FNDB_Pin|FNDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : First_LED_Pin Second_LED_Pin Fault_Relay_Control_Pin MCU_CONTROL_PWR_Pin */
  GPIO_InitStruct.Pin = First_LED_Pin|Second_LED_Pin|Fault_Relay_Control_Pin|MCU_CONTROL_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : First_sw_in_Pin Second_sw_in_Pin LED1_INPUT_Pin LED2_INPUT_Pin
                           Main_Volt_detecto_Pin */
  GPIO_InitStruct.Pin = First_sw_in_Pin|Second_sw_in_Pin|LED1_INPUT_Pin|LED2_INPUT_Pin
                          |Main_Volt_detecto_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FND1_Pin FND2_Pin FND3_Pin FND4_Pin
                           FND5_Pin FND6_Pin FND7_Pin FND8_Pin */
  GPIO_InitStruct.Pin = FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin
                          |FND5_Pin|FND6_Pin|FND7_Pin|FND8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CUR_CON2_Pin CUR_CON1_Pin CUR_CON0_Pin MCU_to_WATCHDOG_INPUT_Pin */
  GPIO_InitStruct.Pin = CUR_CON2_Pin|CUR_CON1_Pin|CUR_CON0_Pin|MCU_to_WATCHDOG_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Day_Pin Night_Pin Group_Pin Signalling_Block_Pin
                           Shunting_Pin Unguide_Pin R_SET_Pin G_SET_Pin */
  GPIO_InitStruct.Pin = Day_Pin|Night_Pin|Group_Pin|Signalling_Block_Pin
                          |Shunting_Pin|Unguide_Pin|R_SET_Pin|G_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FNDA_Pin FNDB_Pin FNDC_Pin */
  GPIO_InitStruct.Pin = FNDA_Pin|FNDB_Pin|FNDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : WATCHDOG_to_MCU_INPUT_Pin */
  GPIO_InitStruct.Pin = WATCHDOG_to_MCU_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WATCHDOG_to_MCU_INPUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
