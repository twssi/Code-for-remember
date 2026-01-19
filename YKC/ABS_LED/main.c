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
#include "calc.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

static ModeOwner_t mode_owner = MODE_OWNER_SWITCH;
static uint32_t    hold_until_ms = 0;           // 자동절체 후 스위치 무시 기간

// CDS 변경으로 인한 PWM 변경 시 고장 오검출 억제 기간 (flag_change_CDS_Val 사용)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


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

///////////////////////UART 송수신////////////////////////

static uint8_t COM1_data = 0; // UART1  1바이트 수신 데이터

static uint8_t transmit_buf[23]={0,};
static uint8_t receive_buf[8]={0,};


uint8_t mode_setup;
uint8_t LED_setup;
uint8_t card_setup;
uint8_t LED_status1;
uint8_t LED_status2;
uint8_t day_night_status;
uint8_t LDE_card_status;
uint8_t Y_Pin_set			= 0;
uint8_t Y1_Pin_set			= 0;
//////////////////////////////////////////////////////////

// ===== Fault Latch (고장 의심 상태 유지) =====
static uint8_t  fault_latch_1 = 0;         // 1계 latch
static uint8_t  fault_latch_2 = 0;         // 2계 latch
static uint32_t fault_ref_1   = 0;         // 1계 동결 기준(peak)
static uint32_t fault_ref_2   = 0;         // 2계 동결 기준(peak)

bool  flag_change_CDS_Val   	  = FALSE;


bool  flag_check_fault     			= FALSE;
bool  flag_transmit	   	  			= FALSE;
bool  flag_check_input	   	 		= FALSE;
bool  flag_change_Temp  	      = FALSE;


bool  flag_Mainvolt_detected		= FALSE;
bool  flag_LED1_fail            = FALSE;
bool  flag_LED2_fail            = FALSE;


static uint16_t count_5ms  = 0;
static uint16_t count_10ms = 0;
//static uint16_t count_20ms = 0;
//static uint16_t count_30ms = 0;
//static uint16_t count_40ms = 0;
static uint16_t count_50ms = 0;

static uint16_t count_100ms = 0;
static uint16_t count_200ms = 0;
static uint16_t count_300ms = 0;
static uint16_t count_400ms = 0;
static uint16_t count_500ms = 0;

static uint16_t count_1000ms = 0;
static uint16_t count_1600ms = 0;
static uint16_t count_2000ms = 0;
static uint16_t count_3000ms = 0;

static uint16_t receive_count = 0;

static uint16_t DI_countHIGH_190ms  = 0;
static uint16_t DI_countLOW_10ms    = 0;

static uint16_t count_TX_timeout = 0;
static uint16_t count_RX_timeout = 0;

static uint16_t count_DI_100ms = 0;

static uint8_t LED1_failcount = 0;
static uint8_t LED2_failcount = 0;

// 1계, 2계 채널 전류 피크 기준값 (30% 드롭 감지용)
static uint32_t peak_first  = 0;
static uint32_t peak_second = 0;


static uint8_t Switch_val		= 0;


// 1) 수동 절체 이벤트(1회성)
static volatile uint8_t g_manual_switchover_event = FALSE; // 0/1
// 2) 고장 절체 래치(지속)
static volatile uint8_t g_autofault_latched = FALSE;       // 0/1 (1이면 계속 2 전송)
//static uint32_t g_manual_event_until_ms = 0;
// static 변수를 사용하여 이전 모드 상태를 유지합니다.
static RunMode_t prev_run_mode = MODE_FIRST_ACTIVE;



bool flag_override_pwm        = FALSE;
bool flag_system_stabilizing  = FALSE; // 통합 안정화 상태 플래그

uint32_t stability_start_ms = 0;      // 안정화 시작 시점 기록
uint16_t last_DC_Val = 0;             // 이전 전압 비교용

static RunMode_t      	run_mode;
static OutputResult_t	  set_pinup;


static uint8_t current_state = CDS_DAY;   // 확정된 상태
static uint8_t candidate_state = CDS_DAY; // 현재 후보 상태

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	if(htim->Instance == TIM2)
	{

    timer_counter_run();
    watchdog_run_irq_handler();
    read_mainvolt();
    mv_qualifier_tick();
    
	
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
    
    put_PWM_current_Value();
  }
 
}

#if 1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
    OnUart_Recv(COM1_data);
    HAL_UART_Receive_IT(&huart1, &COM1_data, 1);
	}

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{

	}
}
#endif


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
    OutputResult_t res = {0};   

    // Night/Day
    res.night_active = (s->Night_pin_set == LOW) ? HIGH : LOW;
    res.day_active   = (s->Day_pin_set   == LOW) ? HIGH : LOW;

    // Group/Insert
    if (s->Group_pin_set == LOW) 
    {
        res.Group_active  = HIGH;
        res.Insert_active = LOW;
    } 
    else 
    {
        res.Group_active  = LOW;
        res.Insert_active = HIGH;
    }

    // (G/R/Y/Y1) 
    if (s->G_Pin_set == LOW && s->R_Pin_set == LOW) 
    {
        res.y_on     = HIGH;
        res.y1_on    = LOW;
        res.G_active = LOW;
        res.R_active = LOW;   // Y일 때 R_active를 명시적으로 LOW   
    }
    else if (s->G_Pin_set == HIGH && s->R_Pin_set == HIGH)
    {
        res.y_on     = LOW;
        res.y1_on    = HIGH;
        res.G_active = LOW;
        res.R_active = LOW;   // Y1일 때도 명시적으로 LOW
    }
    else if (s->G_Pin_set == LOW && s->R_Pin_set == HIGH) 
    {
        res.y_on     = LOW;
        res.y1_on    = LOW;
        res.G_active = HIGH;
        res.R_active = LOW;
    }
    else 
    { // (s->G_Pin_set == HIGH && s->R_Pin_set == LOW)
        res.y_on     = LOW;
        res.y1_on    = LOW;
        res.G_active = LOW;
        res.R_active = HIGH;
    }

    // Modes
    res.unguide_active    = (s->Unguide_pin_set    == LOW) ? HIGH : LOW;
    res.signalling_active = (s->Signalling_pin_set == LOW) ? HIGH : LOW;
    res.shunting_active   = (s->Shunting_pin_set   == LOW) ? HIGH : LOW;

    return res;
}


void app_setup(void)
{
  
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_values, NUM_ADC_CHANNELS);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart1, &COM1_data, 1);
  HAL_UART_Transmit_IT(&huart1, transmit_buf, 23);

	

  uint8_t raw_bits = 0; // 1) 스위치 전체 비트 읽기
	for (uint8_t i = 0; i < Pin_Count; i++)
	{
		if (HAL_GPIO_ReadPin(kpins[i].port, kpins[i].pin) == GPIO_PIN_SET)
		{
			raw_bits |= (1u << i);
		}
	}

	Switch_val = raw_bits;

	Switchstate_t sw = Get_switchState(Switch_val);// 2) 스위치 상태 
	OutputResult_t result = ApplyOutputsFromSwitch(&sw);// 3) 하드웨어/카드 설정 반영
	set_pinup = result;

	HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_SET); // 3) 전원제어 라인 On
  HAL_GPIO_WritePin(CUR_CON2_GPIO_Port, CUR_CON2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Fault_Relay_Control_GPIO_Port, Fault_Relay_Control_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);

  // 4) 1계/2계 LED 처리
	if (HAL_GPIO_ReadPin(LED1_INPUT_GPIO_Port, LED1_INPUT_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port,FAULT_LED_Pin,GPIO_PIN_SET);
	}
	else if (HAL_GPIO_ReadPin(LED2_INPUT_GPIO_Port, LED2_INPUT_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(First_LED_GPIO_Port, First_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port,FAULT_LED_Pin,GPIO_PIN_RESET);
	}
  
}


/////////////////////////////////////////////////////////////////////////
//워치독
/////////////////////////////////////////////////////////////////////////
void watchdog_run_irq_handler(void)
{
    static uint32_t wd_cnt = 0;

    if (run_mode != MODE_FAULT)
    {
        wd_cnt++;
        if (wd_cnt >= 100) wd_cnt = 0; // 100ms 주기로 반복

        // 0~49ms까지는 HIGH, 50~99ms까지는 LOW (50% 듀티 사이클)
        if (wd_cnt < 50) 
        {
            HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_SET);
        }
        else 
        {
            HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_RESET);
        }
    }
    else
    {
        HAL_GPIO_WritePin(MCU_to_WATCHDOG_INPUT_GPIO_Port, MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_SET);
    }
   
}

///////////////////////////////////////////////////////////////////////////
//타이머 카운터
///////////////////////////////////////////////////////////////////////////
void timer_counter_run(void)
{
  	count_5ms++;
		count_10ms++;
		count_50ms++;
		count_100ms++;
    count_200ms++;
    count_300ms++;
    count_400ms++;
		count_500ms++;
		count_1000ms++;
		count_1600ms++;
		count_2000ms++;
		count_3000ms++;
    count_RX_timeout++;
		count_TX_timeout++;
    receive_count++;

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
      put_CDS_value();
      put_Temp_Value();
      put_DC_value();
		  count_100ms = 0;
		}
		if(count_500ms >= 500)
		{

			make_fnd_data();
			count_500ms = 0;
		}
		if(count_1000ms >= 1000)
		{

			flag_transmit     = TRUE;
      flag_check_input  = TRUE; //PWM 전류값 측정 트리거
      
			count_1000ms = 0;
		}
		if(count_1600ms >= 1600)
		{
      
    }
		if(count_2000ms >= 2000)
    {
      count_2000ms = 0;
    }
		if(count_3000ms >= 3000)
		{
      
      
			count_3000ms = 0;
		}
		
    

}


/////////////////////////////////////////////////////////////////////////
//전압 입력 감지 함수
/////////////////////////////////////////////////////////////////////////

void read_mainvolt(void)
{
  
  GPIO_PinState raw = HAL_GPIO_ReadPin(Main_Volt_detecto_GPIO_Port, Main_Volt_detecto_Pin);

  if (raw == GPIO_PIN_SET) // 전압 미검출 (High)
  {

      DI_countLOW_10ms = 0; // Low 카운트 리셋

      if (DI_countHIGH_190ms < 0xFFFF) DI_countHIGH_190ms++;
      if (DI_countHIGH_190ms >= IGNORE_HIGH_MS)
      {
          flag_Mainvolt_detected = FALSE;
          
      }
  }

  else // 전압 검출 (Low)
  {

      DI_countHIGH_190ms = 0; // High 카운트 리셋

      if (DI_countLOW_10ms < 0xFFFF) DI_countLOW_10ms++;
      if (DI_countLOW_10ms >= IGNORE_HIGH_toLOW_MS)
      {
          flag_Mainvolt_detected = TRUE;
         
      }

  }

}

void mv_qualifier_tick(void)
{

    if (flag_Mainvolt_detected == FALSE)
    {
        count_DI_100ms = 0;


        flag_override_pwm = FALSE;


        HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CUR_CON0_GPIO_Port, CUR_CON0_Pin, GPIO_PIN_RESET);
        return;
    }

    else 
    {

        count_DI_100ms++;
  
      // 1) ASSERT (>= 50ms) 
      if ( count_DI_100ms >= MV_ASSERT_MS)
      {

        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_GPIO_WritePin(CUR_CON1_GPIO_Port, CUR_CON1_Pin, GPIO_PIN_SET);
        
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 125);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 25);

        flag_override_pwm = TRUE;
        
      }
  
      // 2) RELEASE (>= 900ms)
      if (count_DI_100ms >= MV_RELEASE_MS)
      {
      
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  
          flag_override_pwm = FALSE;
         
  
          // 더 이상 증가/오버플로 방지
          count_DI_100ms = MV_RELEASE_MS + 1;
      }
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
  else                           				    return MODE_SWITCH_OFF;

}

void control_logic_step(void)
{

    if ( run_mode == MODE_FAULT)  // 0) FAULT 확인
    {
        run_mode = MODE_FAULT;
        return;
    }

    if (HAL_GetTick() >= hold_until_ms) // 1) 자동절체 직후 HOLD 종료 시 스위치 반영
    {
        RunMode_t req = read_setup_switch();
        
        if (req == MODE_FIRST_ACTIVE || req == MODE_SECOND_ACTIVE)
        {
          
          if (mode_owner == MODE_OWNER_SWITCH)
          {
            if (run_mode != req) run_mode = req;
            
          }
   
        }
        else if (req == MODE_SWITCH_OFF) // 스위치 고장 시 2계 작동
        {
        	run_mode = MODE_SECOND_ACTIVE;
        }

    }

    //모든 모드 변경(자동/수동)을 감지하는 로직
    if (prev_run_mode != run_mode && mode_owner == MODE_OWNER_SWITCH)
    {
        // 1계 -> 2계, 2계 -> 1계 모든 변화를 여기서 캐치
        g_manual_switchover_event = TRUE;
        prev_run_mode = run_mode;// 디버깅 등을 위해 이전 상태 업데이트
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
void check_Fault(void)
{
    uint8_t Max_count_LED = 60;

    // 전압 없으면 판정은 하지 않음
    if (flag_Mainvolt_detected == FALSE) return;
    if (flag_check_fault == FALSE) return;
    if (!current_ready) return;

    // 안정화 기간엔 판정만 멈춤 (failcount/기준은 리셋하지 않음)
    if (flag_system_stabilizing == TRUE)
    {
        flag_check_fault = FALSE; 
        return;
    }

    flag_check_fault = FALSE;

    // ===== (2) run_mode별 고장 판정 =====
    if (run_mode == MODE_FIRST_ACTIVE)
    {
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);

        
        if (fault_latch_1 == 0 && LED1_failcount == 0)
        {
            if (peak_first == 0 || measured_PWM_Cur > peak_first)    peak_first = (uint32_t)measured_PWM_Cur;
        }

        uint32_t ref = (fault_latch_1 != 0) ? fault_ref_1 : peak_first;

        if (ref > 0 && (measured_PWM_Cur * 100u <= (ref * 70u) || measured_PWM_Cur * 100u >= (ref * 130u)))
        {
            // 고장 의심 시작 순간: 기준을 "현재 peak_first"로 동결
            
            if (fault_latch_1 == 0)
            {
              fault_latch_1 = 1;
              fault_ref_1 = peak_first;   // 직전 정상 기준을 동결
              if (fault_ref_1 == 0) fault_ref_1 = (uint32_t)measured_PWM_Cur; 
              ref = fault_ref_1;
            }

            if (LED1_failcount < 0xFF) LED1_failcount++;

            if (LED1_failcount > Max_count_LED) // 약 3초
            {
                run_mode = MODE_SECOND_ACTIVE;
                mode_owner = MODE_OWNER_AUTO;

                hold_until_ms = HAL_GetTick() + HOLD_MS_AFTER_FAILOVER;

                g_autofault_latched = TRUE;
                flag_LED1_fail = TRUE;

                LED1_failcount = Max_count_LED;
                current_ready = 0;

                // 1계 기준/래치 초기화 (절체 후에는 2계 기준을 새로 잡아야 함)
                peak_first = 0;
                fault_latch_1 = 0;
                fault_ref_1 = 0;
            }

        }
        else
        {
            // 정상이라면 failcount 리셋 + latch 해제 + 동결 기준 해제
            LED1_failcount = 0;
            fault_latch_1 = 0;
            fault_ref_1 = 0;
            flag_LED1_fail = FALSE;
        }
    }

    else if (run_mode == MODE_SECOND_ACTIVE)
    {
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    
        if (mode_owner == MODE_OWNER_AUTO) HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET); // ON 
        else if (mode_owner == MODE_OWNER_SWITCH) HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);   // OFF

        // peak 갱신은 latch가 없고 failcount가 0일 때만
        if (fault_latch_2 == 0 && LED2_failcount == 0)
        {
            if (peak_second < 10 || measured_PWM_Cur > peak_second)  peak_second = (uint32_t)measured_PWM_Cur;
        }

        uint32_t ref = (fault_latch_2 != 0) ? fault_ref_2 : peak_second;

        if (ref > 0 && (measured_PWM_Cur * 100u <= (ref * 70u) || measured_PWM_Cur * 100u >= (ref * 130u)))
        {
            if (fault_latch_2 == 0)
            {
                fault_latch_2 = 1;
                fault_ref_2 = peak_second;
                if (fault_ref_2 == 0) fault_ref_2 = (uint32_t)measured_PWM_Cur; // 최후 보호
                ref = fault_ref_2;
            }

            if (LED2_failcount < 0xFF) LED2_failcount++;

            if (LED2_failcount > Max_count_LED)
            {
                run_mode = MODE_FAULT;
                mode_owner = MODE_OWNER_AUTO;

                flag_LED2_fail = TRUE;
                LED2_failcount = Max_count_LED;

                current_ready = 0;
                peak_second = 0;
                // FAULT 진입 이후 latch 유지/해제는 의미가 적지만, 정리 차원에서 리셋
                fault_latch_2 = 0;
                fault_ref_2 = 0;
            }
        }
        else
        {
            LED2_failcount = 0;
            fault_latch_2 = 0;
            fault_ref_2 = 0;
            flag_LED2_fail = FALSE;
        }
    }

}


void Fail_setup(void)
{

	if(run_mode == MODE_FAULT)
	{ 

    HAL_GPIO_WritePin(Fault_Relay_Control_GPIO_Port, Fault_Relay_Control_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(First_LED_GPIO_Port,  First_LED_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Second_LED_GPIO_Port, Second_LED_Pin, GPIO_PIN_SET);

	  
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);

    if(set_pinup.R_active != HIGH)
    {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    }
    
	}
  else return;
 
 
}

/////////////////////////////////////////////////////////////////////////
//PWM 함수
//////////////////////////////////////////////////////////////////////////
void update_system_PWM(void)
{
    uint32_t final_pwm;
     // 이전 온도 상태를 기억하기 위한 정적 변수
    static bool is_temp_high = FALSE;

    // CDS 상태 판단
    if (flag_override_pwm == TRUE) return; 
    
    set_PWM_by_CDSvalue();


    if (current_state == CDS_NIGHT )
    {
      final_pwm = PWM_fourty_duty_cycle;  //40% 듀티

      if (flag_Mainvolt_detected == TRUE)
      {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 12);
      }
      else HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    
    } 
    else
    {
      final_pwm = PWM_seventy_duty_cycle;
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    }


    if (measured_TEMP_Val >= 70)
    {   
        // 70도 이상인 순간에만 플래그 발생
        if (is_temp_high == FALSE)
        {
            flag_change_Temp = TRUE;
            is_temp_high = TRUE;
        }
        final_pwm = PWM_half_duty_cycle; // 50% 듀티로 하강

    }
    else 
    {
        // 다시 70도 이하로 복구되는 순간에도 플래그 발생
        if (is_temp_high == TRUE)
        {
            flag_change_Temp = TRUE;
            is_temp_high = FALSE;
        }
    }
    

    // CDS 변경 직후 안정화 기간 동안은 즉시 출력
    if (flag_change_CDS_Val == TRUE) 
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, final_pwm);
        return; 
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, final_pwm); // 출력
}

void set_PWM_by_CDSvalue(void)
{
  if(flag_check_input == FALSE) return;
  flag_check_input = FALSE;

  static uint8_t stable_count = 0; 
  uint8_t new_candidate = (measured_CDS_Val > 650) ? CDS_DAY : CDS_NIGHT;
  

  // 1. 현재 확정된 상태와  다를 때만 로직 가동
  if (new_candidate != current_state) 
  {
    // 2. 바뀌었는지 체크 (채터링 방지)
    if (new_candidate != candidate_state)
    {
      candidate_state = new_candidate;
      stable_count = 0; 
      test4 =0;
    }
    else 
    {
      stable_count++; 
      test4++;
      if (stable_count >= 5) // 5번 유지 시(5초)
      {
        current_state = candidate_state; // 상태 확정
        flag_change_CDS_Val = TRUE;      // 플래그 발생 (딱 한 번만)
       
        stable_count = 0;                // 카운트 초기화
      }

    }
  }

  else 
  {
    // 입력값이 현재 상태와 같으면 카운트와 후보군 초기화 (아무것도 안 함)
    flag_change_CDS_Val = FALSE;
    candidate_state = current_state;
    stable_count = 0;
  }

}

void check_System_Stability(void)
{
    uint32_t now = HAL_GetTick();
    static bool last_volt_state = true; // 이전 전압 상태 저장용
    static uint16_t prev_DC_Val = 0;
    uint16_t dc = measured_DC_Val;

    // 1. 전압이 복구되는 시점 감지 (FALSE -> TRUE)
    if (flag_Mainvolt_detected == TRUE && last_volt_state == FALSE)
    {
        flag_system_stabilizing = TRUE;
        stability_start_ms = now;
      
    }
    
    last_volt_state = flag_Mainvolt_detected;

    // 2. 기존 CDS/온도 변경 시 안정화 로직
    if ((flag_change_CDS_Val == TRUE || flag_change_Temp == TRUE) && flag_system_stabilizing == FALSE)
    {
        flag_system_stabilizing = TRUE;
        peak_first = 0; 
        peak_second = 0;
        stability_start_ms = now;
    }

    if ( (dc > prev_DC_Val && (dc - prev_DC_Val) >= 3) || (prev_DC_Val > dc && (prev_DC_Val - dc) >= 3) )
    {
        prev_DC_Val = measured_DC_Val;
        peak_first = 0; 
        peak_second = 0;
        flag_system_stabilizing = TRUE;
        stability_start_ms = now;
    }
    // 3. 5초 유예 관리
    if (flag_system_stabilizing == TRUE)
    {
        if ((uint32_t)(now - stability_start_ms) >= 5000)
        {
            flag_system_stabilizing = FALSE;
            flag_change_CDS_Val = FALSE;
            flag_change_Temp = FALSE;
        }
    }
 
  
}


/////////////////////////////////////////////////////////////////////////
//통신 함수
//////////////////////////////////////////////////////////////////////////
/* CRC16-CCITT 계산 함수
   Polynomial: 0x1021
   Initial value: 0xFFFF
   Final XOR: 0x0000
*/
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;  // 초기값
  uint16_t i, j;

  for (i = 0; i < len; i++)
  {
    crc ^= ((uint16_t)data[i] << 8);
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
      crc &= 0xFFFF;  // 16비트 유지
    }
  }

  return crc;
}

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
  uint8_t sw_code = 0;

	if(flag_transmit == FALSE) return;
  
	flag_transmit = FALSE;

  if(set_pinup.signalling_active == HIGH)            card_setup = 0x01;
  else if(set_pinup.shunting_active == HIGH)         card_setup = 0x02;
  else if(set_pinup.unguide_active == HIGH)          card_setup = 0x03;

  if(set_pinup.R_active == LOW && set_pinup.G_active == HIGH)             LED_setup = 0x01;
  else if(set_pinup.G_active == LOW && set_pinup.R_active == HIGH)        LED_setup = 0x02;
  else if(set_pinup.y_on == HIGH)                                         LED_setup = 0x03;
  else if(set_pinup.y1_on == HIGH)                                        LED_setup = 0x04;

  if(current_state == CDS_DAY)        day_night_status = 0x01;
  else if(current_state == CDS_NIGHT)  day_night_status = 0x02;


  if ( g_manual_switchover_event == FALSE || mode_owner == MODE_OWNER_AUTO)
  {
      sw_code = (0x01 % 10) + '0';              
  }
  if ( g_manual_switchover_event == TRUE && mode_owner == MODE_OWNER_SWITCH)
  {
      sw_code = (0x02 % 10) + '0';              //수동취급    
  } 


  transmit_buf[0] = 0x02;

  uint8_t frame_len = (uint8_t)(sizeof(transmit_buf) / sizeof(transmit_buf[0]));
  transmit_buf[1] = frame_len;

  static uint16_t tx_sequence = 0;
  tx_sequence++;

  transmit_buf[2] = (uint8_t)((tx_sequence >> 8) & 0xFF);
  transmit_buf[3] = (uint8_t)(tx_sequence & 0xFF);
	transmit_buf[4] = OP_CODE;

  
  for(i=0; i<3; i++)
  {
    transmit_buf[5+i] = (measured_DC_Val / POW(10, 2-i)) % 10 + '0';
  }
  for(i=0; i<3; i++)
  {
    transmit_buf[8+i] = (measured_PWM_Cur/ POW(10, 2-i)) % 10 + '0';
  }

  int16_t tmp = measured_TEMP_Val;
  if (tmp < 0)
  {
    int16_t absval = (int16_t)(-tmp);
    if (absval > 99) absval = 99; 
    transmit_buf[11] = (0x01%10) + '0'; // '-'
    transmit_buf[12] = (absval / 10) % 10 + '0';
    transmit_buf[13] = (absval % 10) + '0';
  }
  else
  {
    int16_t val = tmp;
    if (val > 999) val = 999; 
    transmit_buf[11] = (val / 100) % 10 + '0';
    transmit_buf[12] = (val / 10) % 10 + '0';
    transmit_buf[13] = (val % 10) + '0';
  }
  



  if (run_mode == MODE_FIRST_ACTIVE)                                  LDE_card_status = 0x01;
  else if (run_mode == MODE_SECOND_ACTIVE && flag_LED1_fail == FALSE) LDE_card_status = 0x02; 
  else if (run_mode == MODE_SECOND_ACTIVE && flag_LED1_fail == TRUE)  LDE_card_status = 0x03; //1계 고장 → 2계 자동 절체 
  else if (run_mode == MODE_FAULT)                                    LDE_card_status = 0x04; // 완전 고장 
 

  transmit_buf[14] = (LDE_card_status % 10) + '0';

  transmit_buf[15] = (card_setup)%10 + '0';
  transmit_buf[16] = (LED_setup) %10 + '0';
  transmit_buf[17] = (day_night_status)%10 +'0';
  transmit_buf[18] = sw_code;

  if(flag_Mainvolt_detected == TRUE)  transmit_buf[19] = (0x02 % 10) + '0';
  else                                transmit_buf[19] = (0x01 % 10) + '0';

  // CRC16-CCITT 계산 (bytes [0..13] 대상)
  uint16_t crc_val = crc16_ccitt(transmit_buf, 20);
  transmit_buf[20] = (uint8_t)((crc_val >> 8) & 0xFF);  // CRC MSB
  transmit_buf[21] = (uint8_t)(crc_val & 0xFF);         // CRC LSB
  transmit_buf[22] = END_CODE;

	HAL_UART_Transmit_IT(&huart1, transmit_buf, 23);
	count_TX_timeout = 0;

}


void OnUart_Recv (uint8_t ch)
{
	static uint8_t i=0;
	static uint8_t buf[8];

  if(i>8) i=0;
	
	buf[i] = ch;

	if(buf[i++] == 0x03)
	{
		memcpy(receive_buf,buf,i);
		memset(buf,0x00,8);	
		i=0;
	}

}



void data_receive_timeoutControl(void)
{
  if(count_RX_timeout >= 3000)memset(&receive_buf, 0x00, 8);
  if(count_TX_timeout >= 3000) memset(&transmit_buf, 0x00, 17);

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
	LL_GPIO_WriteOutputPort(GPIOD,(ReadValue_portD & 0xff00));	//FND9~FND15 clear
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
	
}


void make_fnd_data (void)
{

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

    update_system_PWM();
    check_System_Stability();
    check_Fault();
    
    control_logic_step();

    Fail_setup();

    transmit();



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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 125;
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
  htim5.Init.Period = 25-1;
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
  huart1.Init.BaudRate = 19200;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, First_LED_Pin|Second_LED_Pin|Fault_Relay_Control_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_CONTROL_PWR_GPIO_Port, MCU_CONTROL_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FAULT_LED_Pin|RUN_LED_Pin|CUR_CON2_Pin|CUR_CON1_Pin
                          |MCU_to_WATCHDOG_INPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin
                          |FND5_Pin|FND6_Pin|FND7_Pin|FND8_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : FAULT_LED_Pin RUN_LED_Pin */
  GPIO_InitStruct.Pin = FAULT_LED_Pin|RUN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FND1_Pin FND2_Pin FND3_Pin FND4_Pin
                           FND5_Pin FND6_Pin FND7_Pin FND8_Pin */
  GPIO_InitStruct.Pin = FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin
                          |FND5_Pin|FND6_Pin|FND7_Pin|FND8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CUR_CON2_Pin CUR_CON1_Pin MCU_to_WATCHDOG_INPUT_Pin */
  GPIO_InitStruct.Pin = CUR_CON2_Pin|CUR_CON1_Pin|MCU_to_WATCHDOG_INPUT_Pin;
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
