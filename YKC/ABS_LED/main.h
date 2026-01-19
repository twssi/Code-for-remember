/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

enum {  CDS_DAY = 1, CDS_NIGHT = 2 };

typedef enum {
  MODE_OWNER_SWITCH = 0,   // 스위치가 세운 모드
  MODE_OWNER_AUTO   = 1,   // 고장으로 인한 자동절체가 세운 모드
} ModeOwner_t;



typedef struct {
    uint8_t run_mode_first  : 1; // 0번 비트: MODE_FIRST_ACTIVE 여부
    uint8_t run_mode_second : 1; // 1번 비트: MODE_SECOND_ACTIVE 여부
    uint8_t led1_fail       : 1; // 2번 비트: LED1 실패 여부
    uint8_t led2_fail       : 1; // 3번 비트: LED2 실패 여부
    uint8_t reserved        : 4; // 나머지 4비트 (예비용)
} SystemStatus_t;


typedef struct{
	uint8_t Day_pin_set;
	uint8_t Night_pin_set;
	uint8_t Group_pin_set;
	uint8_t G_Pin_set;
	uint8_t R_Pin_set;
	uint8_t Shunting_pin_set;
	uint8_t Signalling_pin_set;
	uint8_t Unguide_pin_set;
    
}Switchstate_t;

typedef enum {
    CARD_NONE = 0,
    CARD_GREEN,
    CARD_RED,
    CARD_YELLOW,
    CARD_SHUNTING,
    CARD_GROUP,
    CARD_UNGUIDE,
    CARD_SIGNALLING,
    // 필요하면 계속 추가
} ActiveCard_t;

typedef struct {

    uint8_t shunting_active;     
    uint8_t signalling_active;   
    uint8_t unguide_active;     

    uint8_t day_active;          // 주간 설정  추후 삭제 될 예정임 설정 핀 대신 CDS로 대체
    uint8_t night_active;        // 야간설정

    uint8_t y_on;                // G+R 동시 HIGH 시 Y 상태 (1이면 Y 로직 동작)
    uint8_t y1_on;				// G+R 동시 LOW 시 Y1
    uint8_t G_active;        // G 상태
    uint8_t R_active;        // R 상태

    uint8_t Group_active;    // Group 모드 상태
    uint8_t Insert_active;
   

} OutputResult_t;

typedef enum {
    MODE_FIRST_ACTIVE = 0,   // 1계 ON
    MODE_SECOND_ACTIVE,      // 2계 ON
    MODE_SWITCH_OFF,         // 사용자 스위치 둘 다 OFF(스위치 망가짐?)
    MODE_FAULT               // 1,2계 고장 -> 고장 상태
} RunMode_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void app_setup(void);

void watchdog_run_irq_handler(void);
void mv_qualifier_tick(void);



void control_logic_step(void);
void check_Fault(void);

void PWM_SetDutyBy_Switch(void);
RunMode_t read_setup_switch(void);



void set_pwm_bysoftware(void);
void set_PWM_by_CDSvalue(void);
void update_system_PWM(void);
void timer_counter_run(void);
void read_mainvolt(void);

uint16_t POW(uint16_t a, uint16_t b);
void OnUart_Recv (uint8_t ch);

void transmit(void);
void software_PWM_control(void);
void output_fnd_display(void);
void fnd_output_data(const unsigned int id, const uint8_t* bufptr, unsigned int bufsize);
void fnd_write(unsigned int select, uint8_t fndvalue);
void display_maked_fnd_data (const char* numstring, uint8_t* bufptr, unsigned int bufsize);
void make_fnd_data (void);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define First_LED_Pin GPIO_PIN_0
#define First_LED_GPIO_Port GPIOC
#define Second_LED_Pin GPIO_PIN_1
#define Second_LED_GPIO_Port GPIOC
#define Fault_Relay_Control_Pin GPIO_PIN_2
#define Fault_Relay_Control_GPIO_Port GPIOC
#define MCU_CONTROL_PWR_Pin GPIO_PIN_3
#define MCU_CONTROL_PWR_GPIO_Port GPIOC
#define CUR_measure_Pin GPIO_PIN_0
#define CUR_measure_GPIO_Port GPIOA
#define First_sw_in_Pin GPIO_PIN_4
#define First_sw_in_GPIO_Port GPIOA
#define Second_sw_in_Pin GPIO_PIN_5
#define Second_sw_in_GPIO_Port GPIOA
#define LED1_INPUT_Pin GPIO_PIN_6
#define LED1_INPUT_GPIO_Port GPIOA
#define LED2_INPUT_Pin GPIO_PIN_7
#define LED2_INPUT_GPIO_Port GPIOA
#define FAULT_LED_Pin GPIO_PIN_0
#define FAULT_LED_GPIO_Port GPIOB
#define RUN_LED_Pin GPIO_PIN_1
#define RUN_LED_GPIO_Port GPIOB
#define FND1_Pin GPIO_PIN_8
#define FND1_GPIO_Port GPIOE
#define FND2_Pin GPIO_PIN_9
#define FND2_GPIO_Port GPIOE
#define FND3_Pin GPIO_PIN_10
#define FND3_GPIO_Port GPIOE
#define FND4_Pin GPIO_PIN_11
#define FND4_GPIO_Port GPIOE
#define FND5_Pin GPIO_PIN_12
#define FND5_GPIO_Port GPIOE
#define FND6_Pin GPIO_PIN_13
#define FND6_GPIO_Port GPIOE
#define FND7_Pin GPIO_PIN_14
#define FND7_GPIO_Port GPIOE
#define FND8_Pin GPIO_PIN_15
#define FND8_GPIO_Port GPIOE
#define CUR_CON2_Pin GPIO_PIN_13
#define CUR_CON2_GPIO_Port GPIOB
#define CUR_CON1_Pin GPIO_PIN_14
#define CUR_CON1_GPIO_Port GPIOB
#define Day_Pin GPIO_PIN_8
#define Day_GPIO_Port GPIOD
#define Night_Pin GPIO_PIN_9
#define Night_GPIO_Port GPIOD
#define Group_Pin GPIO_PIN_10
#define Group_GPIO_Port GPIOD
#define Signalling_Block_Pin GPIO_PIN_11
#define Signalling_Block_GPIO_Port GPIOD
#define Shunting_Pin GPIO_PIN_12
#define Shunting_GPIO_Port GPIOD
#define Unguide_Pin GPIO_PIN_13
#define Unguide_GPIO_Port GPIOD
#define R_SET_Pin GPIO_PIN_14
#define R_SET_GPIO_Port GPIOD
#define G_SET_Pin GPIO_PIN_15
#define G_SET_GPIO_Port GPIOD
#define CUR_CON0_Pin GPIO_PIN_7
#define CUR_CON0_GPIO_Port GPIOC
#define PWM_CONTROL_Pin GPIO_PIN_8
#define PWM_CONTROL_GPIO_Port GPIOC
#define Main_Volt_detecto_Pin GPIO_PIN_8
#define Main_Volt_detecto_GPIO_Port GPIOA
#define FNDA_Pin GPIO_PIN_0
#define FNDA_GPIO_Port GPIOD
#define FNDB_Pin GPIO_PIN_1
#define FNDB_GPIO_Port GPIOD
#define FNDC_Pin GPIO_PIN_2
#define FNDC_GPIO_Port GPIOD
#define MCU_to_WATCHDOG_INPUT_Pin GPIO_PIN_9
#define MCU_to_WATCHDOG_INPUT_GPIO_Port GPIOB
#define WATCHDOG_to_MCU_INPUT_Pin GPIO_PIN_0
#define WATCHDOG_to_MCU_INPUT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define LOW 0u
#define HIGH 1u


#define TRUE 1u
#define FALSE 0u

#define OP_CODE  0x30
#define END_CODE 0x03
#define TOTAL_SAMPLE_LIMIT   2000u

// ADC DMA 버퍼 인덱스
#define ADC_CH_C_IDX 0 // ADC_CHANNEL_0 (기존 전압 채널2)
#define ADC_CH_TEMP_IDX 1 // ADC_CHANNEL_2
#define ADC_CH_CDS_IDX 2 // ADC_CHANNEL_1
#define ADC_CH_DC_IDX 3 // ADC_CHANNEL_3

#define HOLD_MS_AFTER_FAILOVER  5000

#define NUM_PWM_CURRENT_CHANNELS      1
#define NUM_DC_VOLATAGE_CHANNELS      1
#define NUM_CDS_CHANNELS              1
#define NUM_TEMP_CHANNELS             1
#define NUM_ADC_CHANNELS (NUM_PWM_CURRENT_CHANNELS + NUM_DC_VOLATAGE_CHANNELS + NUM_CDS_CHANNELS + NUM_TEMP_CHANNELS)



#define PWM_full_duty_cycle     250u  // 100% duty for 250 max
#define PWM_ninety_duty_cycle   225u
#define PWM_eighty_duty_cycle   200u
#define PWM_seventy_duty_cycle  175u  // 70% duty for 250 max
#define PWM_sixty_duty_cycle    150u  // 60% duty for 250 max
#define PWM_half_duty_cycle     125u  // 50% duty for 250 max
#define PWM_fourty_duty_cycle   100u  // 40% duty for 250 max
#define PWM_thirty_duty_cycle  	75u
#define PWM_twenty_duty_cycle  	50u
#define PWM_ten_duty_cycle   	  25u
#define PWM_zero_duty_cycle     0u    // 0% duty for 250 max


#define MV_ASSERT_MS          50u  
#define MV_RELEASE_MS         950u   // HIGH로 '해제'할 최소 지속시간
#define IGNORE_HIGH_MS        8u    // 190ms보다 살짝 크게
#define IGNORE_HIGH_toLOW_MS  3u



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
