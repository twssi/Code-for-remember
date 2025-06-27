#ifndef INCLUDED__MCU_GPIO_ALIAS__H
#define INCLUDED__MCU_GPIO_ALIAS__H



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#ifndef MCU_SIMULATOR
#define GPIO_ALIAS_API static inline
#else
#define GPIO_ALIAS_API static __inline
#endif


//===========================================================================
//External WATCHDOG
GPIO_ALIAS_API void      	GPIO_O_EXTERNAL_WATCHDOG_CLOCK 	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOB, GPIO_Pin_9 , (BitAction)v); }
GPIO_ALIAS_API cx_uint_t	GPIO_I_EXTERNAL_WATCHDOG_INPUT	(void)       	{ return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0 ); }

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
//RUN/FAIL LED
GPIO_ALIAS_API void      	GPIO_O_CPU_LED_STATUS       	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOC, GPIO_Pin_1 , (BitAction)v); } 

//===========================================================================
//DI/DO 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI0     	(void)        { return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8 ); }	 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI1     	(void)        { return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9 ); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI2     	(void)        { return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8 ); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI3     	(void)        { return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7 ); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI4     	(void)        { return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI5     	(void)        { return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI6     	(void)        { return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13); } 
GPIO_ALIAS_API cx_uint_t GPIO_I_COWORKER_DI7     	(void)        { return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12); } 

GPIO_ALIAS_API void      GPIO_O_COWORKER_DO0     	(cx_uint_t v) { GPIO_WriteBit(GPIOD, GPIO_Pin_11 , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO1     	(cx_uint_t v) { GPIO_WriteBit(GPIOD, GPIO_Pin_10 , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO2     	(cx_uint_t v) { GPIO_WriteBit(GPIOD, GPIO_Pin_9  , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO3     	(cx_uint_t v) { GPIO_WriteBit(GPIOD, GPIO_Pin_8  , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO4     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_15 , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO5     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_14 , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_COWORKER_DO6     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_13 , (BitAction)v);  } 
//GPIO_ALIAS_API void      GPIO_O_COWORKER_DO7     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_12 , (BitAction)v);  } 

//===========================================================================
GPIO_ALIAS_API cx_uint_t GPIO_I_LOCATION_INPUT 		(void)		{ return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5  ); }

//===========================================================================
GPIO_ALIAS_API void      GPIO_O_IMPULSE_CLR1 		(cx_uint_t v) 	{ GPIO_WriteBit(GPIOA, GPIO_Pin_6 , (BitAction)v); }
GPIO_ALIAS_API void      GPIO_O_IMPULSE_CLR2 		(cx_uint_t v) 	{ GPIO_WriteBit(GPIOA, GPIO_Pin_7 , (BitAction)v); }

//GPIO_ALIAS_API void      GPIO_O_MCU_SMPS_Control 	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOB, GPIO_Pin_1 , (BitAction)v); }
GPIO_ALIAS_API void      GPIO_O_SwitchOver_Control 	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)v); }
GPIO_ALIAS_API void      GPIO_O_MCU_TM_State     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_0 , (BitAction)v);  } 
GPIO_ALIAS_API void      GPIO_O_MCU_TM_Data     	(cx_uint_t v) { GPIO_WriteBit(GPIOB, GPIO_Pin_1 , (BitAction)v);  } 

GPIO_ALIAS_API cx_uint_t GPIO_I_Manual_Switch_INPUT (void)		{ return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3  ); }
GPIO_ALIAS_API cx_uint_t GPIO_TEST_MODE          	(void)      { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12 ); }	 
//===========================================================================

#endif
