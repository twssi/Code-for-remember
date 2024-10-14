#ifndef INCLUDED__MCU_GPIO_ALIAS__H
#define INCLUDED__MCU_GPIO_ALIAS__H



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#ifndef MCU_SIMULATOR
#define GPIO_ALIAS_API static inline
#else
#define GPIO_ALIAS_API static __inline
#endif



/////////////////////////////////////////////////////////////////////////////
//===========================================================================
//STATUS LED
GPIO_ALIAS_API void      GPIO_O_CPU_LED_STATUS       	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOC, GPIO_Pin_6 , (BitAction)v); } 

//===========================================================================
//External WATCHDOG
GPIO_ALIAS_API void      	GPIO_O_EXTERNAL_WATCHDOG_CLOCK 	(cx_uint_t v) 	{ GPIO_WriteBit(GPIOB, GPIO_Pin_9 , (BitAction)v); }
GPIO_ALIAS_API cx_uint_t	GPIO_I_EXTERNAL_WATCHDOG_INPUT	(void)       	{ return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0 ); }

//===========================================================================
GPIO_ALIAS_API void      	GPIO_O_IMPULSE_CLR1 		(cx_uint_t v) 	{ GPIO_WriteBit(GPIOC, GPIO_Pin_8 , (BitAction)v); }
GPIO_ALIAS_API void      	GPIO_O_IMPULSE_CLR2 		(cx_uint_t v) 	{ GPIO_WriteBit(GPIOC, GPIO_Pin_9 , (BitAction)v); }

GPIO_ALIAS_API cx_uint_t GPIO_I_MCU_TRANS_1 		(void)		{ return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12); }
GPIO_ALIAS_API cx_uint_t GPIO_I_MCU_TRANS_2 		(void)		{ return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13); }
GPIO_ALIAS_API cx_uint_t GPIO_I_MCU_TRANS_3 		(void)		{ return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14); }
GPIO_ALIAS_API cx_uint_t GPIO_I_MCU_TRANS_4 		(void)		{ return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15); }

//===========================================================================
GPIO_ALIAS_API cx_uint_t GPIO_I_Track_Relay			(void)		{ return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1 ); }
/*
GPIO_ALIAS_API cx_uint_t GPIO_I_SwitchOver_INPUT_1  (void)		{ return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4  ); }
GPIO_ALIAS_API cx_uint_t GPIO_I_SwitchOver_INPUT_2  (void)		{ return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5  ); }
*/
//===========================================================================


#endif
