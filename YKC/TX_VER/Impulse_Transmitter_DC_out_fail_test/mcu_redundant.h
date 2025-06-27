#ifndef INCLUDED__MCU_REDUNDANT__H
#define INCLUDED__MCU_REDUNDANT__H





/////////////////////////////////////////////////////////////////////////////
//===========================================================================
#define WORKER_STATE_UNKNOWN      	0u
#define WORKER_STATE_BOOT         	1u
#define WORKER_STATE_ACTIVE_SWITCH  5u
#define WORKER_STATE_ACTIVE_NONE  	4u
#define WORKER_STATE_STANDBY_SWITCH 3u
#define WORKER_STATE_STANDBY_NONE 	2u
#define WORKER_STATE_HALT         	6u


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_uint_t get_worker_number   (void);
MCU_API cx_uint_t get_coworker_number (void);

MCU_API cx_bool_t check_worker_active (cx_uint_t state);
MCU_API cx_bool_t check_worker_health (cx_uint_t state);
MCU_API cx_bool_t check_worker_halt   (cx_uint_t state);
MCU_API cx_bool_t check_worker_switch (cx_uint_t state);

//===========================================================================
MCU_API cx_uint_t get_worker_state    (void);
MCU_API cx_bool_t get_worker_active   (void);
MCU_API cx_bool_t get_worker_health   (void);
MCU_API cx_bool_t get_worker_halt     (void);
MCU_API cx_bool_t get_worker_switch   (void);

MCU_API cx_uint_t get_coworker_state  (void);
MCU_API cx_bool_t get_coworker_active (void);
MCU_API cx_bool_t get_coworker_health (void);
MCU_API cx_bool_t get_coworker_halt   (void);
MCU_API cx_bool_t get_coworker_switch (void);

MCU_API cx_bool_t get_fault_cause     (void);

//===========================================================================
MCU_API cx_uint_t get_worker1_state  (void);
MCU_API cx_bool_t get_worker1_active (void);
MCU_API cx_bool_t get_worker1_health (void);
MCU_API cx_bool_t get_worker1_halt   (void);
MCU_API cx_bool_t get_worker1_switch (void);

MCU_API cx_uint_t get_worker2_state  (void);
MCU_API cx_bool_t get_worker2_active (void);
MCU_API cx_bool_t get_worker2_health (void);
MCU_API cx_bool_t get_worker2_halt   (void);
MCU_API cx_bool_t get_worker2_switch (void);

//===========================================================================
MCU_API void set_worker_fault 		(void);
MCU_API void set_worker_non_switch  (void);
MCU_API void set_worker_switch   	(void);
MCU_API void set_worker_run 		(void);

MCU_API cx_bool_t pair_dido_heartbeat_fault_update (void);

/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API void hotstandby_irq_handler (void);

//===========================================================================
MCU_API cx_bool_t hotstandby_initialize (void);
MCU_API void      hotstandby_update     (void);


/////////////////////////////////////////////////////////////////////////////
//===========================================================================
MCU_API cx_byte_t redundant_get_status (void);
MCU_API void      redundant_show       (void);


#endif




