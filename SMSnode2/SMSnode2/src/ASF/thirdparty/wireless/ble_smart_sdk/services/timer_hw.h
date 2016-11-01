#ifndef __TIMER_HW_H__
#define __TIMER_HW_H__

#include "dualtimer.h"

#define PWR_WAKEUP_DOMAIN_ARM   (1)
#define DUALTIMER_CALLBACK             0x41
#define AON_SLEEP_TIMER_EXPIRY_CALLBACK 0x42

/** Enum for the possible callback types for the timer module. */
enum timer_callback_type {
	/** Callback for timer expiry*/
	TIMER_EXPIRED_CALLBACK_TYPE_DETECT = 1,
    COUNTER_OVERFLOWED_CALLBACK_CONTINUE,
    AON_TIMER_EXPIRED
};

typedef enum timer_unit_type {
    TIMER_UNIT_US,
    TIMER_UNIT_MS,
    TIMER_UNIT_S
}timer_unit_type_t;


typedef void (*hw_timer_callback_t)(void);

void hw_timer_init(void);
void hw_timer_register_callback(enum dualtimer_timer tmr, hw_timer_callback_t cb_ptr);
void hw_timer_start(timer_unit_type_t unit, uint32_t delay, enum dualtimer_timer tmr);
void hw_timer_stop(enum dualtimer_timer tmr);

void dualtimer_callback2(void);


#endif /* __TIMER_HW_H__ */
