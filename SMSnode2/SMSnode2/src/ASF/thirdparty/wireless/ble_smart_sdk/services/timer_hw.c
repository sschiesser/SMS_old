#include <asf.h>
#include "console_serial.h"
#include "timer_hw.h"
#include "conf_timer.h"
#include "ble_utils.h"

extern struct uart_module uart_instance;


void hw_timer_init(void)
{
	struct dualtimer_config config_dualtimer;
	dualtimer_get_config_defaults(&config_dualtimer);

	config_dualtimer.timer1.load_value = CONF_DUALTIMER_TIMER1_LOAD;
	config_dualtimer.timer2.load_value = CONF_DUALTIMER_TIMER2_LOAD;

	dualtimer_init(&config_dualtimer);
    dualtimer_disable(DUALTIMER_TIMER1);
    dualtimer_disable(DUALTIMER_TIMER2);
}

void hw_timer_register_callback(enum dualtimer_timer tmr, hw_timer_callback_t cb_handler)
{
	dualtimer_register_callback(tmr, cb_handler);

	dualtimer_disable(tmr);

	NVIC_EnableIRQ(DUALTIMER0_IRQn);
}

void hw_timer_start(timer_unit_type_t unit, uint32_t delay, enum dualtimer_timer tmr)
{
    uint32_t timer_load = 0;
    switch(unit) {
        case TIMER_UNIT_US:
        timer_load = CONF_DUALTIMER_TIMER_LOAD_US;
        break;
        
        case TIMER_UNIT_MS:
        timer_load = CONF_DUALTIMER_TIMER_LOAD_MS;
        break;
        
        case TIMER_UNIT_S:
        default:
        timer_load = CONF_DUALTIMER_TIMER_LOAD_S;
        break;
    }     
       
	if(delay <= 0) {
        //DBG_LOG("[hw_timer_start]  Warning! Delay value < 0... setting to 1000");
		delay = 1000;
	}
    
	dualtimer_set_counter (tmr, DUALTIMER_SET_CURRUNT_REG, timer_load * delay);
	dualtimer_enable(tmr);
}

void hw_timer_stop(enum dualtimer_timer tmr)
{
    dualtimer_disable(tmr);
}
