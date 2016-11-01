/**
* \file
*
* \brief BLE Startup Template
*
* Copyright (c) 2016 Atmel Corporation. All rights reserved.
*
* \asf_license_start
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \asf_license_stop
*
*/

/*
* Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
* Support</a>
*/

/**
* \mainpage
* \section preface Preface
* This is the reference manual for the Startup Template
*/
/*- Includes ---------------------------------------------------------------*/
#include "app_include.h"

#ifndef SABRE_DEVICE
#error "SABRE_DEVICE not defined, cannot assign a role"
#else
#if (SABRE_DEVICE == NODE_FULL)
sms_device_type_t sms_device = SMS_DEVICE_NODE_FULL;
#elif (SABRE_DEVICE == NODE_REDUCED)
sms_device_type_t sms_device = SMS_DEVICE_NODE_REDUCED;
#elif (SABRE_DEVICE == GATEWAY)
sms_device_type_t sms_device = SMS_DEVICE_GATEWAY;
#else
#error "Wrong assignation of SABRE_DEVICE, cannot assign a role"
#endif
#endif


//uint8_t db_mem[1024] = {0};
gatt_service_handler_t bas_service_handler;


uint8_t conn_status = 0;
uint8_t button_value = 0;


//uint8_t wakeup_gpio_pin = PIN_AO_GPIO_0;

/* ----------------------
* BLE callback functions
* ---------------------- */

/* Callback registered for AT_BLE_ADV_REPORT (#3) event from stack */
static at_ble_status_t ble_adv_report_event(void *params)
{
    DBG_LOG_DEV("[ble_adv_report_event]  advertisement timeout");
    DBG_LOG_DEV("Going to sleep...");
    release_sleep_lock();

    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CONNECTED (#5) event from stack */
static at_ble_status_t ble_connected_app_event(void *params)
{
    DBG_LOG_DEV("[ble_connected_app_event] devices connected");
    at_ble_connected_t *connected = (at_ble_connected_t *)params;
    sms_connection_handle = connected->handle;
    conn_status = 1;
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCONNECTED (#6) event from stack */
static at_ble_status_t ble_disconnected_app_event(void *param)
{
    conn_status = 0;
    DBG_LOG_DEV("[ble_disconnected_app_event] peer disconnected...");
    switch(sms_working_mode) {
        case SMS_MODE_BUTTON_SOLO:
        break;
        
        case SMS_MODE_IMU_SOLO:
        // disable AO GPIO interrupt
        break;
        
        case SMS_MODE_PRESSURE_SOLO:
        // disable AON sleep timer
        break;
        
        case SMS_MODE_BUTTON_IMU:
        // disable AO GPIO interrupt
        break;
        
        case SMS_MODE_BUTTON_PRESSURE:
        // disable AON sleep timer
        break;
        
        case SMS_MODE_IMU_PRESSURE:
        // disable AO GPIO interrupt
        break;
        
        case SMS_MODE_COMPLETE:
        // disable AO GPIO interrupt
        break;
        
        default:
        break;
    }
    // enable ~30s counter for advertisement, then go to sleep
    sms_service_advertise();
    ALL_UNUSED(param);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_PAIR_DONE (#9) event from stack */
static at_ble_status_t ble_paired_app_event(void *param)
{
    DBG_LOG_DEV("[ble_paired_app_event] Devices paired...");
    DBG_LOG_CONT_DEV(" enabling sleep.");
    //release_sleep_lock();
    ALL_UNUSED(param);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_NOTIFICATION_CONFIRMED (#29) event from stack */
static at_ble_status_t ble_notification_confirmed_app_event(void *param)
{
    at_ble_cmd_complete_event_t *notification_status = (at_ble_cmd_complete_event_t *)param;
    if(!notification_status->status)
    {
        DBG_LOG_DEV("[ble_notification_confirmed_app_event] notification sent successfully");
        return AT_BLE_SUCCESS;
    }
    return AT_BLE_FAILURE;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_CHANGED (#31) event from stack */
static at_ble_status_t ble_char_changed_app_event(void *params)
{
    ALL_UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_PLATFORM_EVENT (#58) event */
static void sms_plf_event_cb(void)
{
    sms_plf_int_state.int_on = true;
}

/* --------------------------
* BLE callbacks enumerations
* -------------------------- */
/* Enumerate all app GAT callbacks */
static const ble_event_callback_t startup_template_app_gap_cb[] = {
    NULL,
    NULL,
    NULL,
    ble_adv_report_event,
    NULL,
    ble_connected_app_event,
    ble_disconnected_app_event,
    NULL,
    NULL,
    ble_paired_app_event,
    NULL,
    NULL,
    NULL,
    NULL,
    ble_paired_app_event,
    NULL,
    NULL,
    NULL,
    NULL
};

/* Enumerate all app GATT SERVER callbacks */
static const ble_event_callback_t startup_template_app_gatt_server_cb[] = {
    ble_notification_confirmed_app_event,
    NULL,
    ble_char_changed_app_event,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};


///* ------------------------
//* Timer callback functions
//* ------------------------ */
//static void hw_timer1_callback_fn(void)
//{
    //DBG_LOG_DEV("[hw_timer1_callback_fn]");
    //sms_plf_int_state.source = SMS_CB_DUALTIMER1;
    //send_plf_int_msg_ind(DUALTIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
//}
//
//static void hw_timer2_callback_fn(void)
//{
    //DBG_LOG_DEV("[hw_timer2_callback_fn]");
    //sms_plf_int_state.source = SMS_CB_DUALTIMER2;
    //send_plf_int_msg_ind(DUALTIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
//}

/* -------------------------
* Resume callback functions
* ------------------------- */
static void resume_cb(void)
{
    init_port_list(); // Re-initialize all ports
    serial_console_init(); // UART GPIO for the console
    sms_dualtimer_init(); // dualtimer
    if(sms_device == SMS_DEVICE_NODE_FULL) {
        ///* GPIO interrupt (AO_GPIO_2) for IMU data ready */
        //sms_imu_configure_gpio();
        spi_master_configure(); // SPI GPIO
        //spi_master_configure_callbacks();
        i2c_master_configure(); // I2C GPIO
        //i2c_master_configure_callbacks();
    }
    
    //hw_timer_register_callback(hw_timer1_callback_fn);
    //sms_button_configure_gpio(); // GPIO for the AO button interrupts
    //sms_button_register_callbacks();
}


/**@brief Initialize the service with its included service, characteristics, and descriptors
*/

int main(void)
{
    at_ble_status_t status;
    app_state = SMS_APP_STARTING;
    sms_plf_int_state.source = SMS_CB_NONE;
    sms_plf_int_state.int_on = false;
    mpu9250_device.init_ok = false;
    mpu9250_device.comm_error = false;


    /* Initialize platform */
    platform_driver_init();
    /* Initialize gpio */
    gpio_init();
    /* Prevent sleep mode */
    acquire_sleep_lock();
    /* Initialize serial console */
    serial_console_init();
    /* Welcome... */
    //DBG_LOG_DEV("\n\n\n\r"\
    //"********************************\n\r"\
    //"********************************\n\r"\
    //"**   SABRe SMS application    **\n\r"\
    //"**----------------------------**");
    //switch(sms_device) {
    //case SMS_DEVICE_GATEWAY_1:
    //DBG_LOG_DEV("** - SMS_DEVICE_GATEWAY_1     **");
    //break;
    //
    //case SMS_DEVICE_NODE_1:
    //DBG_LOG_DEV("** - SMS_DEVICE_NODE_1        **");
    //break;
    //
    //case SMS_DEVICE_NODE_2:
    //DBG_LOG_DEV("** - SMS_DEVICE_NODE_2        **");
    //break;
    //
    //default:
    //break;
    //}
    //DBG_LOG_DEV("********************************\n\r"\
    //"********************************\n\r");
    /* Initialize the BLE chip and Set the Device Address */
    DBG_LOG("[main] initializing BLE application");
    ble_device_init(NULL);

    /* Hardware timer for BLE use */
    sms_dualtimer_init(); // initialize dualtimer with both ms counter


    delay_init();
    
    /* SMS button on AO_GPIO */
    sms_button_configure_gpio(); // configure 2 AO GPIO as input with wake-up enabled
    sms_button_register_callbacks(); // set GPIO interrupt callbacks

    if(sms_device == SMS_DEVICE_NODE_FULL) {
        /* GPIO interrupt (AO_GPIO_2) for IMU data ready */
        sms_imu_configure_gpio();
        /* SPI config --> MS58 pressure sensor */
        spi_master_configure();
        spi_master_configure_callbacks();
        /* I2C config --> MPU-9250 IMU */
        i2c_master_configure();
        i2c_master_configure_callbacks();
    }
    
    /* Register sleep resume callback */
    register_resume_callback(resume_cb);
    
    /* Initialize & define primary BLE button service */
    sms_button_char_init_value = 0;
    sms_button_service_init(&sms_button_service_handler, &sms_button_char_init_value);
    if((status = sms_button_primary_service_define(&sms_button_service_handler)) != AT_BLE_SUCCESS) {
        //DBG_LOG("[main]  defining SMS button service failed, reason 0x%x", status);
    }
    else {
        //DBG_LOG_DEV("[main] SMS primary service defined, sms button handle %d", sms_button_service_handler.serv_handle);
    }

    /* Stuff for full-featured SMS device */
    if(sms_device == SMS_DEVICE_NODE_FULL) {
        for(uint8_t i = 0; i < 12; i++) {
            sms_imu_char_init_values[i] = 0;
        }
        sms_imu_service_init(&sms_imu_service_handler, 0);
        if((status = sms_imu_primary_service_define(&sms_imu_service_handler)) != AT_BLE_SUCCESS) {
            //DBG_LOG("[main]  defining SMS IMU service failed, reason 0x%x", status);
        }
        else {
            //DBG_LOG_DEV("[main]  SMS primary service defined, sms imu handle %d", sms_imu_service_handler.serv_handle);
        }
        
        sms_pressure_service_init(&sms_pressure_service_handler, 0);
        if((status = sms_pressure_primary_service_define(&sms_pressure_service_handler)) != AT_BLE_SUCCESS) {
            //DBG_LOG("[main]  defining SMS pressure service failed, reason 0x%x", status);
        }
        else {
            //DBG_LOG_DEV("[main]  SMS primary service defined, sms pressure handle %d", sms_pressure_service_handler.serv_handle);
        }
    }
    
    /* Register callbacks for GAP related events */
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, startup_template_app_gap_cb);
    /* Register callbacks for GATT SERVER related events */
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, startup_template_app_gatt_server_cb);
    /* Register user-defined callback */
    register_ble_user_event_cb(sms_plf_event_cb);
    
    sms_service_advertise();
    
    while(true)
    {
        /* BLE Event task */
        ble_event_task(BLE_EVENT_TIMEOUT);

        if(sms_plf_int_state.int_on) {
            acquire_sleep_lock();
            
            switch(sms_plf_int_state.source) {
                case SMS_CB_BT1:
                DBG_LOG_DEV("[main] button1 pressed...");
                sms_button_bt1_fn();
                break;

                case SMS_CB_BT2:
                DBG_LOG_DEV("[main] button2 pressed...");
                sms_button_bt2_fn();
                break;

                case SMS_CB_IMU_DRDY:
                //DBG_LOG_DEV("[main] imu interruption...");
                if(mpu9250_device.init_ok) sms_imu_poll_data();
                if(mpu9250_device.comm_error) sms_imu_startup();
                sms_dualtimer_timer2.cur_counter = (SMS_TIMER_AON_COUNT_10H - aon_sleep_timer_get_current_value());
                uint32_t delta = (uint32_t)((sms_dualtimer_timer2.cur_counter - sms_dualtimer_timer2.old_counter) / SMS_TIMER_AON_COUNT_1MS);
                DBG_LOG_DEV("[main]  elapsed timer: %ld ms", delta);
                if((delta > MS58_CONV_WAIT_MS) && (ms58_device.init_ok)) {
                    DBG_LOG_CONT_DEV(" ...time to read ms58");
                    sms_dualtimer_timer2.old_counter = sms_dualtimer_timer2.cur_counter;
                    gpio_pin_set_output_level(PIN_LP_GPIO_5, true);
                    sms_pressure_ms58_poll_data();
                    gpio_pin_set_output_level(PIN_LP_GPIO_5, false);
                }
                else {
                    DBG_LOG_CONT_DEV(" ...not yet");
                }
                break;

                case SMS_CB_AON_TIMER:
                DBG_LOG_DEV("[main] aon timer interruption... reading ms58");
                gpio_pin_set_output_level(PIN_LP_GPIO_5, true);
                sms_pressure_ms58_poll_data();
                gpio_pin_set_output_level(PIN_LP_GPIO_5, false);
                break;

                case SMS_CB_DUALTIMER1:
                DBG_LOG_DEV("[main]  dualtimer1 interruption...");
                //if(ms58_device.init_ok) {
                    //gpio_pin_set_output_level(PIN_LP_GPIO_5, true);
                    //sms_pressure_ms58_poll_data();
                    //gpio_pin_set_output_level(PIN_LP_GPIO_5, false);
                //}
                break;
                
                case SMS_CB_DUALTIMER2:
                DBG_LOG_DEV("[main]  dualtimer2 interruption...");
                break;
                
                case SMS_CB_NONE:
                default:
                break;
            }
            sms_plf_int_state.source = SMS_CB_AON_TIMER;
            sms_plf_int_state.int_on = false;
            //release_sleep_lock();
        }

        //if(sms_timer0_state.int_on) {
        //switch(sms_timer0_state.source) {
        //case SMS_TIMER_DEBOUNCE:
        //DBG_LOG_DEV("[main] debouncing time elapsed");
        //break;
        //
        //case SMS_TIMER_MS58_RESET:
        //DBG_LOG_DEV("[main] ms58 reset time elapsed");
        //break;
        //
        //case SMS_TIMER_MS58_D1_CONV:
        //DBG_LOG_DEV("[main] ms58 D1 conversion time elapsed");
        //break;
        //
        //case SMS_TIMER_MS58_D2_CONV:
        //DBG_LOG_DEV("[main] ms58 D2 conversion time elapsed");
        //break;
        //
        //case SMS_TIMER_SPI_FINISH:
        //DBG_LOG_DEV("[main] SPI finish waiting timer elapsed");
        //break;
        //
        //case SMS_TIMER_DELTA_CNT:
        //DBG_LOG_DEV("[main] SMS node delta counter done");
        //break;
        //
        //case SMS_TIMER_GET_MS:
        //gpio_pin_set_output_level(PIN_LP_GPIO_5, true);
        //sms_timer_cycle_cnt++;
        //DBG_LOG_DEV("[main] timer get ms done... cnt: %ld", sms_timer_cycle_cnt);
        //gpio_pin_set_output_level(PIN_LP_GPIO_5, false);
        //break;
        //
        //case SMS_TIMER_DUMMY:
        //DBG_LOG_DEV("[main] timer dummy done");
        //break;
        //
        //case SMS_TIMER_NONE:
        //default:
        //DBG_LOG_DEV("[main] no timer source selected");
        //break;
        //}
        //sms_timer0_state.int_on = false;
        //}
    }
}

