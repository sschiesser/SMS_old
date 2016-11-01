/**
 * \file
 *
 * \brief Startup Template declarations
 *
 * Copyright (c) 2014-2016 Atmel Corporation. All rights reserved.
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

#ifndef __STARTUP_TEMPLATE_H__
#define __STARTUP_TEMPLATE_H__

/* -------
 * INCLUDE
 * ------- */
#include <asf.h>
#include "at_ble_api.h"
#include "platform.h"
#include "console_serial.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "battery.h"
/* SMS include */
#include "sms_button.h"
#include "sms_ble.h"
#include "sms_pressure.h"
#include "sms_spi.h"
#include "sms_imu.h"
#include "sms_i2c.h"
#include "sms_timer.h"
#include "ms58.h"
#include "mpu9250.h"
/* Motion driver include */
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu.h"
#include "log.h"
/* Delay */
#include "delay.h"


/* ------
 * MACROS
 * ------ */
#define GATEWAY        0x01
#define NODE_FULL      0x11
#define NODE_RED       0x12

/** @brief APP_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
#define APP_FAST_ADV						(1600)

/** @brief APP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in seconds, 0x0000 disables time-out.*/
//#define APP_ADV_TIMEOUT						(655) // 655 -> ~ 10 minutes
#define APP_ADV_TIMEOUT                     (30)

#define APP_UPDATE_MS                       (500)


int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t const *data);
int i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t *data);


/* ---------
 * VARIABLES
 * --------- */
 typedef enum sms_device_type {
    SMS_DEVICE_GATEWAY,
    SMS_DEVICE_NODE_FULL,
    SMS_DEVICE_NODE_REDUCED
 }sms_device_type_t;

 typedef enum sms_app_state {
    SMS_APP_STARTING,
    SMS_APP_CONNECTING,
    SMS_APP_RUNNING,
    SMS_APP_MS58_RESET_DONE,
    SMS_APP_MS58_READ_PROM_DONE,
    SMS_APP_ERROR
 }sms_app_state_t;
volatile sms_app_state_t app_state;

typedef enum sms_plf_callback_type {
    SMS_CB_NONE,
    SMS_CB_BT1,
    SMS_CB_BT2,
    SMS_CB_IMU_DRDY,
    SMS_CB_AON_TIMER,
    SMS_CB_DUALTIMER1,
    SMS_CB_DUALTIMER2
}sms_plf_callback_type_t;
typedef struct sms_plf_int {
    sms_plf_callback_type_t source;
    bool int_on;
}sms_plf_int_t;
volatile sms_plf_int_t sms_plf_int_state;

typedef struct sms_dualtimer_state
{
    //sms_timer_callback_t source;
    volatile bool int_on;
    uint32_t load;
    volatile uint32_t cur_counter;
    volatile uint32_t old_counter;
}sms_dualtimer_state_t;
volatile sms_dualtimer_state_t sms_dualtimer_timer2;

typedef enum sms_modes {
    SMS_MODE_NONE,
    SMS_MODE_BUTTON_SOLO,
    SMS_MODE_IMU_SOLO,
    SMS_MODE_PRESSURE_SOLO,
    SMS_MODE_BUTTON_IMU,
    SMS_MODE_BUTTON_PRESSURE,
    SMS_MODE_IMU_PRESSURE,
    SMS_MODE_COMPLETE
}sms_modes_t;
sms_modes_t sms_working_mode;

/* ------------
 * DECLARATIONS
 * ------------ */

#endif /* __STARTUP_TEMPLATE_H__ */
