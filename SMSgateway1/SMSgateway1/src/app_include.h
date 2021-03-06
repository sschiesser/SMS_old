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

/** @brief APP_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
#define APP_FAST_ADV						(1600)

/** @brief APP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in seconds, 0x0000 disables time-out.*/
#define APP_ADV_TIMEOUT						(655)

#define SMS_ID_PATTERN1                     "SABRe-SMS"
#define SMS_ID_PATTERN2                     "\x1C\x57\x2d\x5A\xBE\x2d\x53\x50"

#define SMS_BLE_SERV_BUTTON_POS				0
#define SMS_BLE_SERV_PRESSURE_POS			1
#define SMS_BLE_SERV_MPU_POS				2
#define SMS_BLE_SERVICE_MAX					3

#define SMS_BLE_CHAR_BTN0_POS				0
#define SMS_BLE_CHAR_BNT1_POS				0
#define SMS_BLE_CHAR_PRESS_POS				1
#define SMS_BLE_CHAR_MPU_POS				2
#define SMS_BLE_CHARS_MAX					4

#define SMS_BLE_PERIPHERAL_MAX				BLE_MAX_DEVICE_CONNECTED

#define SPI_BUF_LENGTH						64

typedef enum {
	SMS_DEV_UNCONNECTED,
	SMS_DEV_CONNECTING,
	SMS_DEV_CONNECTED,
	SMS_DEV_PAIRED,
	SMS_DEV_SERVICE_FOUND
} SMS_DEV;

typedef enum sms_state {
    SMS_STARTING,
    SMS_CONNECTING,
    SMS_RUNNING,
    SMS_DISCONNECTED,
    SMS_INT_BUTTON1,
    SMS_INT_BUTTON2,
    SMS_INT_TIMER1,
    SMS_INT_TIMER2
}sms_state_t;
volatile sms_state_t app_state;

struct sms_spi_message {
	uint8_t periph_id;
	uint8_t service;
	uint8_t length;
	uint8_t data[61];
};
struct sms_spi_message spi_message;
bool spi_send;
struct spi_slave_inst spi_slave;
struct spi_module spi_master_instance;

//enum sms_ble_serv_type {
	//BLE_SERV_DISABLED = 0x00,
	//BLE_SERV_BUTTON,
	//BLE_SERV_PRESSURE,
	//BLE_SERV_MPU
//};
//
//enum sms_ble_char_type {
	//BLE_CHAR_DISABLED = 0x00,
	//BLE_CHAR_BTN0,
	//BLE_CHAR_BTN1,
	//BLE_CHAR_PRESS,
	//BLE_CHAR_MPU
//};
typedef struct sms_periph {
	uint8_t id;
	uint16_t conn_handle;
	bool available_services[SMS_BLE_SERVICE_MAX];
	//bool available_chars[SMS_BLE_CHARS_MAX];
	uint16_t service_handle_range[SMS_BLE_SERVICE_MAX][2];
}sms_periph_t;
sms_periph_t periph_instance[SMS_BLE_PERIPHERAL_MAX];
uint8_t periph_counter;

typedef struct gatt_smsb_char_handler
{
	at_ble_handle_t start_handle;
	at_ble_handle_t end_handler;
	at_ble_handle_t char_handle;
	at_ble_status_t char_discovery;
	uint8_t *char_data;
}gatt_smsb_char_handler_t;

at_ble_status_t sms_gateway_discover_services(at_ble_handle_t handle);
at_ble_status_t sms_gateway_discover_characteristics(at_ble_handle_t handle);

#endif /* __STARTUP_TEMPLATE_H__ */
