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
#include <asf.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "pxp_monitor.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "button.h"
#include "app_include.h"

bool volatile app_timer_done = false;
//bool volatile dbg_pin_state = false;
extern ble_connected_dev_info_t ble_dev_info[BLE_MAX_DEVICE_CONNECTED];
extern volatile uint8_t scan_response_count;
extern at_ble_scan_info_t scan_info[MAX_SCAN_DEVICE];
uint8_t scan_index;
uint8_t supp_scan_index[MAX_SCAN_DEVICE];
volatile uint8_t sms_gateway_connection_flag = SMS_DEV_UNCONNECTED;
uint8_t dbg_gpio_pin = PIN_LP_GPIO_12;
volatile bool dbg_gpio_pin_state = false;

/* ------------------ */
/* Callback functions */
/* ------------------ */

/* GAP */
/* Callback registered for AT_BLE_SCAN_REPORT (#2) event from stack */
static at_ble_status_t ble_scan_data_app_event(void *param)
{
    uint8_t scan_device[MAX_SCAN_DEVICE];
    uint8_t scan_device_count = 0;
    uint8_t scanned_dev_count = scan_response_count;
    scan_index = 0;
    uint8_t index;
    bool device_identified = false;
    uint8_t device_index = 0;
    char search_pattern1[strlen(SMS_ID_PATTERN1)];
    char search_pattern2[strlen(SMS_ID_PATTERN2)];
    at_ble_scan_info_t *scan_buffer = (at_ble_scan_info_t *)scan_info;
    memset(scan_device, 0, MAX_SCAN_DEVICE);
    if (scanned_dev_count) {
        at_ble_uuid_t service_uuid;

        for (index = 0; index < scanned_dev_count; index++) {
            /* Display only the connectible devices*/
            if((scan_buffer[index].type == AT_BLE_ADV_TYPE_DIRECTED)
            || (scan_buffer[index].type == AT_BLE_ADV_TYPE_UNDIRECTED)) {
                scan_device[scan_device_count++] = index;
            }
        }
        
        if (scan_device_count) {
            /* Service type to be searched */
            service_uuid.type = AT_BLE_UUID_16;

            /* Service UUID */
            service_uuid.uuid[1] = (LINK_LOSS_SERVICE_UUID >> 8);
            service_uuid.uuid[0] = (uint8_t)LINK_LOSS_SERVICE_UUID;
            
            memcpy(search_pattern1, SMS_ID_PATTERN1, strlen(SMS_ID_PATTERN1));
            memcpy(search_pattern2, SMS_ID_PATTERN2, strlen(SMS_ID_PATTERN2));

            for (index = 0; index < scan_device_count; index++) {
                DBG_LOG("Info: Device found address [%d]  0x%02X%02X%02X%02X%02X%02X ",
                index,
                scan_buffer[scan_device[index]].dev_addr.addr[5],
                scan_buffer[scan_device[index]].dev_addr.addr[4],
                scan_buffer[scan_device[index]].dev_addr.addr[3],
                scan_buffer[scan_device[index]].dev_addr.addr[2],
                scan_buffer[scan_device[index]].dev_addr.addr[1],
                scan_buffer[scan_device[index]].dev_addr.addr[0]);

                DBG_LOG_DEV("      - advertised data: 0x ");
                uint8_t i = 0;
                uint8_t search_index = 0;
                bool id1 = true;
                bool id2 = false;
                for(i = 0; i < AT_BLE_ADV_MAX_SIZE; i++) {
                    if(id1) {
                        DBG_LOG_DEV("%02x... looking for %02x", scan_buffer[scan_device[index]].adv_data[i], search_pattern1[search_index]);
                        if(scan_buffer[scan_device[index]].adv_data[i] == search_pattern1[search_index]) {
                            DBG_LOG_CONT_DEV(" FOUND! ...index: %d", search_index);
                            if(search_index < (sizeof(search_pattern1)-1)) {
                                search_index += 1;
                                } else {
                                DBG_LOG_DEV("PATTERN1 MATCHING!!!");
                                device_identified = true;
                                device_index = index;
                                break;
                                //search_index = 0;
                                //id1 = false;
                                //id2 = true;
                            }
                        }
                        } else if(id2) {
                        DBG_LOG_DEV("%02x... looking for %02x", scan_buffer[scan_device[index]].adv_data[i], search_pattern2[search_index]);
                        if(scan_buffer[scan_device[index]].adv_data[i] == search_pattern2[search_index]) {
                            DBG_LOG_CONT_DEV(" FOUND! ...index: %d", search_index);
                            if(search_index < (sizeof(search_pattern2)-1)) {
                                search_index += 1;
                                } else {
                                DBG_LOG_DEV("PATTERN2 MATCHING!!!");
                                device_identified = true;
                                device_index = index;
                                break;
                            }
                        }
                    }
                }
                
                if (scan_info_parse(&scan_buffer[scan_device[index]], &service_uuid, AD_TYPE_COMPLETE_LIST_UUID) == AT_BLE_SUCCESS) {
                    /* Device Service UUID  matched */
                    supp_scan_index[scan_index++] = index;
                    DBG_LOG_CONT("---PXP");
                }
            }
        }

        if (!scan_index)  {
            DBG_LOG("Proximity Profile supported device not found ");
        }
        
        /* Stop the current scan active */
        at_ble_scan_stop();
        
        if(device_identified) {
            DBG_LOG_DEV("Compatible device found");
            return pxp_monitor_connect_request(scan_buffer, scan_device[device_index]);
        }
        /*Updating the index pointer to connect */
        else if(scan_device_count) {
            /* Successful device found event*/
            DBG_LOG_DEV("NO compatible device found");
            app_state = SMS_RUNNING;
            //uint8_t deci_index = scan_device_count;
            //deci_index+=PXP_ASCII_TO_DECIMAL_VALUE;
            //do {
            //DBG_LOG("Select Index number to Connect or [s] to scan");
            //index = getchar_b11();
            //DBG_LOG("%c", index);
            //} while (!(((index < (deci_index)) && (index >='0')) || (index == 's')));
            //
            //if(index == 's') {
            //return gap_dev_scan();
            //} else {
            //index -= PXP_ASCII_TO_DECIMAL_VALUE;
            //return pxp_monitor_connect_request(scan_buffer,	scan_device[index]);
            //}
        }
    }
    else {
        /* from no device found event*/
        DBG_LOG_DEV("Nothing found");
        app_state = SMS_RUNNING;
        //do
        //{
        //DBG_LOG("Select [s] to scan again");
        //index = getchar_b11();
        //DBG_LOG("%c", index);
        //} while (!(index == 's'));
        //
        //if(index == 's') {
        //return gap_dev_scan();
        //}
    }
    ALL_UNUSED(param);
    return AT_BLE_FAILURE;
}

/* Callback registered for AT_BLE_CONNECTED (#5) event from stack */
static at_ble_status_t ble_connected_app_event(void *params)
{
    at_ble_connected_t *conn_params;
    conn_params = (at_ble_connected_t *)params;

    if(!ble_check_iscentral(conn_params->handle))
    {
        return AT_BLE_FAILURE;
    }
    
    DBG_LOG_DEV("Device connected...");

    //at_ble_status_t discovery_status = AT_BLE_FAILURE;

    //discovery_status = sms_gateway_discover_services(pair_done_val->handle);
    //discovery_status = sms_gateway_discover_characteristics(conn_params->handle);
    //if(discovery_status != AT_BLE_SUCCESS) {
    //DBG_LOG_DEV("Characteristic discovering start failed!");
    //} else {
    //DBG_LOG_DEV("Characteristic discovering start successful");
    //}

    sms_gateway_connection_flag = SMS_DEV_CONNECTED;
    return conn_params->conn_status;
}

/* Callback registered for AT_BLE_DISCONNECTED (#6) event from stack */
static at_ble_status_t ble_disconnected_app_event(void *params)
{
    ALL_UNUSED(params);
    app_state = SMS_DISCONNECTED;
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_PAIR_DONE (#9) event from stack */
static at_ble_status_t ble_paired_app_event(void *params)
{
    DBG_LOG_DEV("Devices paired...");
    at_ble_status_t discovery_status = AT_BLE_FAILURE;
    at_ble_pair_done_t *pair_done_val;
    pair_done_val = (at_ble_pair_done_t *)params;
	
	if(periph_counter == 0xff) {
		periph_counter = 0;
		DBG_LOG_DEV("Registering first peripheral");
	}
	else if((periph_counter + 1) >= SMS_BLE_PERIPHERAL_MAX) {
		DBG_LOG_DEV("Maximum amount of peripherals reached!");
		return AT_BLE_ATT_INSUFF_RESOURCE;
	}
	else {
		periph_counter += 1;
		DBG_LOG_DEV("Increasing peripheral counter: %d", periph_counter);
	}
	periph_instance[periph_counter].id = periph_counter;
	periph_instance[periph_counter].conn_handle = pair_done_val->handle;

    discovery_status = sms_gateway_discover_services(pair_done_val->handle);
    //discovery_status = sms_gateway_discover_characteristics(pair_done_val->handle);
    //if(discovery_status != AT_BLE_SUCCESS) {
    //DBG_LOG_DEV("Characteristic discovering start failed!");
    //} else {
    //DBG_LOG_DEV("Characteristic discovering start successful");
    //}

    sms_gateway_connection_flag = SMS_DEV_PAIRED;

    return discovery_status;
}

static const ble_event_callback_t sms_gateway_app_gap_cb[] = {
    NULL,								// AT_BLE_UNDEFINED_EVENT
    NULL,								// AT_BLE_SCAN_INFO
    ble_scan_data_app_event,			// AT_BLE_SCAN_REPORT
    NULL,								// AT_BLE_ADV_REPORT
    NULL,								// AT_BLE_RAND_ADDR_CHANGED
    ble_connected_app_event,			// AT_BLE_CONNECTED
    ble_disconnected_app_event,			// AT_BLE_DISCONNECTED
    NULL,								// AT_BLE_CONN_PARAM_UPDATE_DONE
    NULL,								// AT_BLE_CONN_PARAM_UPDATE_REQUEST
    ble_paired_app_event,				// AT_BLE_PAIR_DONE
    NULL,								// AT_BLE_PAIR_REQUEST
    NULL,								// AT_BLE_SLAVE_SEC_REQUEST
    NULL,								// AT_BLE_PAIR_KEY_REQUEST
    NULL,								// AT_BLE_ENCRYPTION_REQUEST
    NULL,								// AT_BLE_ENCRYPTION_STATUS_CHANGED
    NULL,								// AT_BLE_RESOLV_RAND_ADDR_STATUS
    NULL,								// AT_BLE_SIGN_COUNTERS_IND
    NULL,								// AT_BLE_PEER_ATT_INFO_IND
    NULL								// AT_BLE_CON_CHANNEL_MAP_IND
};

/* GATT SERVER */
/* .... */

/* GATT CLIENT */
/* Callback registered for AT_BLE_PRIMARY_SERVICE_FOUND (#19) event from stack */
static at_ble_status_t sms_gateway_service_found(void *params)
{
	uint16_t comp_val = 0;
    DBG_LOG_DEV("Primary service found");
    at_ble_primary_service_found_t *service = (at_ble_primary_service_found_t *)params;
    DBG_LOG_DEV("[sms_gateway_service_found]  service characteristics:");
    DBG_LOG_DEV("  service type: %d", service->service_uuid.type);
    DBG_LOG_DEV("  service uuid: 0x");
    for(uint8_t i = 0; i < AT_BLE_UUID_128_LEN; i++) {
        DBG_LOG_CONT_DEV("%02x", service->service_uuid.uuid[i]);
		if(i == 6) {
			comp_val = service->service_uuid.uuid[i];
			comp_val = (comp_val << 8) & 0xff00;
			//DBG_LOG_DEV("Comp val: 0x%04x", comp_val);
		}
		else if(i == 7) {
			comp_val |= service->service_uuid.uuid[i];
			//DBG_LOG_DEV("Comp val: 0x%04x", comp_val);
		}
    }
	DBG_LOG_DEV("  comp val: 0x%04x", comp_val);
    DBG_LOG_DEV("  start handle: %d", service->start_handle);
    DBG_LOG_DEV("  end handle: %d", service->end_handle);
	
	if(comp_val == 0xbbbb) {
		DBG_LOG_DEV("  button service!");
		periph_instance[periph_counter].available_services[SMS_BLE_SERV_BUTTON_POS] = true;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_BUTTON_POS][0] = service->start_handle;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_BUTTON_POS][1] = service->end_handle;
	}
	else if(comp_val == 0xeeee) {
		DBG_LOG_DEV("  pressure service!");
		periph_instance[periph_counter].available_services[SMS_BLE_SERV_PRESSURE_POS] = true;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_PRESSURE_POS][0] = service->start_handle;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_PRESSURE_POS][1] = service->end_handle;
	}
	else if(comp_val == 0x1111) {
		DBG_LOG_DEV("  mpu service!");
		periph_instance[periph_counter].available_services[SMS_BLE_SERV_MPU_POS] = true;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_MPU_POS][0] = service->start_handle;
		periph_instance[periph_counter].service_handle_range[SMS_BLE_SERV_MPU_POS][1] = service->end_handle;
	}
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_FOUND event from stack */
static at_ble_status_t sms_gateway_char_found(void *params)
{
    DBG_LOG_DEV("Characteristic found");
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DESCRIPTOR_FOUND event from stack */
static at_ble_status_t sms_gateway_descr_found(void *params)
{
    DBG_LOG("Descriptor found");
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCOVERY_COMPLETE (#23) event from stack */
static at_ble_status_t sms_gateway_discovery_complete(void *params)
{
    DBG_LOG("Discovery complete...");
	for(uint8_t i = 0; i < SMS_BLE_PERIPHERAL_MAX; i++) {
		DBG_LOG_DEV("Peripheral #%d", i);
		DBG_LOG_DEV(" - id %d", periph_instance[i].id);
		DBG_LOG_DEV(" - conn handle 0x%04x", periph_instance[i].conn_handle);
		for(uint8_t j = 0; j < SMS_BLE_SERVICE_MAX; j++) {
			DBG_LOG_DEV(" - service%d: %d", j, periph_instance[i].available_services[j]);
			DBG_LOG_DEV(" - char range: %d - %d", periph_instance[i].service_handle_range[j][0], periph_instance[i].service_handle_range[j][1]);
		}
	}
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_READ_BY_UUID_RESPONSE event from stack */
static at_ble_status_t 	sms_gateway_char_read_by_uuid(void *params)
{
    DBG_LOG("Characteristic read by uuid response");
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_READ_MULTIBLE_RESPONSE event from stack */
static at_ble_status_t sms_gateway_char_read_multiple(void *params)
{
    DBG_LOG("Characteristic read by multiple BLE responses");
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_WRITE_RESPONSE event from stack */
static at_ble_status_t sms_gateway_char_write_resp(void *params)
{
    DBG_LOG("Characteristic write response");
    UNUSED(params);
    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_NOTIFICATION_RECIEVED (#27) event from stack */
static at_ble_status_t sms_gateway_notification_received(void *params)
{
    gpio_pin_set_output_level(dbg_gpio_pin, true);

    static uint16_t sms_rcv_cnt = 0;
    at_ble_notification_recieved_t *notification = (at_ble_notification_recieved_t *)params;
    sms_rcv_cnt++;
    DBG_LOG_DEV("cnt: %d", sms_rcv_cnt);
    DBG_LOG("[sms_gateway_notification_received]\tNotification received...\r\n- conn handle: 0x%04x\r\n- char handle: 0x%04x\r\n- char len: %d\r\n- char value: 0x", notification->conn_handle, notification->char_handle, notification->char_len);
    for(uint8_t i = 0; i < notification->char_len; i++) {
        DBG_LOG_CONT_DEV("%02x", notification->char_value[i]);
    }
	
	for(uint8_t i = 0; i < SMS_BLE_PERIPHERAL_MAX; i++) {
		if(periph_instance[i].conn_handle == notification->conn_handle) {
			spi_message.periph_id = periph_instance[i].id;
			for(uint8_t j = 0; j < SMS_BLE_SERVICE_MAX; j++) {
				if((notification->char_handle > periph_instance[i].service_handle_range[j][0]) && (notification->char_handle < periph_instance[i].service_handle_range[j][1])) {
					spi_message.service = j;
					spi_message.length = notification->char_len;
					DBG_LOG_DEV("Peripheral %d, service %d, length %d", spi_message.periph_id, spi_message.service, spi_message.length);
					DBG_LOG_DEV("Data 0x");
					for(uint8_t k = 0; k < spi_message.length; k++) {
						spi_message.data[k] = notification->char_value[k];
						DBG_LOG_CONT_DEV("%02x ", spi_message.data[k]);
						spi_send = true;
					}
					break;
				}
			}
		}
	}
	

    gpio_pin_set_output_level(dbg_gpio_pin, false);

    return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_INDICATION_RECIEVED event from stack */
static at_ble_status_t sms_gateway_indication_received(void *param)
{
    gpio_pin_set_output_level(dbg_gpio_pin, true);

    at_ble_indication_recieved_t *indication = (at_ble_indication_recieved_t *)param;
    static uint16_t btn_ind_cnt = 0;
    static uint16_t press_ind_cnt = 0;
    static uint16_t imu_ind_cnt = 0;
    switch(indication->char_len) {
        case 1:
        DBG_LOG_DEV("BTN %d: 0x%02x", btn_ind_cnt++, indication->char_value[0]);
        break;
        
        case 8:
        DBG_LOG_DEV("\t\tPRESS %d: 0x", press_ind_cnt++);
        for(uint8_t i = 0; i < 8; i++) {
            DBG_LOG_CONT_DEV("%02x", indication->char_value[i]);
        }
        break;
        
        case 12:
        DBG_LOG_DEV("\t\t\t\tIMU %d: 0x", imu_ind_cnt++);
        for(uint8_t i = 0; i < 12; i++) {
            DBG_LOG_CONT_DEV("%02x", indication->char_value[i]);
        }
        break;
        
        default:
        break;
    }    
    //DBG_LOG_DEV("Indication counter: %d... len: %d", sms_ind_cnt, indication->char_len);
    //DBG_LOG_DEV("Indication received...\r\n- conn handle: 0x%04x\r\n- char handle: 0x%04x\r\n- char len: %d", indication->conn_handle, indication->char_handle, indication->char_len);
    //for(uint8_t i = 0; i < indication->char_len; i++) {
        //DBG_LOG_CONT_DEV("\r\n- char value[%d]: 0x%02x", i, indication->char_value[i]);
    //}
 
    gpio_pin_set_output_level(dbg_gpio_pin, false);

    return AT_BLE_SUCCESS;
}

static const ble_event_callback_t sms_gateway_app_gatt_client_cb[] = {
    sms_gateway_service_found,			// AT_BLE_PRIMARY_SERVICE_FOUND
    NULL,								// AT_BLE_INCLUDED_SERVICE_FOUND
    sms_gateway_char_found,				// AT_BLE_CHARACTERISTIC_FOUND
    sms_gateway_descr_found,			// AT_BLE_DESCRIPTOR_FOUND
    sms_gateway_discovery_complete,		// AT_BLE_DISCOVERY_COMPLETE
    sms_gateway_char_read_by_uuid,		// AT_BLE_CHARACTERISTIC_READ_BY_UUID_RESPONSE
    sms_gateway_char_read_multiple,		// AT_BLE_CHARACTERISTIC_READ_MULTIBLE_RESPONSE
    sms_gateway_char_write_resp,		// AT_BLE_CHARACTERISTIC_WRITE_RESPONSE
    sms_gateway_notification_received,	// AT_BLE_NOTIFICATION_RECIEVED
    sms_gateway_indication_received		//	AT_BLE_INDICATION_RECIEVED
};


/* timer callback function */
static void timer_callback_fn(void)
{
    app_state = SMS_INT_TIMER1;
    /* Add timer callback functionality here */
}

static void button_cb(void)
{
    /* Add button callback functionality here */
    app_state = SMS_INT_BUTTON2;
    send_plf_int_msg_ind(BUTTON_0_PIN, GPIO_CALLBACK_RISING, NULL, 0);
}

static void sms_gateway_init(void)
{
    at_ble_status_t scan_status;

    scan_status = gap_dev_scan();

    if(scan_status == AT_BLE_INVALID_PARAM) {
        DBG_LOG("Scan parameters are invalid");
        } else if(scan_status == AT_BLE_FAILURE) {
        DBG_LOG("Scanning failed generic error");
    }
}

at_ble_status_t sms_gateway_discover_services(at_ble_handle_t handle)
{
    at_ble_status_t status;
    status = at_ble_primary_service_discover_all(handle, GATT_DISCOVERY_STARTING_HANDLE, GATT_DISCOVERY_ENDING_HANDLE);
    if(status != AT_BLE_SUCCESS) {
        DBG_LOG("GATT service discovery request failed");
        return AT_BLE_FAILURE;
        } else {
        DBG_LOG_DEV("GATT service discovery request started");
        return AT_BLE_SUCCESS;
    }
}

at_ble_status_t sms_gateway_discover_characteristics(at_ble_handle_t handle)
{
    at_ble_status_t status;
    status = at_ble_characteristic_discover_all(handle, GATT_DISCOVERY_STARTING_HANDLE, GATT_DISCOVERY_ENDING_HANDLE);
    return status;
}

static void sms_gpio_init(void)
{
    struct gpio_config config_gpio_pin;
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction  = GPIO_PIN_DIR_OUTPUT;
    if(gpio_pin_set_config(dbg_gpio_pin, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("Problem while setting gpio pin");
    }
    gpio_pin_set_output_level(dbg_gpio_pin, dbg_gpio_pin_state);
}

static void sms_disconnected_fn(void)
{
    app_state = SMS_RUNNING;
}

static void sms_int_button1_fn(void)
{
    app_state = SMS_RUNNING;
}

static void sms_int_button2_fn(void)
{
    sms_gateway_init();
    app_state = SMS_RUNNING;
}

static void sms_int_timer1_fn(void)
{
    //if(sms_gateway_connection_flag == SMS_DEV_CONNECTING) {
    //at_ble_disconnected_t sms_connect_request_fail;
    //sms_connect_request_fail.reason = AT_BLE_TERMINATED_BY_USER;
    //sms_connect_request_fail.handle = ble_dev_info[0].conn_info.handle;
    //sms_gateway_connection_flag = SMS_DEV_UNCONNECTED;
    //if (at_ble_connect_cancel() == AT_BLE_SUCCESS) {
    //DBG_LOG("Connection Timeout");
    //pxp_disconnect_event_handler(&sms_connect_request_fail);
    //} else {
    //DBG_LOG("Unable to connect with device");
    //}
    //} else if(sms_gateway_connection_flag == SMS_DEV_SERVICE_FOUND) {
    //hw_timer_start(PXP_RSSI_UPDATE_INTERVAL);
    //}
    app_state = SMS_RUNNING;
}

static void sms_int_timer2_fn(void)
{
    app_state = SMS_RUNNING;
}

int main(void)
{
    app_state = SMS_STARTING;

    platform_driver_init();
    acquire_sleep_lock();

    /* Initialize serial console */
    serial_console_init();
    
    /* Hardware timer */
    //hw_timer_init();
    //dualtimer_disable(DUALTIMER_TIMER1);
    //dualtimer_disable(DUALTIMER_TIMER2);
    
    /* button initialization */
    button_init(button_cb);

    sms_gpio_init();
    
    hw_timer_register_callback(timer_callback_fn);

    DBG_LOG("Initializing BLE Application");
    
    /* initialize the BLE chip and Set the Device Address */
    ble_device_init(NULL);

    /* Register callbacks for GAP related events */
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, sms_gateway_app_gap_cb);
    

    /* Register callbacks for GATT CLIENT related events */
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_CLIENT_EVENT_TYPE, sms_gateway_app_gatt_client_cb);
    
    //while(app_state != SMS_INT_BUTTON2);;
    //
    //app_state = SMS_RUNNING;

    sms_gateway_init();

    register_hw_timer_start_func_cb((hw_timer_start_func_cb_t)hw_timer_start);
    register_hw_timer_stop_func_cb(hw_timer_stop);
    
	periph_counter = 0xFF;
	for(uint8_t i = 0; i < SMS_BLE_PERIPHERAL_MAX; i++) {
		periph_instance[i].id = 0xff;
		periph_instance[i].conn_handle = 0xffff;
		for(uint8_t j = 0; j < SMS_BLE_SERVICE_MAX; j++) {
			periph_instance[i].available_services[j] = false;
			periph_instance[i].service_handle_range[j][0] = 0xff;
			periph_instance[i].service_handle_range[j][1] = 0x00;
		}
	}
    
    while(true)
    {
        /* BLE Event task */
        ble_event_task(BLE_EVENT_TIMEOUT);
        
        switch(app_state) {
            case SMS_STARTING:
            break;

            case SMS_CONNECTING:
            break;

            case SMS_RUNNING:
            DBG_LOG_DEV("SMS_RUNNING...");
            break;

            case SMS_DISCONNECTED:
            DBG_LOG_DEV("SMS_DISCONNECTED...");
            sms_disconnected_fn();
            break;

            case SMS_INT_BUTTON1:
            DBG_LOG_DEV("SMS_INT_BUTTON1...");
            sms_int_button1_fn();
            break;

            case SMS_INT_BUTTON2:
            DBG_LOG_DEV("SMS_INT_BUTTON2...");
            sms_int_button2_fn();
            break;

            case SMS_INT_TIMER1:
            DBG_LOG_DEV("SMS_INT_TIMER1...");
            sms_int_timer1_fn();
            break;

            case SMS_INT_TIMER2:
            DBG_LOG_DEV("SMS_INT_TIMER2...");
            sms_int_timer2_fn();
            break;

            default:
            break;
        }
        /* Application Task */
        //if (app_timer_done) {
        //if(sms_gateway_connection_flag == SMS_DEV_CONNECTING) {
        //at_ble_disconnected_t sms_connect_request_fail;
        //sms_connect_request_fail.reason = AT_BLE_TERMINATED_BY_USER;
        //sms_connect_request_fail.handle = ble_dev_info[0].conn_info.handle;
        //sms_gateway_connection_flag = SMS_DEV_UNCONNECTED;
        //if (at_ble_connect_cancel() == AT_BLE_SUCCESS) {
        //DBG_LOG("Connection Timeout");
        //pxp_disconnect_event_handler(&sms_connect_request_fail);
        //} else {
        //DBG_LOG("Unable to connect with device");
        //}
        //} else if(sms_gateway_connection_flag == SMS_DEV_SERVICE_FOUND) {
        //hw_timer_start(PXP_RSSI_UPDATE_INTERVAL);
        //}
        //
        //app_timer_done = false;
        //}
    }

}

