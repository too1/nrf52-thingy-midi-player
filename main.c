/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_uart.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_thingy_uis_c.h"
#include "ble_thingy_tms_c.h"
#include "ble_thingy_tss_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG      1                                     /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that will write to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCAN_INTERVAL             0x00A0                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION             0x0000                                /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(10, UNIT_1_25_MS)      /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(10, UNIT_1_25_MS)       /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                     /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 milliseconds. */


NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_THINGY_UIS_C_ARRAY_DEF(m_thingy_uis_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_THINGY_TMS_C_ARRAY_DEF(m_thingy_tms_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_THINGY_TSS_C_ARRAY_DEF(m_thingy_tss_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
enum {THINGY_SERVICE_MASK_UIS = 1, THINGY_SERVICE_MASK_TMS = 2, THINGY_SERVICE_MASK_TSS = 4, THINGY_SERVICE_ALL_MASKS = 0x07};
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

APP_TIMER_DEF(m_app_timer_perf_update);
APP_TIMER_DEF(m_change_note_timer);

enum {NOTE_C, NOTE_Db, NOTE_D, NOTE_Eb, NOTE_E, NOTE_F, NOTE_Gb, NOTE_G, NOTE_Ab, NOTE_A, NOTE_Bb, NOTE_B};

static char const m_target_periph_name[] = "PongThin";             /**< Name of the device we try to connect to. This name is searched for in the scan report data*/

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

// Game config
static bool  m_sound_enabled = false;

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MIN
};

/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,

    .timeout           = SCAN_DURATION,
    .scan_phys         = BLE_GAP_PHY_1MBPS,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,

};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}

static uint16_t thingy_under_discovery_conn_handle = 0xFFFF;

static void thingy_service_discovered(uint32_t service_mask)
{
    static uint32_t registered_masks[NRF_SDH_BLE_CENTRAL_LINK_COUNT] = {0};
    if(thingy_under_discovery_conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
    {
        registered_masks[thingy_under_discovery_conn_handle] |= service_mask;
        if(registered_masks[thingy_under_discovery_conn_handle] == THINGY_SERVICE_ALL_MASKS)
        {
            // All the services are discovered, which means the thingy is ready for use
            NRF_LOG_INFO("YAY");
            
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }
            registered_masks[thingy_under_discovery_conn_handle] = 0;
            
            // Set up the Thingy for use here
            ble_thingy_tms_c_euler_notif_enable(&m_thingy_tms_c[thingy_under_discovery_conn_handle]);
            ble_thingy_uis_c_button_notif_enable(&m_thingy_uis_c[thingy_under_discovery_conn_handle]);
            ble_thingy_tss_config_send(&m_thingy_tss_c[thingy_under_discovery_conn_handle], 
                                                TSS_CONFIG_SPEAKER_MODE_FREQUENCY, 0);
        
            thingy_under_discovery_conn_handle = 0xFFFF;
        }
    }
    
}

/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void thingy_uis_c_evt_handler(ble_thingy_uis_c_t * p_thingy_uis_c, ble_thingy_uis_c_evt_t * p_thingy_uis_c_evt)
{
    switch (p_thingy_uis_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("UIS service discovered");
            thingy_service_discovered(THINGY_SERVICE_MASK_UIS);
            break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
            NRF_LOG_INFO("Button pressed");
            break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION   */

        default:
            // No implementation needed.
            break;
    }
}
    
static void thingy_tms_c_evt_handler(ble_thingy_tms_c_t * p_thingy_tms_c, ble_thingy_tms_c_evt_t * p_thingy_tms_c_evt)  
{
    switch (p_thingy_tms_c_evt->evt_type)
    {
        case BLE_THINGY_TMS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("TMS service discovered");
            thingy_service_discovered(THINGY_SERVICE_MASK_TMS);
            break;

        case BLE_THINGY_TMS_C_EVT_EULER_NOTIFICATION:
            break;

        default:
            break;
    }  
}
    
static void thingy_tss_c_evt_handler(ble_thingy_tss_c_t * p_thingy_tss_c, ble_thingy_tss_c_evt_t * p_thingy_tss_c_evt) 
{
    switch (p_thingy_tss_c_evt->evt_type)
    {
        case BLE_THINGY_TSS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("TSS service discovered");
            thingy_service_discovered(THINGY_SERVICE_MASK_TSS);
            break;
            
        case BLE_THINGY_TSS_C_EVT_SPEAKER_STATUS_NOTIFICATION:
            break;
            
        case BLE_THINGY_TSS_C_EVT_MICROPHONE_NOTIFICATION:
            break;
    }
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;

    if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name))
    {
        NRF_LOG_INFO("Name match, attempting to connect");
        // Name is a match, initiate connection.
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            if(p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_CENTRAL)
            {
                NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                             p_gap_evt->conn_handle);

                APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                err_code = ble_thingy_uis_c_handles_assign(&m_thingy_uis_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);
            
                err_code = ble_thingy_tms_c_handles_assign(&m_thingy_tms_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);
            
                err_code = ble_thingy_tss_c_handles_assign(&m_thingy_tss_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                thingy_under_discovery_conn_handle = p_gap_evt->conn_handle;
            
                err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                                  p_gap_evt->conn_handle);
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }

                // Update LEDs status, and check if we should be looking for more
                // peripherals to connect to.
                bsp_board_led_on(CENTRAL_CONNECTED_LED);
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            if (ble_conn_state_central_conn_count() == 0)
            {
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);

                // Turn off connection indication LED
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }

            // Start scanning
            scan_start();

            // Turn on LED for indicating scanning
            bsp_board_led_on(CENTRAL_SCANNING_LED);
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(&p_gap_evt->params.adv_report);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


static void thingy_c_init(void)
{
    ret_code_t err_code;
    ble_thingy_uis_c_init_t thingy_uis_c_init_obj;
    ble_thingy_tms_c_init_t thingy_tms_c_init_obj;
    ble_thingy_tss_c_init_t thingy_tss_c_init_obj;
    
    thingy_uis_c_init_obj.evt_handler = thingy_uis_c_evt_handler;
    thingy_tms_c_init_obj.evt_handler = thingy_tms_c_evt_handler;
    thingy_tss_c_init_obj.evt_handler = thingy_tss_c_evt_handler;
    
    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_thingy_uis_c_init(&m_thingy_uis_c[i], &thingy_uis_c_init_obj);
        APP_ERROR_CHECK(err_code);
        err_code = ble_thingy_tms_c_init(&m_thingy_tms_c[i], &thingy_tms_c_init_obj);
        APP_ERROR_CHECK(err_code);
        err_code = ble_thingy_tss_c_init(&m_thingy_tss_c[i], &thingy_tss_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on if the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code from @ref ble_lbs_led_status_send.
 */
static ret_code_t led_status_send_to_all(uint8_t button_action)
{
    ret_code_t err_code;

    /*for (uint32_t i = 0; i< NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }*/
        return NRF_SUCCESS;
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            err_code = led_status_send_to_all(button_action);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_thingy_uis_on_db_disc_evt(&m_thingy_uis_c[p_evt->conn_handle], p_evt);
    ble_thingy_tms_on_db_disc_evt(&m_thingy_tms_c[p_evt->conn_handle], p_evt);
    ble_thingy_tss_on_db_disc_evt(&m_thingy_tss_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#define BASE_FREQ 200
float note_list[] = {1.0f, 1.0595f, 1.1225f, 1.1892f, 1.2599f, 1.3348f, 1.4142f, 1.4983f, 1.5874f, 1.6818f, 1.7818f, 1.8877f, 2.0f};

static uint16_t get_freq(uint32_t note, uint32_t octave)
{
    uint16_t freq = BASE_FREQ;
    while(octave--) freq *= 2;
    return (uint16_t)((float)freq * note_list[note]);

}

static void on_change_note(void *p)
{
    static uint16_t freq = 100;
    static uint32_t mode = 0;
    switch(mode++)
    {
        case 0:
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[0], get_freq(NOTE_C, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[1], get_freq(NOTE_E, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[2], get_freq(NOTE_G, 1), 500, 100);
            break;

        case 1:
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[0], get_freq(NOTE_C, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[1], get_freq(NOTE_F, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[2], get_freq(NOTE_A, 1), 500, 100);
            break;

        case 2:
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[0], get_freq(NOTE_C, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[1], get_freq(NOTE_G, 1), 500, 100);
            ble_thingy_tss_spk_data_frequency_send(&m_thingy_tss_c[2], get_freq(NOTE_Bb, 1), 500, 100);
            break;
    }
    mode %= 4;
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_change_note_timer, APP_TIMER_MODE_REPEATED, on_change_note);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

static void uart_init(void)
{
    uint32_t err_code;
     const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
#if defined (UART_PRESENT)
          UART_BAUDRATE_BAUDRATE_Baud460800
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         16,
                         128,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);   
}

#define PERF_ENABLE 0

#define PERF_UPDATE_MS  250
#define PERF_REF_COUNT  284400
#define PERF_RUN()      NRF_TIMER3->TASKS_COUNT = 1

static void perf_update(void *p)
{
    NRF_TIMER3->TASKS_CAPTURE[0] = 1;
    NRF_LOG_INFO("Perf: %i", (((NRF_TIMER3->CC[0]) * 1000 / PERF_REF_COUNT)));
    NRF_TIMER3->TASKS_CLEAR = 1;
}

static void perf_init(void)
{
#if (PERF_ENABLE == 1)
    app_timer_create(&m_app_timer_perf_update, APP_TIMER_MODE_REPEATED, perf_update);
    app_timer_start(m_app_timer_perf_update, APP_TIMER_TICKS(PERF_UPDATE_MS), 0);
    
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->TASKS_START = 1;
#endif
}


int main(void)
{
    uint32_t err_code;
    
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    uart_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    thingy_c_init();
    ble_conn_state_init();

    // Start execution.
    NRF_LOG_INFO("Thingy Pong started.");
    
    perf_init();   
    
    scan_start();
    
    err_code = app_timer_start(m_change_note_timer, APP_TIMER_TICKS(1000), 0);
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
#if (PERF_ENABLE == 1)
        PERF_RUN();
        NRF_LOG_PROCESS();
#else
        idle_state_handle();
#endif
    }
}
