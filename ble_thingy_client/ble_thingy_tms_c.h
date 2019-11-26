/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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

#ifndef BLE_THINGY_TMS_C_H__
#define BLE_THINGY_TMS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_thingy_tms_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_THINGY_TMS_C_DEF(_name)                                                                        \
static ble_thingy_tms_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_THINGY_TMS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_thingy_tms_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_thingy_tms_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_THINGY_TMS_C_ARRAY_DEF(_name, _cnt)                                                            \
static ble_thingy_tms_c_t _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      BLE_THINGY_TMS_C_BLE_OBSERVER_PRIO,                                                  \
                      ble_thingy_tms_c_on_ble_evt, &_name, _cnt)


#define THINGY_TMS_UUID_BASE                    {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x01, 0x68, 0xEF}
#define THINGY_TMS_UUID_SERVICE                 0x0400
#define THINGY_TMS_UUID_CONFIG_CHAR             0x0401
#define THINGY_TMS_UUID_TAP_CHAR                0x0402
#define THINGY_TMS_UUID_ORIENTATION_CHAR        0x0403
#define THINGY_TMS_UUID_QUATERNION_CHAR         0x0404
#define THINGY_TMS_UUID_STEP_COUNTER_CHAR       0x0405
#define THINGY_TMS_UUID_RAW_CHAR                0x0406
#define THINGY_TMS_UUID_EULER_CHAR              0x0407
#define THINGY_TMS_UUID_ROTATION_MATRIX_CHAR    0x0408
#define THINGY_TMS_UUID_HEADING_CHAR            0x0409
#define THINGY_TMS_UUID_GRAVITY_VECTOR_CHAR     0x040A

/**@brief THINGY_TMS Client event type. */
typedef enum
{
    BLE_THINGY_TMS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the LED Button Service has been discovered at the peer. */
    BLE_THINGY_TMS_C_EVT_TAP_NOTIFICATION,   
    BLE_THINGY_TMS_C_EVT_ORIENTATION_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_QUATERNION_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_STEP_COUNTER_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_RAW_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_EULER_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_ROTATION_MATRIX_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_HEADING_NOTIFICATION,
    BLE_THINGY_TMS_C_EVT_GRAVITY_NOTIFICATION,
} ble_thingy_tms_c_evt_type_t;

typedef struct
{
    uint16_t step_counter_int; // In ms, 100ms - 5s
    uint16_t temp_comp_int; // Temperature compensation interval in ms. Range: 100ms - 5s
    uint16_t mag_comp_int; // Magnetometer compensation interval in ms. Range: 100ms - 5s
    uint16_t motion_proc_freq; // Motion processing unit frequency in Hz: Range: 5 - 200 Hz
    uint8_t  wake_on_motion; // Wake on motion, On/Off
} ble_thingy_tms_config_t;

/**@brief Structure containing the tap value received from the peer. */
typedef struct
{
    uint8_t direction; 
    uint8_t count;
} ble_thingy_tms_tap_t;

typedef struct
{
    uint8_t orientation;
} ble_thingy_tms_orientation_t;

typedef struct
{
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} ble_thingy_tms_quaternion_t;

typedef struct
{
    uint32_t steps;
    uint32_t time; // Time in milliseconds
} ble_thingy_tms_step_counter_t;

typedef struct
{
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t compass_x;
    uint16_t compass_y;
    uint16_t compass_z;
} ble_thingy_tms_raw_t;

typedef struct
{
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
} ble_thingy_tms_euler_t;

typedef struct
{
    int16_t matrix[9];
} ble_thingy_tms_rotation_matrix_t;

typedef struct
{
    int32_t heading;
} ble_thingy_tms_heading_t;

typedef struct
{
    float x;
    float y;
    float z;
} ble_thingy_tms_gravity_t;

/**@brief Structure containing the handles related to the LED Button Service found on the peer. */
typedef struct
{
    uint16_t config_handle;
    uint16_t tap_handle; 
    uint16_t tap_cccd_handle;       
    uint16_t orientation_handle;
    uint16_t orientation_cccd_handle;
    uint16_t quaternion_handle;
    uint16_t quaternion_cccd_handle;
    uint16_t step_counter_handle;
    uint16_t step_counter_cccd_handle;
    uint16_t raw_handle;
    uint16_t raw_cccd_handle;
    uint16_t euler_handle;
    uint16_t euler_cccd_handle;
    uint16_t rotation_handle;
    uint16_t rotation_cccd_handle;
    uint16_t heading_handle;
    uint16_t heading_cccd_handle;
    uint16_t gravity_handle;
    uint16_t gravity_cccd_handle;
} thingy_tms_db_t;

/**@brief LED Button Event structure. */
typedef struct
{
    ble_thingy_tms_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
        ble_thingy_tms_tap_t                tap;     
        ble_thingy_tms_orientation_t        orientation;
        ble_thingy_tms_quaternion_t         quaternion;
        ble_thingy_tms_step_counter_t       step_counter;
        ble_thingy_tms_raw_t                raw;
        ble_thingy_tms_euler_t              euler;
        ble_thingy_tms_rotation_matrix_t    rotation_matrix;
        ble_thingy_tms_heading_t            heading;
        ble_thingy_tms_gravity_t            gravity;
        thingy_tms_db_t     peer_db;  
    } params;
} ble_thingy_tms_c_evt_t;

// Forward declaration of the ble_thingy_tms_c_t type.
typedef struct ble_thingy_tms_c_s ble_thingy_tms_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_thingy_tms_c_evt_handler_t) (ble_thingy_tms_c_t * p_ble_thingy_tms_c, ble_thingy_tms_c_evt_t * p_evt);

/**@brief LED Button Client structure. */
struct ble_thingy_tms_c_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    thingy_tms_db_t         peer_thingy_tms_db;  /**< Handles related to THINGY_TMS on the peer*/
    ble_thingy_tms_c_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the LED Button service. */
    uint8_t                 uuid_type;    /**< UUID type. */
};

/**@brief LED Button Client initialization structure. */
typedef struct
{
    ble_thingy_tms_c_evt_handler_t evt_handler;  /**< Event handler to be called by the LED Button Client module whenever there is an event related to the LED Button Service. */
} ble_thingy_tms_c_init_t;


/**@brief Function for initializing the LED Button client module.
 *
 * @details This function will register with the DB Discovery module. There it registers for the
 *          LED Button Service. Doing so will make the DB Discovery module look for the presence
 *          of a LED Button Service instance at the peer when a discovery is started.
 *
 * @param[in] p_ble_thingy_tms_c      Pointer to the LED Button client structure.
 * @param[in] p_ble_thingy_tms_c_init Pointer to the LED Button initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_thingy_tms_c_init(ble_thingy_tms_c_t * p_ble_thingy_tms_c, ble_thingy_tms_c_init_t * p_ble_thingy_tms_c_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function will handle the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the LED Button Client module, then it uses it to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the LED button client structure.
 */
void ble_thingy_tms_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for requesting the peer to start sending notification of the Button
 *        Characteristic.
 *
 * @details This function will enable to notification of the Button at the peer
 *          by writing to the CCCD of the Button Characteristic.
 *
 * @param[in] p_ble_thingy_tms_c Pointer to the LED Button Client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 *          NRF_ERROR_INVALID_STATE if no connection handle has been assigned (@ref ble_thingy_tms_c_handles_assign)
 *          NRF_ERROR_NULL if the given parameter is NULL
 */
uint32_t ble_thingy_tms_c_tap_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_orientation_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_quaternion_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_step_counter_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_raw_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_euler_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_rotation_matrix_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_heading_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);

uint32_t ble_thingy_tms_c_gravity_notif_enable(ble_thingy_tms_c_t * p_ble_thingy_tms_c);


/**@brief Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery module. This
 *          function will handle an event from the database discovery module, and determine if it
 *          relates to the discovery of LED Button service at the peer. If so, it will call the
 *          application's event handler indicating that the LED Button service has been discovered
 *          at the peer. It also populates the event with the service related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_thingy_tms_c Pointer to the LED Button client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_thingy_tms_on_db_disc_evt(ble_thingy_tms_c_t * p_ble_thingy_tms_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a Handles to this instance of thingy_tms_c.
 *
 * @details Call this function when a link has been established with a peer to associate this link
 *          to this instance of the module. This makes it  possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_thingy_tms_c    Pointer to the LED Button client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associate with the given LED Button Client Instance.
 * @param[in] p_peer_handles LED Button Service handles found on the peer (from @ref BLE_THINGY_TMS_C_EVT_DISCOVERY_COMPLETE event).
 *
 */
uint32_t ble_thingy_tms_c_handles_assign(ble_thingy_tms_c_t *    p_ble_thingy_tms_c,
                                  uint16_t         conn_handle,
                                  const thingy_tms_db_t * p_peer_handles);



#ifdef __cplusplus
}
#endif

#endif // BLE_THINGY_TMS_C_H__

/** @} */
