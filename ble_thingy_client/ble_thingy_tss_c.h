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

#ifndef BLE_THINGY_TSS_C_H__
#define BLE_THINGY_TSS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_thingy_tss_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_THINGY_TSS_C_DEF(_name)                                                                        \
static ble_thingy_tss_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_THINGY_TSS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_thingy_tss_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_thingy_tss_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_THINGY_TSS_C_ARRAY_DEF(_name, _cnt)                                                            \
static ble_thingy_tss_c_t _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      BLE_THINGY_TSS_C_BLE_OBSERVER_PRIO,                                                  \
                      ble_thingy_tss_c_on_ble_evt, &_name, _cnt)


#define THINGY_TSS_UUID_BASE            {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x01, 0x68, 0xEF}
#define THINGY_TSS_UUID_SERVICE         0x0500
#define THINGY_TSS_UUID_CONFIG_CHAR     0x0501
#define THINGY_TSS_UUID_SPK_DATA_CHAR   0x0502
#define THINGY_TSS_UUID_SPK_STATUS_CHAR 0x0503
#define THINGY_TSS_UUID_MIC_CHAR        0x0504

/**@brief THINGY_TSS Client event type. */
typedef enum
{
    BLE_THINGY_TSS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the LED Button Service has been discovered at the peer. */
    BLE_THINGY_TSS_C_EVT_SPEAKER_STATUS_NOTIFICATION,
    BLE_THINGY_TSS_C_EVT_MICROPHONE_NOTIFICATION
    
} ble_thingy_tss_c_evt_type_t;

typedef enum {TSS_CONFIG_SPEAKER_MODE_FREQUENCY = 1, TSS_CONFIG_SPEAKER_MODE_PCM, TSS_CONFIG_SPEAKER_MODE_SAMPLE} thingy_tss_config_speaker_mode_t;
typedef enum {TSS_CONFIG_MIC_MODE_ADPCM = 1, TSS_CONFIG_MIC_MODE_SPL} thingy_tss_config_mic_mode_t;
typedef enum {TSS_SPK_STATUS_FINISHED, TSS_SPK_STATUS_BUFFER_WARNING, TSS_SPK_STATUS_BUFFER_READY, 
              TSS_SPK_STATUS_PACKET_DISREGARDED = 0x10, TSS_SPK_STATUS_INVALID_COMMAND}tss_spk_status_values_t;


typedef struct
{
    uint16_t config_handle;
    uint16_t spk_data_handle;
    uint16_t spk_status_handle;
    uint16_t spk_status_cccd_handle;
    uint16_t mic_handle;
    uint16_t mic_cccd_handle;
} thingy_tss_db_t;


typedef PACKED_STRUCT 
{
    uint8_t speaker_mode;
    uint8_t microphone_mode;
}ble_thingy_tss_config_t;

typedef PACKED_STRUCT
{
    uint16_t frequency_hz;
    uint16_t duration_ms;
    uint8_t  volume_percent;
} ble_thingy_tss_spk_data_freq_mode_t;

typedef PACKED_STRUCT
{
    uint8_t samples[273];
} ble_thingy_tss_spk_data_pcm_mode_t;

typedef PACKED_STRUCT
{
    uint8_t sample_id;
} ble_thingy_tss_spk_data_sample_mode_t;

typedef PACKED_STRUCT
{
    union
    {
        ble_thingy_tss_spk_data_freq_mode_t     frequency;
        ble_thingy_tss_spk_data_pcm_mode_t      pcm;
        ble_thingy_tss_spk_data_sample_mode_t   sample;
    }mode;
}ble_thingy_tss_spk_data_t;

typedef PACKED_STRUCT
{
    uint8_t status;
}ble_thingy_tss_spk_status_t;

typedef PACKED_STRUCT
{
    uint16_t adpcm_frame_length;
    uint8_t  adpcm_frame[273];
}ble_thingy_tss_mic_t;

typedef struct
{
    ble_thingy_tss_c_evt_type_t evt_type;   
    uint16_t             conn_handle; 
    union
    {
        ble_thingy_tss_spk_status_t speaker_status;    
        ble_thingy_tss_mic_t        microphone;
        thingy_tss_db_t             peer_db;       
    } params;
} ble_thingy_tss_c_evt_t;

// Forward declaration of the ble_thingy_tss_c_t type.
typedef struct ble_thingy_tss_c_s ble_thingy_tss_c_t;

typedef void (* ble_thingy_tss_c_evt_handler_t) (ble_thingy_tss_c_t * p_ble_thingy_tss_c, ble_thingy_tss_c_evt_t * p_evt);

struct ble_thingy_tss_c_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    thingy_tss_db_t         peer_thingy_tss_db;  /**< Handles related to THINGY_TSS on the peer*/
    ble_thingy_tss_c_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the LED Button service. */
    uint8_t                 uuid_type;    /**< UUID type. */
};

typedef struct
{
    ble_thingy_tss_c_evt_handler_t evt_handler;  /**< Event handler to be called by the LED Button Client module whenever there is an event related to the LED Button Service. */
} ble_thingy_tss_c_init_t;


uint32_t ble_thingy_tss_c_init(ble_thingy_tss_c_t * p_ble_thingy_tss_c, ble_thingy_tss_c_init_t * p_ble_thingy_tss_c_init);


void ble_thingy_tss_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


uint32_t ble_thingy_tss_c_speaker_status_notif_enable(ble_thingy_tss_c_t * p_ble_thingy_tss_c);

uint32_t ble_thingy_tss_c_microphone_notif_enable(ble_thingy_tss_c_t * p_ble_thingy_tss_c);

uint32_t ble_thingy_tss_config_send(ble_thingy_tss_c_t * p_ble_thingy_tss_c, 
                                    thingy_tss_config_speaker_mode_t speaker_mode, 
                                    thingy_tss_config_mic_mode_t mic_mode);
                                    
uint32_t ble_thingy_tss_spk_data_sample_send(ble_thingy_tss_c_t * p_ble_thingy_tss_c, uint8_t sample_id);

uint32_t ble_thingy_tss_spk_data_frequency_send(ble_thingy_tss_c_t * p_ble_thingy_tss_c, uint16_t freq, uint16_t duration_ms, uint8_t volume);
                                    
void ble_thingy_tss_on_db_disc_evt(ble_thingy_tss_c_t * p_ble_thingy_tss_c, const ble_db_discovery_evt_t * p_evt);


uint32_t ble_thingy_tss_c_handles_assign(ble_thingy_tss_c_t *    p_ble_thingy_tss_c,
                                  uint16_t         conn_handle,
                                  const thingy_tss_db_t * p_peer_handles);


#ifdef __cplusplus
}
#endif

#endif // BLE_THINGY_TSS_C_H__

/** @} */
