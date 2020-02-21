/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 * Copyright (c) 2020, Polidea.
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define DEVICE_NAME                     "BleInTheBackground"                    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Polidea"                               /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                6000000                                 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

#define BLE_CUSTOM_SERVICE_UUID         0x1000
#define BLE_CUSTOM_TX_UUID              0x1001
#define BLE_CUSTOM_RX_UUID              0x1002

/// APPLICATION SPECIFIC CODE --------------------------------------------------

// Declarations ----------------------------------------------------------------

APP_TIMER_DEF(tick_timer);

#define ENABLE_PAIRING 0
#define MAX_EVENT_COUNT 10
#define NOTIFICATION_DELAY_MS 500

typedef enum {
    EVENT_TICK,
    EVENT_CONNECTED,
    EVENT_DISCONNECTED,
    EVENT_RX_ENABLED,
    EVENT_SEND_TICKS
} EventType;

typedef struct {
    EventType type;
    union {
        // EVENT_TICK
        uint32_t tick;
        
        // EVENT_CONNECTED       
        struct {
            uint16_t conn_handle;
        } connected;
        
        // EVENT_DISCONNECTED
        struct {
            uint16_t conn_handle;
        } disconnected;
        
        // EVENT_RX_ENABLED
        struct {
            uint16_t conn_handle;
            bool enabled;
        } rx_enabled;
        
        // EVENT_SEND_TICKS
        struct {
            uint16_t conn_handle;
            uint8_t amount;
        } send_ticks;
    };
} Event;

typedef struct {
    uint16_t conn_handle;
    uint8_t  ticks_to_send;
    bool     rx_enabled;
    bool     connected;
} ConnectionContext;

// Attribute table related variables.
static ble_uuid128_t m_base_uuid = { 0xb2, 0x78, 0xff, 0xea, 0x72, 0x2a, 0x43, 0x06, 0xb1, 0x9d, 0x53, 0x3a, 0x00, 0x00, 0x00, 0x00 };
static uint8_t       m_base_uuid_type = BLE_UUID_TYPE_UNKNOWN;
static ble_uuid_t    m_service_uuid = { .uuid = 0x1000, .type = BLE_UUID_TYPE_UNKNOWN };
static uint16_t      m_service_handle = BLE_GATT_HANDLE_INVALID;
static ble_uuid_t    m_tx_uuid = { .uuid = 0x1001, .type = BLE_UUID_TYPE_UNKNOWN };
static uint16_t      m_tx_handle = BLE_GATT_HANDLE_INVALID;
static uint16_t      m_tx_cccd_handle = BLE_GATT_HANDLE_INVALID;
static ble_uuid_t    m_rx_uuid = { .uuid = 0x1002, .type = BLE_UUID_TYPE_UNKNOWN };
static uint16_t      m_rx_handle = BLE_GATT_HANDLE_INVALID;
static uint16_t      m_rx_cccd_handle = BLE_GATT_HANDLE_INVALID;

// Connection context.
static uint32_t m_last_tick;
static ConnectionContext m_connection_ctx[NRF_BLE_GATT_LINK_COUNT];

// Forward declarations.
static void advertising_start(bool erase_bonds);
static void initialize_connections(void);
static void event_handler(void* data, uint16_t size);
static void send_event(Event event);

// Definitions -----------------------------------------------------------------

static void initialize_connections(void) {
    for (int i = 0; i < NRF_BLE_GATT_LINK_COUNT; i++) {
        m_connection_ctx[i].conn_handle = i;
        m_connection_ctx[i].ticks_to_send = 0;
        m_connection_ctx[i].rx_enabled = false;
        m_connection_ctx[i].connected = false;
    }
}

static void tick_timer_handler(void * p_context) {
    Event tickEvent = {
        .type = EVENT_TICK,
        .tick = app_timer_cnt_get()
    };
    send_event(tickEvent);
}

static void event_handler(void* data, uint16_t size) {
    ASSERT(size == sizeof(Event));
    Event event = *(Event*)(data);
        
    switch (event.type) {
        case EVENT_CONNECTED:
            NRF_LOG_INFO("Event connected: handle=%d", event.connected.conn_handle);
            m_connection_ctx[event.connected.conn_handle].connected = true;
            bsp_board_led_on(0);
            break;
        case EVENT_DISCONNECTED:
            NRF_LOG_INFO("Event disconnected: handle=%d", event.disconnected.conn_handle);
            m_connection_ctx[event.disconnected.conn_handle].connected = true;
            m_connection_ctx[event.disconnected.conn_handle].rx_enabled = false;
            bsp_board_led_off(0);
            bsp_board_led_off(1);
            break;
        case EVENT_RX_ENABLED:
            NRF_LOG_INFO("Event rx enabled: handle=%d enabled=%d", event.rx_enabled.conn_handle, event.rx_enabled.enabled);
            m_connection_ctx[event.rx_enabled.conn_handle].rx_enabled = event.rx_enabled.enabled;
            if (event.rx_enabled.enabled) {
                bsp_board_led_on(1);
            } else {
                bsp_board_led_off(1);
            }
            break;
        case EVENT_SEND_TICKS:
            NRF_LOG_INFO("Event send ticks: handle=%d amount=%d", event.send_ticks.conn_handle, event.send_ticks.amount);
            m_connection_ctx[event.send_ticks.conn_handle].ticks_to_send = event.send_ticks.amount;
            break;
        case EVENT_TICK: {
            m_last_tick = event.tick;
            bool packet_sent = false;
            for  (int i = 0; i < NRF_BLE_GATT_LINK_COUNT; i++) {
                if (m_connection_ctx[i].connected && 
                    m_connection_ctx[i].rx_enabled &&
                    m_connection_ctx[i].ticks_to_send > 0) {
                    
                    uint16_t length = sizeof(m_last_tick);
                    ble_gatts_hvx_params_t params = {
                        .handle = m_rx_handle,
                        .type = BLE_GATT_HVX_NOTIFICATION,
                        .offset = 0,
                        .p_len = &length,
                        .p_data = (const char*)(&m_last_tick)
                    };
                    
                    uint32_t result = sd_ble_gatts_hvx(i, &params);
                    if (NRF_SUCCESS == result) {
                        m_connection_ctx[i].ticks_to_send--;
                        packet_sent = true;
                    } else {
                        NRF_LOG_WARNING("Failed to send ticks: %d", result);
                    }
                }
            }
            if (packet_sent) {
                NRF_LOG_INFO("Tick: %u", m_last_tick);
                bsp_board_led_invert(2);
            }
        } break;
        default:
            NRF_LOG_WARNING("Unhandled event type=%d", event.type);
    }
}
    
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
    uint16_t conn_handle = p_evt->conn_handle;
    switch (p_evt->evt_id) {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
            NRF_LOG_INFO("MTU updated: %d", p_evt->params.att_mtu_effective);
            break;
        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
            NRF_LOG_INFO("DLE updated: %d", p_evt->params.data_length);
            break;
    }   
}
 
static void send_event(Event event) {
    APP_ERROR_CHECK(app_sched_event_put(&event, sizeof event, event_handler));
}

/// APPLICATION SPECIFIC CODE --------------------------------------------------


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    APP_ERROR_CHECK(app_timer_create(&tick_timer, APP_TIMER_MODE_REPEATED, tick_timer_handler));
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    // Initialize custom services and characteristic.
    APP_ERROR_CHECK(sd_ble_uuid_vs_add(&m_base_uuid, &m_base_uuid_type));
    m_service_uuid.type = m_base_uuid_type;
    m_rx_uuid.type = m_base_uuid_type;
    m_tx_uuid.type = m_base_uuid_type;
    APP_ERROR_CHECK(sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &m_service_uuid, &m_service_handle));
    
    // TX characteristic definitions.
    const char tx_desc[] = "TX";
    ble_gatts_char_md_t tx_md = {
        .char_props = {
            .write_wo_resp = 1
        },
        .char_ext_props = {
        },
        .p_char_user_desc = tx_desc,
        .char_user_desc_max_size = sizeof(tx_desc) - 1,
        .char_user_desc_size = sizeof(tx_desc) - 1,
        .p_char_pf = NULL,
        .p_user_desc_md = NULL,
        .p_cccd_md = NULL,
        .p_sccd_md = NULL
    };
    
    ble_gatts_attr_md_t tx_attr_md = {
        .read_perm = {
            .sm = 1,
            .lv = 1
        },
        .write_perm = {
            .sm = 1,
            .lv = 1
        },
        .vlen = 1,
        .vloc = BLE_GATTS_VLOC_STACK,
        .rd_auth = 0,
        .wr_auth = 0
    };
    
    ble_gatts_attr_t tx_attr = {
        .p_uuid = &m_tx_uuid,
        .p_attr_md = &tx_attr_md,
        .init_len = 0,
        .init_offs = 0,
        .max_len = BLE_GATTS_VAR_ATTR_LEN_MAX,
        .p_value = NULL
    };
    
    ble_gatts_char_handles_t tx_handles;
    APP_ERROR_CHECK(sd_ble_gatts_characteristic_add(m_service_handle, &tx_md, &tx_attr, &tx_handles));
    m_tx_handle = tx_handles.value_handle;
    m_tx_cccd_handle = tx_handles.cccd_handle;
    
    // RX characteristic definitions
    const char rx_desc[] = "RX";
    ble_gatts_char_md_t rx_md = {
        .char_props = {
            .notify = 1
        },
        .char_ext_props = {
        },
        .p_char_user_desc = rx_desc,
        .char_user_desc_max_size = sizeof(rx_desc) - 1,
        .char_user_desc_size = sizeof(rx_desc) - 1,
        .p_char_pf = NULL,
        .p_user_desc_md = NULL,
        .p_cccd_md = NULL,
        .p_sccd_md = NULL
    };
    
    ble_gatts_attr_md_t rx_attr_md = {
        .read_perm = {
            .sm = 1,
            .lv = 1
        },
        .write_perm = {
            .sm = 1,
            .lv = 1
        },
        .vlen = 1,
        .vloc = BLE_GATTS_VLOC_STACK,
        .rd_auth = 0,
        .wr_auth = 0
    };
    
    ble_gatts_attr_t rx_attr = {
        .p_uuid = &m_rx_uuid,
        .p_attr_md = &rx_attr_md,
        .init_len = 0,
        .init_offs = 0,
        .max_len = BLE_GATTS_VAR_ATTR_LEN_MAX,
        .p_value = NULL
    };
    
    ble_gatts_char_handles_t rx_handles;
    APP_ERROR_CHECK(sd_ble_gatts_characteristic_add(m_service_handle, &rx_md, &rx_attr, &rx_handles));
    m_rx_handle = rx_handles.value_handle;
    m_rx_cccd_handle = rx_handles.cccd_handle;
    
    NRF_LOG_INFO("rx_handle=%u rx_cccd_handle=%u tx_handle=%u", m_rx_handle, m_rx_cccd_handle, m_tx_handle);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    APP_ERROR_CHECK(app_timer_start(tick_timer, APP_TIMER_TICKS(NOTIFICATION_DELAY_MS), NULL));
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;
    
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            bsp_board_led_on(3);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Entering idle state.");
            bsp_board_led_off(3);
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED: {
            NRF_LOG_INFO("Disconnected.");
            Event event = { 
                .type = EVENT_DISCONNECTED, 
                .disconnected = { 
                    .conn_handle = p_ble_evt->evt.gap_evt.conn_handle 
                }
            };
            send_event(event);
        } break;

        case BLE_GAP_EVT_CONNECTED: {
            NRF_LOG_INFO("Connected.");
            Event event = {
                .type = EVENT_CONNECTED,
                .connected = {
                    .conn_handle = p_ble_evt->evt.gap_evt.conn_handle 
                }
            };
            send_event(event);
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
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_WRITE: {
            uint16_t connection_handle = p_ble_evt->evt.gatts_evt.conn_handle;
            const ble_gatts_evt_write_t* write = &p_ble_evt->evt.gatts_evt.params.write;
            NRF_LOG_INFO("GATT Write: off=%d len=%d handle=%d", write->offset, write->len, write->handle);
            
            // Handle CCCD
            if (write->handle == m_rx_cccd_handle && write->len == 2 && write->offset == 0) {
                Event event = {
                    .type = EVENT_RX_ENABLED,
                    .rx_enabled = {
                        .conn_handle = connection_handle,
                        .enabled = write->data[0] & BLE_GATT_HVX_NOTIFICATION
                    }
                };
                send_event(event);
            }
            
            // Handle command
            if (write->handle == m_tx_handle && write->len == 1 && write->offset == 0) {
                Event event = {
                    .type = EVENT_SEND_TICKS,
                    .send_ticks = {
                        .conn_handle = connection_handle,
                        .amount = write->data[0]
                    }
                };
                send_event(event);
            }
        }
        
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
#ifdef ENABLE_PAIRING
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
#endif // ENABLE_PAIRING
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
#ifdef ENABLE_PAIRING
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
#endif // ENABLE_PAIRING
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            for (int i = 0; i < NRF_BLE_GATT_LINK_COUNT; i++) {
                if (!m_connection_ctx[i].connected) continue;   
                err_code = sd_ble_gap_disconnect(m_connection_ctx[i].conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF: {
            bool all_connected = true;
            for (int i = 0; i < NRF_BLE_GATT_LINK_COUNT; i++) {
                if (!m_connection_ctx[i].connected) {
                    all_connected = false;
                    break;
                }
            }
            
            if (!all_connected)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0
        }
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    
    ble_uuid_t m_adv_uuids[] = {
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
        {m_service_uuid.uuid, m_service_uuid.type}
    };
    
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
    init.srdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.srdata.include_appearance      = true;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
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
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    initialize_connections();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    APP_SCHED_INIT(sizeof(Event), MAX_EVENT_COUNT);

    // Start execution.
    NRF_LOG_INFO("Template example started.");
    application_timers_start();

    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
