/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-23     supperthomas first version
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/serial.h>
#include <drv_adc.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_params.h"

#include "sensorsim.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_nus.h"
#include "ipc/ringbuffer.h"

#define DBG_SECTION_NAME "BLE"
#define DBG_COLOR
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>


#include <dfs_posix.h>
int fd,size;

#define DEVICE_NAME                         "PluseMonitor"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_DURATION                    0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

//#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(20)                   /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

//define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(20, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
//APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
//APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */
//APP_TIMER_DEF(m_rr_interval_timer_id);                              /**< RR interval timer. */
//APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */





extern rt_uint32_t result1,result2,result3;//ADC采样值
extern rt_adc_device_t adc_dev;


static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
   {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
   {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
   {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE},
   {BLE_UUID_NUS_SERVICE,                  BLE_UUID_TYPE_BLE}
};


static struct rt_ringbuffer ringbuffer_handler;
static rt_uint8_t ringbuffer[1024] = {0};
static rt_device_t serial;
#define UART_NAME       "uart0"      /* 串口设备名称 */


static struct rt_ringbuffer ringbuffer_putc_handler;
static rt_uint8_t ringbuffer_putc[1024] = {0};
static void advertising_start(void);

static uint8_t testvalue = 1;
static uint16_t testlen = 1;

static rt_timer_t update;
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
     ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
           // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;

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
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;
    uint32_t ram_start = 0;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.

    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
           // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
           // sleep_mode_enter();
            break;

        default:
            break;
    }
}

/**@brief Struct that contains pointers to the encoded advertising data. */

static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

   // NRF_LOG_INFO("Erase bonds!");

  //  err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
//    if (erase_bonds == true)
//    {
//      //  delete_bonds();
//        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
//    }
//    else
//    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        //APP_ERROR_CHECK(err_code);
//    }
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
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        //bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        //bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
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
static void uart_software_intterrupt(void)
{
    rt_hw_serial_isr((struct rt_serial_device *)serial, RT_SERIAL_EVENT_RX_IND);
};
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//        uint32_t err_code;

        rt_kprintf("Received data from BLE NUS. Writing data on UART.\r\n");
        for(int i = 0; i < p_evt->params.rx_data.length; i++)
        {
            rt_kprintf("%c", p_evt->params.rx_data.p_data[i]);
            rt_ringbuffer_putchar(&ringbuffer_handler, p_evt->params.rx_data.p_data[i]);
            uart_software_intterrupt();
        }
        rt_kprintf("\r\n");
        rt_ringbuffer_putchar(&ringbuffer_handler, '\n');
        uart_software_intterrupt();
        switch (p_evt->params.rx_data.p_data[0])
        {
        case 'A':
            hrs_tf_on();
            LOG_I("start transport sample...");
            break;
        case 'B':
            hrs_tf_off();
            LOG_I("start transport sample...");
            break;
        case 'C':
            //开始采样存储数据接口
            char *filename = '1616466979';
            save_sample(filename);
            LOG_I("start recording sample...");
            break;
        case 'D':
            //结束采样
            LOG_I("start transport sample...");
            break;
        case 'E':
            //查询当前文件
            LOG_I("start transport sample...");
            break;
        
        case 'F':
            //要求传输指定文件
            LOG_I("start transport sample...");
            break;
          
        case 'R':
            //重启设备
            rt_hw_cpu_reset();
            break;      
        default:
            break;
        }
        
        // ble_nus_data_send(&m_nus, (uint8_t *)p_evt->params.rx_data.p_data, &p_evt->params.rx_data.length, m_conn_handle);
    }


}

//移植未成功，用于主要是实现电脑串口->nrf52840->上位机发送数据，不影响使用
static void uart_task(void *param)
{
    uint16_t data_len = 0;
    uint16_t onece_send_data_len = 50;

    // 每隔1秒钟， 查询当前putc ringbuffer是否有数据需要发送
    while (1)
    {
        data_len = rt_ringbuffer_data_len(&ringbuffer_putc_handler);
        if (data_len > 0)
        {
            uint8_t *pdata = (uint8_t *)rt_malloc(data_len);
            if(pdata == NULL)
            {
                rt_kprintf("malloc data failed, malloc len is %d\r\n", data_len);
            }
            else
            {
                // 内存分配成功
                rt_ringbuffer_get(&ringbuffer_putc_handler, pdata, data_len);

                // 将要发送的数据， 分次发送，每次发送onece_send_data_len数据
                uint8_t count = 0;
                uint16_t remainint_data = 0;
                count = data_len / onece_send_data_len;
                remainint_data = data_len % onece_send_data_len;

                int i = 0;
                for (i = 0; i < count; i++)
                {
                    ble_nus_data_send(&m_nus, pdata + i * onece_send_data_len, &onece_send_data_len, m_conn_handle);
                    rt_thread_mdelay(200);
                }
                ble_nus_data_send(&m_nus, pdata + i * onece_send_data_len, &remainint_data, m_conn_handle);
                rt_thread_mdelay(200);

                // 释放内存
                if (pdata != NULL)
                {
                    rt_free(pdata);
                    pdata = NULL;
                }
            }
        }
        rt_thread_mdelay(1000);
    }
    
}
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;    
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_nus_init_t     nus_init;    
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_WRIST;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
   // APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}
/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = 20000;//FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = 5000;//NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
static void ble_app_softdevice(void *param)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    rt_kprintf("\r\nhrs example started.\r\n");
    advertising_start();
 
}

static void timeout(void *param)
{
    result1 = rt_adc_read(adc_dev, 5);
//        rt_kprintf("saadc channel 0 value = %d, ",result); 
    result2 = rt_adc_read(adc_dev, 6);
//       rt_kprintf("saadc channel 1 value = %d \n",result);
    result3 = rt_adc_read(adc_dev, 7);
//        rt_kprintf("saadc channel 5 value = %d",result);  
	//rt_kprintf("data: %d, %d, %d \n",result1,result2,result3);
    //static int i = 0;
    //i++;
    //ble_bas_battery_level_update(&m_bas, i % (MAX_BATTERY_LEVEL - MIN_BATTERY_LEVEL + 1) + MIN_BATTERY_LEVEL, BLE_CONN_HANDLE_ALL);
    ble_hrs_heart_rate_measurement_send(&m_hrs, (rt_uint16_t)result1);
    //ble_nus_data_send(&m_nus, &testvalue, &testlen, m_conn_handle);
}

/**@brief Function for 初始化蓝牙协议栈.
 */
int ble_app_init(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    LOG_I("BLE started.");
    advertising_start();
    //上面是初始化蓝牙协议栈，下面是将本地串口与蓝牙串口连接
    rt_ringbuffer_init(&ringbuffer_handler, ringbuffer, sizeof(ringbuffer));
    rt_ringbuffer_init(&ringbuffer_putc_handler, ringbuffer_putc, sizeof(ringbuffer_putc));
    serial = rt_device_find(UART_NAME);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", UART_NAME);
        return -RT_ERROR;
    }
}
MSH_CMD_EXPORT(ble_app_init, ble app start);
INIT_APP_EXPORT(ble_app_init);



//以下为串口接收到数据后操作的各个函数

/**@brief Function for .
 *开启实时传输
 */

static void timeout(void *param)
{
    result1 = rt_adc_read(adc_dev, 5);
//        rt_kprintf("saadc channel 0 value = %d, ",result); 
    result2 = rt_adc_read(adc_dev, 6);
//       rt_kprintf("saadc channel 1 value = %d \n",result);
    result3 = rt_adc_read(adc_dev, 7);
//        rt_kprintf("saadc channel 5 value = %d",result);  
	//rt_kprintf("data: %d, %d, %d \n",result1,result2,result3);
    //static int i = 0;
    //i++;
    //ble_bas_battery_level_update(&m_bas, i % (MAX_BATTERY_LEVEL - MIN_BATTERY_LEVEL + 1) + MIN_BATTERY_LEVEL, BLE_CONN_HANDLE_ALL);
    ble_hrs_heart_rate_measurement_send(&m_hrs, (rt_uint16_t)result1);
    //ble_nus_data_send(&m_nus, &testvalue, &testlen, m_conn_handle);
}
int hrs_tf_on(void)
{
    update = rt_timer_create("update", timeout, RT_NULL, rt_tick_from_millisecond(20), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    //启动定时器，20ms广播一次数据
    if (update != RT_NULL)
        rt_timer_start(update);
    return RT_EOK;

}

MSH_CMD_EXPORT(hrs_tf_on, transport realtime);
int hrs_tf_off(void)
{
    rt_timer_delete(update);
    return RT_EOK;
}
MSH_CMD_EXPORT(hrs_tf_off, stop transport);

int save_sample(char *filename)
{
    char *fname = '1616161616.txt';
    fname = filename;
    fd=  open(&fname, O_WRONLY | O_CREAT);
    if(fd>0)
    {
        rt_kprintf('file creat and open successfully\r\n');
    }
    while(1)
    {
        result1 = rt_adc_read(adc_dev, 5);
        result2 = rt_adc_read(adc_dev, 6);
        result3 = rt_adc_read(adc_dev, 7);


    }

}
MSH_CMD_EXPORT(save_sample, adcvalue save to file);

int stop_save(void)
{

}
MSH_CMD_EXPORT(stop_save, stop_save save adcvalue);

// int ble_app_hrs(void)
// {
//     rt_ringbuffer_init(&ringbuffer_handler, ringbuffer, sizeof(ringbuffer));
//     rt_ringbuffer_init(&ringbuffer_putc_handler, ringbuffer_putc, sizeof(ringbuffer_putc));
//     serial = rt_device_find(UART_NAME);
//     if (!serial)
//     {
//         rt_kprintf("find %s failed!\n", UART_NAME);
//         return -RT_ERROR;
//     }
//     static rt_thread_t tid1 = RT_NULL;
//     tid1 = rt_thread_create("softdevice",
//                         ble_app_softdevice, RT_NULL,
//                         4096,
//                         22, 5);
//     if (tid1 != RT_NULL)
//         rt_thread_startup(tid1);
//     rt_timer_t update = rt_timer_create("update", timeout, RT_NULL, rt_tick_from_millisecond(20), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
//     //启动定时器，20ms广播一次数据
//     if (update != RT_NULL)
//         rt_timer_start(update);
//     return RT_EOK;
//     // tid1 = rt_thread_create("serial_task",
//     //                     uart_task, RT_NULL,
//     //                     1024,
//     //                     22, 5);
//     // if (tid1 != RT_NULL)
//     // {
//     //     rt_thread_startup(tid1);
//     // }
//     //
// }
// MSH_CMD_EXPORT(ble_app_hrs, ble app heart rate service);
// INIT_APP_EXPORT(ble_app_hrs);