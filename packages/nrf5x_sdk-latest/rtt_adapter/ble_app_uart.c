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

#include "ble_nus.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_params.h"
#include "ble_advertising.h"
#include "ipc/ringbuffer.h"

#define DBG_SECTION_NAME "BLE"
#define DBG_COLOR
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>


#include <dfs_posix.h>


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Pluse"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */



//#define NRF_LOG_ERROR(...)                     rt_kprintf(__VA_ARGS__)
//#define NRF_LOG_WARNING(...)                   rt_kprintf( __VA_ARGS__)
//#define NRF_LOG_INFO(...)                      rt_kprintf( __VA_ARGS__)
//#define NRF_LOG_DEBUG(...)                     rt_kprintf( __VA_ARGS__)

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static struct rt_ringbuffer ringbuffer_handler;
static rt_uint8_t ringbuffer[1024] = {0};
static rt_device_t serial;
#define UART_NAME       "uart0"      /* 串口设备名称 */

extern rt_uint32_t result1,result2,result3;//ADC采样值
extern rt_adc_device_t adc_dev;
rt_uint8_t  result[6];

int fd,size;
char file_name[20] = "0000000000000.hex"; 	//记录数据的文件名
char time_stamp[14]= "0000000000000"; 					    //采样开始时的时间戳

static struct rt_ringbuffer ringbuffer_putc_handler;
static rt_uint8_t ringbuffer_putc[1024] = {0};
static rt_thread_t savedate = NULL;
static rt_thread_t sendfile = NULL;
static rt_timer_t update;

/**@brief Function for saveadcvalue.
 *

 */
static void save_date(void *param)
{
        result1 = rt_adc_read(adc_dev, 5);
        result2 = rt_adc_read(adc_dev, 6);
        result3 = rt_adc_read(adc_dev, 7);
        rt_kprintf("%d %d %d \r\n",result1,result2,result3);
        result[0] = (result1>>8)&0xff;
        result[1] =  result1&0xff;
        result[2] = (result2>>8)&0xff;
        result[3] =  result2&0xff;
        result[4] = (result3>>8)&0xff;
        result[5] =  result3&0xff;
        rt_kprintf("%d %d %d %d %d %d \r\n",result[0],result[1],result[2],result[3],result[4],result[5]);
        write(fd,result,6);
        rt_kprintf("writesuccesss\r\n");

}
/**@brief Function for transfor adcvalue history.
 *
 */
static void rsdate(void *param)
{
        char *buffer;
        read(fd,buffer,6);
        uint16_t len = 6;
        ble_nus_data_send(&m_nus, buffer,&len , m_conn_handle);
}
static void send_file_test(void *param)
{
        while(1){
        result[0] = 0x01;
        result[1] = 0x02;
        result[2] = 0x03;
        result[3] = 0x04;
        result[4] = 0x05;
        result[5] = 0x06;
        uint16_t len = 6;
        ble_nus_data_send(&m_nus, result,&len , m_conn_handle);
        rt_thread_mdelay(20);
        }
}
static void read_fname(void *param)
{
    DIR *dirp;
    struct dirent *dp;
    /* 打开根目录 */
    rt_kprintf("the directory is:\n");
    dirp = opendir("/");
    for (dp = readdir(dirp); dp != RT_NULL; dp = readdir(dirp))
    {          
        rt_kprintf("%s\n", dp->d_name);
    }
}
MSH_CMD_EXPORT(read_fname, readfilename)
/**@brief Function for transforadcvalue ontime.
 *
 */
static void timeout(void *param)
{
    result1 = rt_adc_read(adc_dev, 5);
//        rt_kprintf("saadc channel 0 value = %d, ",result); 
    result2 = rt_adc_read(adc_dev, 6);
//       rt_kprintf("saadc channel 1 value = %d \n",result);
    result3 = rt_adc_read(adc_dev, 7);
//        rt_kprintf("saadc channel 5 value = %d",result);  
    result[0] = (result1>>8)&0xff;
    result[1] =  result1&0xff;
    result[2] = (result2>>8)&0xff;
    result[3] =  result2&0xff;
    result[4] = (result3>>8)&0xff;
    result[5] =  result3&0xff;
	rt_kprintf("data: %d, %d, %d \n",result1,result2,result3);
    uint16_t len = 6;
    //·ble_nus_data_send(&m_nus, result,&len , m_conn_handle);
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 *
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
         //   err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
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

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
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
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
          //  err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
          //  sleep_mode_enter();
            break;
        default:
            break;
    }
}
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
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
    
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
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

static void uart_software_intterrupt(void)
{
    rt_hw_serial_isr((struct rt_serial_device *)serial, RT_SERIAL_EVENT_RX_IND);
};

int uart_getc_hook(rt_uint8_t *ch)
{
    if (0 == rt_ringbuffer_getchar(&ringbuffer_handler, ch))
    {
        return -1;
    }
    return 0;
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//        uint32_t err_code;

        rt_kprintf("Received data from BLE NUS. Writing data on UART.\r\n");
        for(int i = 0; i < p_evt->params.rx_data.length; i++)
        {
            rt_kprintf("%c", p_evt->params.rx_data.p_data[i]);
            //rt_ringbuffer_putchar(&ringbuffer_handler, p_evt->params.rx_data.p_data[i]);
            uart_software_intterrupt();
        }
        rt_kprintf("\r\n");
        //rt_ringbuffer_putchar(&ringbuffer_handler, '\n');
        uart_software_intterrupt();
        switch (p_evt->params.rx_data.p_data[0])
        {
        case 'A':
            hrs_tf_on();
            LOG_I("start transport sample...");
            break;
        case 'B':
            hrs_tf_off();
            LOG_I("stopls transport sample...");
            break;
        case 'C':
            //开始采样存储数据接口
			strcpy(time_stamp, (char *)(p_evt->params.rx_data.p_data)+1);
			time_stamp[13] = '\0';
			printf("start sampling, time: %s \r\n",time_stamp);
			sprintf(file_name, "%s.hex", time_stamp); 	//将时间戳作为文件名
            fd=  open(file_name, O_WRONLY | O_CREAT);
            if(fd>0)
            {
            rt_kprintf("file creat and open successfully\r\n");
            }
            savedate = rt_thread_create("savedate",
                        save_date, RT_NULL,
                        2048,
                        8, 20);
            if (savedate != RT_NULL)
                rt_thread_startup(savedate);
            LOG_I("start sample...");

            break;
        case 'D':
            //结束采样
            rt_thread_delete(savedate);
            if(fd>0)
                close(fd);

            LOG_I("stop sample...");
            break;
        case 'E':
            //查询当前文件
            LOG_I("start transport sample...");
            break;
        
        case 'F':
            //要求传输指定文件
            LOG_I("start transport sample...");
            break;
        case 'T':
            sendfile = savedate = rt_thread_create("sendfile",
                        send_file_test, RT_NULL,
                        2048,
                        8, 20);
            if (sendfile != RT_NULL)
                rt_thread_startup(sendfile);
            LOG_I("file transport test...");
            break;
        case 'S':
             rt_thread_delete(sendfile);
             LOG_I("file transport test stop");
            break;
        case 'R':
            //重启设备
            rt_hw_cpu_reset();
            break;      
        default:
            break;
        }
        
        //ble_nus_data_send(&m_nus, (uint8_t *)p_evt->params.rx_data.p_data, &p_evt->params.rx_data.length, m_conn_handle);
    }


}


/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
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
static int ble_app_init(void)
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

int uart_putc_hook(rt_uint8_t *ch)
{
    rt_ringbuffer_putchar(&ringbuffer_putc_handler, *ch);
    return 0;
}

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

int savetest(void)
{

    fd=  open("\hello.hex", O_WRONLY | O_CREAT);
    if(fd>0)
    {
        rt_kprintf("file creat and open successfully\r\n");
    }
    int32_t timestart=rt_tick_get();
    int count = 10000;
    while(count)
    {
        count --;
        result1 = rt_adc_read(adc_dev, 5);
        result2 = rt_adc_read(adc_dev, 6);
        result3 = rt_adc_read(adc_dev, 7);
        //rt_kprintf("%d %d %d \r\n",result1,result2,result3);
        result[0] = (result1>>8)&0xff;
        result[1] =  result1&0xff;
        result[2] = (result2>>8)&0xff;
        result[3] =  result2&0xff;
        result[4] = (result3>>8)&0xff;
        result[5] =  result3&0xff;
        //rt_kprintf("%d %d %d %d %d %d \r\n",result[0],result[1],result[2],result[3],result[4],result[5]);
        write(fd,result,6);
        rt_thread_mdelay(20);
        

    }
    int32_t timestop=rt_tick_get();
    close(fd);
    rt_kprintf("usetime = %d\r\n",timestop-timestart);

}
MSH_CMD_EXPORT(savetest, test adc save to file);
int readtest()
{   int size;
    char buffer[40];
    fd=  open("\hello.hex", O_RDONLY);
    size = read(fd, buffer, sizeof(buffer));
    for(int i=0;i<=30;i++)
    {
        rt_kprintf("%d ",buffer[i]);
    }
    rt_kprintf(" \r\nsize=%d",size);
}
MSH_CMD_EXPORT(readtest, test adc read to file);
int stop_save(void)
{

}
MSH_CMD_EXPORT(stop_save, stop_save save adcvalue);

int ble_uart_send(int argc, char **argv)
{
    char buffer[256];
    uint16_t length; 
    char text[15] = "ble uart test\n";

    if(argc==1)
    {
        length = rt_strlen(text);
        ble_nus_data_send(&m_nus, text, &length, m_conn_handle);
    }


    if(argc==2)
    {
        length = rt_strlen(argv[1]);
        rt_strncpy(buffer, argv[1], length);
        ble_nus_data_send(&m_nus, buffer, &length, m_conn_handle);
    }



    return 0;
}
MSH_CMD_EXPORT(ble_uart_send, ble uart test)

// int ble_app_uart(void)
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

//     tid1 = rt_thread_create("serial_task",
//                         uart_task, RT_NULL,
//                         1024,
//                         22, 5);
//     if (tid1 != RT_NULL)
//     {
//         rt_thread_startup(tid1);
//     }
//     return RT_EOK;
// }
// MSH_CMD_EXPORT(ble_app_uart, ble app uart);
