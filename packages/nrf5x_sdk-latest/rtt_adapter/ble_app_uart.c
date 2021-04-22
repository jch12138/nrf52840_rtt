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

#include <dfs_posix.h>
#include "rtdevice.h"

#define DBG_SECTION_NAME "BLE"
#define DBG_COLOR
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME "Pluse_BH"                        /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL 200 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

extern int16_t channel_data[8];

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                         /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                           /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);               /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] =                                      /**< Universally unique service identifier. */
    {
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

static struct rt_ringbuffer ringbuffer_handler;
static rt_uint8_t ringbuffer[1024] = {0};
static rt_device_t serial;
#define UART_NAME "uart0" /* 串口设备名称 */


int ble_file_transfer_test(int argc, char **argv);

int fd, size;
char file_name[20] = "0000000000000.hex"; //记录数据的文件名
char time_stamp[14] = "0000000000000";    //采样开始时的时间戳

static struct rt_ringbuffer ringbuffer_putc_handler;
static rt_uint8_t ringbuffer_putc[1024] = {0};

/**@brief Function for handling BLE events.
  *
  * @param[in]   p_ble_evt   Bluetooth stack event.
  * @param[in]   p_context   Unused.
  */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        LOG_I("Connected");
        rt_pin_write(14, PIN_LOW);
        //   err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        LOG_I("Disconnected");
        rt_pin_write(14, PIN_HIGH);
        // LED indication will be changed when advertising starts.
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        LOG_I("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

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
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
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
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

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
static void nus_data_handler(ble_nus_evt_t *p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {

        for (int i = 0; i < p_evt->params.rx_data.length; i++)
        {
            rt_kprintf("%c", p_evt->params.rx_data.p_data[i]);
            uart_software_intterrupt();
        }
        //rt_kprintf("\r\n");
        uart_software_intterrupt();

        switch (p_evt->params.rx_data.p_data[0])
        {
        case 'A':
            // strcpy(time_stamp, (char *)(p_evt->params.rx_data.p_data)+1);
            // time_stamp[13] = '\0';
            // printf("start sampling, time: %s \r\n",time_stamp);
            // sprintf(file_name, "%s.hex", time_stamp); 	//将时间戳作为文件名
            // LOG_I("data save in %s",file_name);
            //ads_start();
            break;
        case 'B':
            //ads_stop();
            break;
        case 'C':
            char **t;
            ble_file_transfer_test(1,t);
            //开始采样存储数据接口
            // strcpy(time_stamp, (char *)(p_evt->params.rx_data.p_data)+1);
            // time_stamp[13] = '\0';
            // printf("start sampling, time: %s \r\n",time_stamp);
            // sprintf(file_name, "%s.hex", time_stamp); 	//将时间戳作为文件名
            // fd=  open(file_name, O_WRONLY | O_CREAT);
            // if(fd>0)
            // {
            // rt_kprintf("file creat and open successfully\r\n");
            // }
            // savedate = rt_thread_create("savedate",
            //             save_date, RT_NULL,
            //             2048,
            //             8, 20);
            // if (savedate != RT_NULL)
            //     rt_thread_startup(savedate);
            // LOG_I("start sample...");
            
            break;
        case 'D':

            break;

        case 'R':
            //重启设备
            rt_hw_cpu_reset();
            break;
        default:
            break;
        }
    }
}

/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
  */
static void services_init(void)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;
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
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
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
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = 20000; //FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = 5000;   //NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


int uart_putc_hook(rt_uint8_t *ch)
{
    rt_ringbuffer_putchar(&ringbuffer_putc_handler, *ch);
    return 0;
}


ret_code_t ble_printf(const char *fmt, ...)
{
    ret_code_t ret = NRF_SUCCESS;
    va_list args;
    rt_uint16_t length;
    static char rt_log_buf[255];

    va_start(args, fmt);

    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);

    ret = ble_nus_data_send(&m_nus, rt_log_buf, &length, m_conn_handle);

    va_end(args);

    return ret;
}

void led_init()
{
    rt_pin_mode(14, PIN_MODE_OUTPUT);
    rt_pin_write(14, PIN_HIGH);
}
static int ble_app_init(void)
{
    led_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    LOG_I("stack started.");
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



int ble_uart_send(int argc, char **argv)
{
    char buffer[256];
    uint16_t length;
    char text[15] = "ble uart test\n";

    if (argc == 1)
    {
        // length = rt_strlen(text);
        // ble_nus_data_send(&m_nus, text, &length, m_conn_handle);
//        ble_printf("data:%d,%d,%d,%d,%d,%d,%d,%d\r\n", channel_data[0], channel_data[1], channel_data[2], channel_data[3], channel_data[4], channel_data[5], channel_data[6], channel_data[7]);
    }

    if (argc == 2)
    {
        length = rt_strlen(argv[1]);
        rt_strncpy(buffer, argv[1], length);
        ble_nus_data_send(&m_nus, buffer, &length, m_conn_handle);
    }

    return 0;
}
MSH_CMD_EXPORT(ble_uart_send, ble uart test)

int ble_file_transfer_test(int argc, char **argv)
{
    ret_code_t err_code;

    int ret;
    int fd;            /* 文件指针 */
    struct stat buf;   /* 文件状态 */
    char filename[20]; /* 文件名 */
    uint16_t data_size,send_size;
    uint8_t buffer[256]; /* 发送缓存 */

    if (argc == 1)
    {
        rt_strncpy(filename, "/file.txt", 9);
        filename[9] = '\0';
    }
    if (argc == 2)
    {
        rt_strncpy(filename, argv[1], rt_strlen(argv[1]));
        filename[rt_strlen(argv[1])] = '\0';
    }

    ret = stat(filename, &buf);
    if (ret == 0)
    {
        rt_kprintf("%s file size = %d\n", filename, buf.st_size);
    }
    else
    {
        rt_kprintf("%s not found\n", filename);
        return 0;
    }

    ble_printf("filesize:%dbytes\r\n", buf.st_size);

    fd = open(filename, O_RDONLY);

    if (fd >= 0)
    {
        while (true)
        {
            data_size = read(fd, buffer, m_ble_nus_max_data_len);
            if(data_size == 0)
            {
                break;
            }

            for (int i = 0; i < data_size; i++)
            {
                rt_kprintf("%x ", buffer[i]);
            }

            rt_kprintf("\r\n");

//            buffer[data_size] = '\r';
//            buffer[data_size + 1] = '\n';

//            send_size = data_size +2;
           
            do
            {
                err_code = ble_nus_data_send(&m_nus, buffer, &data_size, m_conn_handle);
            } while (err_code != NRF_SUCCESS);

            
        }

        close(fd);
    }

    ble_printf("file transfer end\r\n");
}
MSH_CMD_EXPORT(ble_file_transfer_test, ble send file test);
