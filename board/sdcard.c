#include "rtthread.h"
#include "rtdevice.h"
#include "drv_spi.h"
#include "spi_msd.h"
#include <dfs_elm.h>
#include <dfs_fs.h>
#include <dfs_posix.h>

#define DBG_TAG "SDCARD"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define SD_CS_PIN   19

#ifdef BSP_USING_SDCARD

static int rt_hw_spi_sd_init(void)
{
    rt_hw_spi_device_attach("spi0", "spi01", SD_CS_PIN); // spi01 表示挂载在 spi0 总线上的 1 号设备,SD_CS_PIN是片选，这一步就可以将从设备挂在到总线中。

    LOG_I("SD card attach success");

    return msd_init("sd0", "spi01");
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_sd_init);

void sd_mount(void *parameter)
{
    while (1)
    {
        rt_thread_mdelay(500);
        if(rt_device_find("sd0") != RT_NULL)
        {
            if (dfs_mount("sd0", "/", "elm", 0, 0) == RT_EOK)
            {
                LOG_I("SD card mount to '/'");
                break;
            }
            else
            {
                LOG_W("SD card mount to '/' failed!");
            }
        }
    }
}

int sdcard_mount(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("sd_mount", sd_mount, RT_NULL,
                           1024, RT_THREAD_PRIORITY_MAX - 2, 20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("Create sd_mount thread err!");
    }
    return RT_EOK;
}
INIT_APP_EXPORT(sdcard_mount);

#endif