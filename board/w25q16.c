#include "rtthread.h"
#include "rtdevice.h"
#include "drv_spi.h"
#include "spi_flash_sfud.h"
#include <dfs_elm.h>
#include <dfs_fs.h>
#include <dfs_posix.h>

#define DBG_TAG "FLASH"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_FLASH

static int rt_hw_spi_flash_init(void)
{
    rt_hw_spi_device_attach("spi0", "spi00", BSP_SPI0_SS_PIN); // spi00 表示挂载在 spi0 总线上的 0 号设备,BSP_SPI0_SS_PIN是片选，这一步就可以将从设备挂在到总线中。

    LOG_I("Flash attach success");

    if (RT_NULL == rt_sfud_flash_probe("W25Q16", "spi00")) //注册块设备，这一步可以将外部flash抽象为系统的块设备
    {
        LOG_E("Flash probe failed");
        return -RT_ERROR;
    };

    LOG_I("Flash probe success");
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);

static int mnt_init(void)
{
    //mkfs("elm","W25Q16");//挂在前需格式化
    if (dfs_mount("W25Q16", "/", "elm", 0, 0) == 0) //挂载文件系统，参数：块设备名称、挂载目录、文件系统类型、读写标志、私有数据0
    {
        LOG_I("Dfs mount success");
    }
    else
    {
        LOG_E("Dfs mount failed");
    }
}
INIT_ENV_EXPORT(mnt_init);

#endif