#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "ads1293.h"
#include "drv_spi.h"

///* CS Pin */
//#define ADS1293_CS_PIN GET_PIN(A, 3)

//// /* Alarm Pin */
//// #define ADS1293_AlARM_PIN GET_PIN(A, 1)

///* DRDB Pin */
//#define ADS1293_DRDB_PIN GET_PIN(C, 6)

//// /* WCT Pin */
//// #define ADS1293_WCT_PIN GET_PIN(A, 1)

#define ADS1293_DRDY_PIN 2
#define ADS1293_CS_PIN 43

#define ADS_SPI_DEVICE_NAME "spi10" /* SPI 设备名称 */
#define ADS_SPI_BUS_NAME "spi1"     /* SPI 总线名称 */
struct rt_spi_device *spi_dev_ads;  /* SPI 设备句柄 */
int32_t three_channel_data[3];      /* 3个差分通道的数据 */
uint8_t buff[9];


void ads1293_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t inst;
    uint8_t payload[2];

    /* 片选使能 */
    rt_pin_write(ADS1293_CS_PIN, PIN_LOW);

    inst = ADS1293_WRITE_BIT & addr;

    payload[0] = inst;
    payload[1] = value;

    // rt_spi_send(spi_dev_ads, &inst, 1);
    // rt_spi_send(spi_dev_ads, &value, 1);

    rt_spi_send(spi_dev_ads,payload,2);

    /* 释放片选 */
    rt_pin_write(ADS1293_CS_PIN, PIN_HIGH);
}

uint8_t ads1293_read_reg(uint8_t addr)
{
    uint8_t x, inst;

    /* 片选使能 */
    rt_pin_write(ADS1293_CS_PIN, PIN_LOW);

    inst = ADS1293_READ_BIT | addr;
    rt_spi_send_then_recv(spi_dev_ads, &inst,1,&x,1);
    // rt_spi_send(spi_dev_ads, &inst, 1);
    // rt_spi_recv(spi_dev_ads, &x, 1);

    /* 释放片选 */
    rt_pin_write(ADS1293_CS_PIN, PIN_HIGH);

    return x;
}

void ads1293_read_multi_regs(uint8_t addr, uint8_t count,uint8_t* buff)
{
    uint8_t inst,x;

     /* 片选使能 */
    rt_pin_write(ADS1293_CS_PIN, PIN_LOW);

    inst = ADS1293_READ_BIT | addr;
    rt_spi_send_then_recv(spi_dev_ads,&inst,1,buff,count);
    // rt_spi_send(spi_dev_ads, &inst, 1);

    // // for(int i=0;i<count;i++)
    // // {
    // //     rt_spi_recv(spi_dev_ads, buff+i, 1);
    // // }
    // rt_spi_recv(spi_dev_ads,buff,count);
    

    /* 释放片选 */
    rt_pin_write(ADS1293_CS_PIN, PIN_HIGH);
    
}

void ads1293_read_data(void)
{
    // for (int i = 0; i < 9; i++)
    // {
    //     buff[i] = ads1293_read_reg(0x37 + i);
    // }
    ads1293_read_multi_regs(0x37,9,buff);
    // rt_kprintf("%d,%d,%d\n",buff[0],buff[1],buff[2]);
    three_channel_data[0] = (buff[0] * 65536 + buff[1] * 256 + buff[2]);
    three_channel_data[1] = (buff[3] * 65536 + buff[4] * 256 + buff[5]);
    three_channel_data[2] = (buff[6] * 65536 + buff[7] * 256 + buff[8]);
}

void drdb_pin_irq(void)
{
    ads1293_read_data();
}

void ads1293_hw_init(void)
{
    uint8_t id;

    /* 挂载spi设备 */
    rt_hw_spi_device_attach(ADS_SPI_BUS_NAME, ADS_SPI_DEVICE_NAME, ADS1293_CS_PIN);
    spi_dev_ads = (struct rt_spi_device *)rt_device_find(ADS_SPI_DEVICE_NAME);

    /* 配置spi参数 */
    struct rt_spi_configuration cfg;
    cfg.mode = (RT_SPI_MSB | RT_SPI_MASTER | RT_SPI_NO_CS | RT_SPI_MODE_0);
    cfg.data_width = 8;
    cfg.max_hz = 20 * 1000 * 1000;

    rt_spi_configure(spi_dev_ads, &cfg);

    /* 配置普通IO */
    rt_pin_mode(ADS1293_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ADS1293_CS_PIN, PIN_HIGH);

    rt_pin_mode(ADS1293_DRDY_PIN, PIN_MODE_INPUT_PULLUP);
    //rt_pin_attach_irq(ADS1293_DRDB_PIN, PIN_IRQ_MODE_FALLING,drdb_pin_irq,RT_NULL);
    //rt_pin_irq_enable(ADS1293_DRDB_PIN,PIN_IRQ_ENABLE);


    id = ads1293_read_reg(ADS1293_REVID_REG);
    rt_kprintf("id: %d\n",id);
    if (id == 1)
    {
        rt_kprintf("ads1293 init success\n");
    }
    else
    {
        rt_kprintf("ads1293 init failed\n");
    }
}

void ads1293_init_5lead(void)
{
    /* 硬件初始化 */
    ads1293_hw_init();

    /* 配置寄存器 */
    ads1293_write_reg(0x01, 0x11);
    ads1293_write_reg(0x02, 0x19);
    ads1293_write_reg(0x03, 0x2e);
    ads1293_write_reg(0x0a, 0x07);
    ads1293_write_reg(0x0c, 0x04);

    ads1293_write_reg(0x0d, 0x01);
    ads1293_write_reg(0x0e, 0x02);
    ads1293_write_reg(0x0f, 0x03);

    ads1293_write_reg(0x10, 0x01);
    ads1293_write_reg(0x12, 0x04);
    //ADCmax=0xB964F0=0D12,150,000
    //R1 default 4 R2 5  R3 6
    ads1293_write_reg(0x21, 0x02);//
    ads1293_write_reg(0x22, 0x02);
    ads1293_write_reg(0x23, 0x02);
    ads1293_write_reg(0x24, 0x02);

    ads1293_write_reg(0x27, 0x08);
    ads1293_write_reg(0x2f, 0x70);

    ads1293_write_reg(0x00, 0x01);
}





void ads1293_thread_entry(void *parameter)
{
    ads1293_init_5lead();

    while (1)
    {
        //ads1293_read_data();
        // while(rt_pin_read(ADS1293_DRDB_PIN) != PIN_HIGH);
        // while(rt_pin_read(ADS1293_DRDB_PIN) != PIN_LOW);
        ads1293_read_data();
        rt_kprintf("data:%d ,%d ,%d\r\n", three_channel_data[0], three_channel_data[1], three_channel_data[2]);
        rt_thread_mdelay(20);
    }
}

void ads_start(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("ads", ads1293_thread_entry, RT_NULL, 2048, 3, 20);
    rt_thread_startup(tid);
}
MSH_CMD_EXPORT(ads_start, ads test);
INIT_APP_EXPORT(ads_start);