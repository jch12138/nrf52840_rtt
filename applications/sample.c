#include "rtthread.h"
#include "rtdevice.h"

#include <drv_adc.h>
#define DBG_SECTION_NAME "SAADC"
#define DBG_COLOR
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>

#define SAMPLE_ADC_MODE_SINGLE_ENDED    0   //single-ended mode
#define SAMPLE_ADC_MODE_DIFFERENTIAL    1   //differential mode


#define SAMPLE_ADC_AIN5     5
#define SAMPLE_ADC_AIN6     6
#define SAMPLE_ADC_AIN7     7
#define SAMPLE_ADC_AIN_NC   0   //disable input of AINx


#define SAMPLE_ADC_CHANNEL_5   5
#define SAMPLE_ADC_CHANNEL_6   6
#define SAMPLE_ADC_CHANNEL_7   7
rt_int32_t result1,result2,result3;
rt_adc_device_t adc_dev;

int sample_init(void)// 原void sample_thread_entry(void *parameter)
{
    drv_nrfx_saadc_channel_t channel_config;
     
    
    
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_NAME);
    adc_dev->parent.user_data = &channel_config;
    
    channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
                                                 .pin_p = SAMPLE_ADC_AIN5, 
                                                 .pin_n = SAMPLE_ADC_AIN_NC, 
                                                .channel_num = SAMPLE_ADC_CHANNEL_5};
    rt_adc_enable(adc_dev, channel_config.channel_num);
        
    channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
                                                .pin_p = SAMPLE_ADC_AIN6, 
                                                .pin_n = SAMPLE_ADC_AIN_NC, 
                                                .channel_num = SAMPLE_ADC_CHANNEL_6};
    rt_adc_enable(adc_dev, channel_config.channel_num);
    
    channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
                                                .pin_p = SAMPLE_ADC_AIN7, 
                                                .pin_n = SAMPLE_ADC_AIN_NC, 
                                                .channel_num = SAMPLE_ADC_CHANNEL_7};
    rt_adc_enable(adc_dev, channel_config.channel_num);
    LOG_I("ADC init");
    return RT_EOK;
//     int count = 1; 
//     while(count++)
//     {
//         result1 = rt_adc_read(adc_dev, 5);
// //        rt_kprintf("saadc channel 0 value = %d, ",result);
        
//        result2 = rt_adc_read(adc_dev, 6);
// //       rt_kprintf("saadc channel 1 value = %d \n",result);

//        result3 = rt_adc_read(adc_dev, 7);
// //        rt_kprintf("saadc channel 5 value = %d",result);  
// 		rt_kprintf("data: %d, %d, %d \n",result1,result2,result3);
//         rt_thread_mdelay(20);
//     }
}
INIT_ENV_EXPORT(sample_init);
/*废弃的采样进程启动函数
 */
// int sample_start(void)
// {
//     rt_thread_t tid;

//     tid = rt_thread_create("sample", sample_thread_entry, RT_NULL, 2048, 2, 20);
//     if (tid != RT_NULL)
//     {
//         rt_thread_startup(tid);
//     }
//     else
//     {
//         rt_kprintf("create sample thread err!\n");
//     }
//     return RT_EOK;
// }
