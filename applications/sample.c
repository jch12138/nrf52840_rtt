#include "rtthread.h"
#include "rtdevice.h"

#include <drv_adc.h>

#define SAMPLE_ADC_MODE_SINGLE_ENDED    0   //single-ended mode
#define SAMPLE_ADC_MODE_DIFFERENTIAL    1   //differential mode

#define SAMPLE_ADC_AIN1     1
#define SAMPLE_ADC_AIN2     2
#define SAMPLE_ADC_AIN7     7
#define SAMPLE_ADC_AIN_NC   0   //disable input of AINx

#define SAMPLE_ADC_CHANNEL_0   0
#define SAMPLE_ADC_CHANNEL_1   1
#define SAMPLE_ADC_CHANNEL_5   5

rt_uint32_t result;

void sample_thread_entry(void* parameter)
{
    drv_nrfx_saadc_channel_t channel_config;
     
    
    rt_adc_device_t adc_dev;
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_NAME);
    adc_dev->parent.user_data = &channel_config;
    
    // channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
    //                                             .pin_p = SAMPLE_ADC_AIN1, 
    //                                             .pin_n = SAMPLE_ADC_AIN_NC, 
    //                                             .channel_num = SAMPLE_ADC_CHANNEL_0};
    // rt_adc_enable(adc_dev, channel_config.channel_num);
        
    channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
                                                .pin_p = SAMPLE_ADC_AIN2, 
                                                .pin_n = SAMPLE_ADC_AIN_NC, 
                                                .channel_num = SAMPLE_ADC_CHANNEL_1};
    rt_adc_enable(adc_dev, channel_config.channel_num);
    
    // channel_config = (drv_nrfx_saadc_channel_t){.mode = SAMPLE_ADC_MODE_SINGLE_ENDED, 
    //                                             .pin_p = SAMPLE_ADC_AIN7, 
    //                                             .pin_n = SAMPLE_ADC_AIN_NC, 
    //                                             .channel_num = SAMPLE_ADC_CHANNEL_5};
    // rt_adc_enable(adc_dev, channel_config.channel_num);
    
    int count = 1; 
    while(count++)
    {
        // result = rt_adc_read(adc_dev, 0);
        // rt_kprintf("saadc channel 0 value = %d, ",result);
        
        result = rt_adc_read(adc_dev, 1);
        // rt_kprintf("saadc channel 1 value = %d \n",result);
        
        // result = rt_adc_read(adc_dev, 5);
        // rt_kprintf("saadc channel 5 value = %d",result);  
        
        rt_thread_mdelay(20);
    }
}

int sample_start(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("sample", sample_thread_entry, RT_NULL, 2048,RT_THREAD_PRIORITY_MAX - 2, 20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("create sample thread err!\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(sample_start);