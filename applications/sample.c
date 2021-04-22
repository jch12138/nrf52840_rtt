#include <drv_adc.h>

#define SAMPLE_ADC_MODE_SINGLE_ENDED    0   //single-ended mode
#define SAMPLE_ADC_MODE_DIFFERENTIAL    1   //differential mode

#define SAMPLE_ADC_AIN5     5
#define SAMPLE_ADC_AIN6     6
#define SAMPLE_ADC_AIN7     7
#define SAMPLE_ADC_AIN_NC   0   //disable input of AINx

#define SAMPLE_ADC_CHANNEL_5   5
#define SAMPLE_ADC_CHANNEL_6   6
#define SAMPLE_ADC_CHANNEL_7   7

#include "app_error.h"
extern ret_code_t ble_printf(const char *fmt, ...);

 int low_pass_filter_channel_1(  int com )
{
    static  int iLastData;    //上一次值
     int iData;               //本次计算值
    float dPower = 0.1;               //滤波系数
    iData = ( com * dPower ) + ( 1 - dPower ) * iLastData; //计算
    iLastData = iData;                                     //存贮本次数据
    return iData;                                         //返回数据
}

 int low_pass_filter_channel_2(  int com )
{
    static  int iLastData;    //上一次值
     int iData;               //本次计算值
    float dPower = 0.2;               //滤波系数
    iData = ( com * dPower ) + ( 1 - dPower ) * iLastData; //计算
    iLastData = iData;                                     //存贮本次数据
    return iData;                                         //返回数据
}
 int low_pass_filter_channel_3(  int com )
{
    static  int iLastData;    //上一次值
     int iData;               //本次计算值
    float dPower = 0.2;               //滤波系数
    iData = ( com * dPower ) + ( 1 - dPower ) * iLastData; //计算
    iLastData = iData;                                     //存贮本次数据
    return iData;                                         //返回数据
}
void saadc_sample(void)
{
    drv_nrfx_saadc_channel_t channel_config;
    rt_uint32_t result[3];
    int channel[3];
    rt_adc_device_t adc_dev;
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

    //int count = 1;
    while(1)
    {
        result[0] = rt_adc_read(adc_dev, SAMPLE_ADC_CHANNEL_5);
        // rt_kprintf("saadc channel 0 value = %d, ",result);

        result[1] = rt_adc_read(adc_dev, SAMPLE_ADC_CHANNEL_6);
        // rt_kprintf("saadc channel 1 value = %d, ",result);

        result[2] = rt_adc_read(adc_dev, SAMPLE_ADC_CHANNEL_7);
        // rt_kprintf("saadc channel 5 value = %d",result);
        channel[0] = (int)(1000*low_pass_filter_channel_1(result[0])/4096);
        channel[1] = (int)(1000*low_pass_filter_channel_2(result[1])/4096);
        channel[2] = (int)(1000*low_pass_filter_channel_3(result[2])/4096);
        //  channel[1] = low_pass_filter_channel_1(result[1]);
        //   channel[2] = low_pass_filter_channel_1(result[2]);

//        rt_kprintf("data:%d,%d,%d\n",channel[0],channel[1],channel[2]);
				ble_printf("data:%d,%d,%d\r\n",channel[0],channel[1],channel[2]);
                // ble_printf("data:%d,%d,%d\r\n",result[0],result[1],result[2]);
        rt_thread_mdelay(20);
    }
}
//MSH_CMD_EXPORT(saadc_sample, saadc sample);
//INIT_APP_EXPORT(saadc_sample);

int sample_start(void)
{
	rt_thread_t tid = rt_thread_create("sample",saadc_sample,RT_NULL,2048,3,10);
	rt_thread_startup(tid);
}
INIT_APP_EXPORT(sample_start);