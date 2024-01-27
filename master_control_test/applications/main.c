/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-27     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*
 * 使用的gpio引脚 进行每隔0.5s的高低电平变化，并且通过串口三每隔1s打印数据
 */
static rt_thread_t test_gpio=RT_NULL;

void thread_gpio_test_entry(void *parameter)
{
    while(1)
    {
        for(rt_uint8_t i=0;i<80;++i)
        {
            if(i==18||i==56||i==57)//去掉PB2(boot0),PD8,PD9(uart3)
                continue;
            rt_pin_write(i, PIN_HIGH);
        }
        rt_thread_mdelay(500);
        for(rt_uint8_t i=0;i<80;++i)
        {
            if(i==18||i==56||i==57)//去掉PB2(boot0),PD8,PD9(uart3)
                continue;
            rt_pin_write(i, PIN_LOW);
        }
        rt_thread_mdelay(500);
    }
}

int main(void)
{
    //GPIOA.B.C.D.E 0-15
    for(rt_uint8_t i=0;i<80;++i)
    {
        if(i==18||i==56||i==57)//去掉PB2(boot0),PD8,PD9(uart3)
            continue;
        rt_pin_mode(i, PIN_MODE_OUTPUT);
    }
    test_gpio=rt_thread_create("gpio_test", thread_gpio_test_entry, NULL, 512, 20, 10);
    if(test_gpio==RT_NULL)
    {
        LOG_D("thread gpio_test created failed!\n");
    }
    else
    {
        rt_thread_startup(test_gpio);
        LOG_D("thread gpio_test created success!\n");
    }
    while (1)
    {
        LOG_D("Hello RT-Thread!");
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
