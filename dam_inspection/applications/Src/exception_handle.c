/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-28     b       the first version
 */
#include "exception_handle.h"
#include "global_variable.h"
#include <rtthread.h>
#include <rtdevice.h>

/*温湿度*/
extern float temperature;//温湿度传感器获得的温度
extern float RH;//温湿度传感器获得的湿度
extern rt_int8_t shtc3_flag;

void tem_handle()
{


}

void thread_exception_entry(void *parameter)
{
    while(1)
    {
        if(temperature > UPPER_T)   tem_handle();         //温度异常检测

        rt_thread_mdelay(50);
    }
}

uint8_t exception_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    //创建一个动态线程
    static rt_thread_t thread_exception=NULL;
    thread_exception=rt_thread_create("th_exception", thread_exception_entry, NULL, stack_size, priority, tick);
    if(thread_exception!=RT_NULL)
    {
       rt_kprintf("thread_exception create succeed...\n");
       rt_thread_startup(thread_exception);
    }
    return 1;
}
