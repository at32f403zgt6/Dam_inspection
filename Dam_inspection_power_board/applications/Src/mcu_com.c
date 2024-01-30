/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-29     b       the first version
 */

#include "global_variable.h"
#include <rtthread.h>
#include <board.h>

void data_conver(rt_uint8_t *high_data,rt_uint8_t *low_data,rt_uint16_t my_data)
{
    *high_data= (my_data >> 8) & 0xff;
    *low_data= my_data & 0xff;
}

void thread_com_entry(void *parameter)
{
    rt_uint8_t h_data = 0;
    rt_uint8_t l_data = 0;
    while(1)
    {
       rt_kprintf("s");
       for(int i=0;i<10;i++)
       {
       data_conver(&h_data,&l_data,current_adc_data[i]);
       rt_kprintf("%c",h_data);
       rt_kprintf("%c",l_data);
       }
       rt_thread_mdelay(10);
    }
}

uint8_t com_init(uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    //创建一个动态线程
    static rt_thread_t thread_com=NULL;
    thread_com=rt_thread_create("th_com", thread_com_entry, NULL, stack_size, priority, tick);
    if(thread_com!=RT_NULL)
    {
        rt_thread_startup(thread_com);
    }
    return 1;
}
