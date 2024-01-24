/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-16     b       the first version
 */
#include "servo.h"
#include "tim.h"
#include "global_variable.h"
#include <rtthread.h>
#include <rtdevice.h>

/*
 * 当手柄接收到信号或上位机进行指令，将下列对应标志位置1或-1
 * 1->一关节舵机
 * 2->二关节舵机
 * 3->三关节舵机
 * */
uint8_t fl_1,fl_2,fl_3;

void thread_servo_entry(void *parameter)
{
    uint32_t roll=500;
    uint32_t yaw=500;
    uint32_t pitch=500;
    while(1)
    {
       if(!fl_1)
       {
          yaw+=fl_1;
          yaw = yaw > 1000 ? 1000 : yaw;
          yaw = yaw < 0 ? 0 : yaw;
          __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, yaw);
          rt_thread_mdelay(10);
       }
       if(!fl_2)
       {
          roll+=fl_2;
          roll = roll > 1000 ? 1000 : roll;
          roll = roll < 0 ? 0 : roll;
          __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, roll);
          rt_thread_mdelay(10);
       }
       if(!fl_3)
       {
          pitch+=fl_3;
          pitch = pitch > 1000 ? 1000 : pitch;
          pitch = pitch < 0 ? 0 : pitch;
          __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pitch);
          rt_thread_mdelay(10);
       }
    }
}

void servo_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    MX_TIM2_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 500);
    //创建一个动态线程
    static rt_thread_t thread_servo=NULL;
    thread_servo=rt_thread_create("th_servo", thread_servo_entry, NULL, stack_size, priority, tick);
    if(thread_servo!=RT_NULL)
    {
        rt_kprintf("thread_servo create succeed...\n");
        rt_thread_startup(thread_servo);
    }
}
