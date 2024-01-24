/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-24     b       the first version
 */
#include "motor.h"
#include "tim.h"
#include "global_variable.h"
#include <rtthread.h>
#include <rtdevice.h>

/*数据传入记得进行映射,只要映射到motorx变量里
 *
  *  ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
 *
 * 1000对应正向最大转速
 * 1500对应停转
 * 2000对应反向最大转速
 */

uint32_t motor1=1500;
uint32_t motor2=1500;
uint32_t motor3=1500;
uint32_t motor4=1500;
uint32_t motor5=1500;
uint32_t motor6=1500;
uint32_t motor7=1500;
uint32_t motor8=1500;

void thread_motor_entry(void *parameter)
{
    rt_thread_mdelay(3000);
    while(1)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, motor1);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, motor2);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, motor3);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, motor4);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, motor5);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, motor6);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, motor7);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, motor8);
    }
}

void motor_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    MX_TIM3_Init();
    MX_TIM4_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 1500);
    //创建一个动态线程
    static rt_thread_t thread_motor=NULL;
    thread_motor=rt_thread_create("th_motor", thread_motor_entry, NULL, stack_size, priority, tick);
    if(thread_motor!=RT_NULL)
    {
        rt_kprintf("thread_motor create succeed...\n");
        rt_thread_startup(thread_motor);
    }
}
