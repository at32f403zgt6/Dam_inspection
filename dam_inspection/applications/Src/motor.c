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

/*  默认pid_out传入范围为-1000-1000,记得给pid限幅
 */

void motor_set(int16_t num,float duty)
{
    if(num==1)
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (int32_t)(duty/2.0f+1500.0f));
    else if(num==2)
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, (int32_t)(duty/2.0f+1500.0f));
    else if(num==3)
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, (int32_t)(duty/2.0f+1500.0f));
    else if(num==4)
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, (int32_t)(duty/2.0f+1500.0f));
    else if(num==5)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (int32_t)(duty/2.0f+1500.0f));
    else if(num==6)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (int32_t)(duty/2.0f+1500.0f));
    else if(num==7)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (int32_t)(duty/2.0f+1500.0f));
    else if(num==8)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, (int32_t)(duty/2.0f+1500.0f));
}

void motor_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    MX_TIM2_Init();
    MX_TIM3_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 1500);
    rt_thread_mdelay(3000);
}
