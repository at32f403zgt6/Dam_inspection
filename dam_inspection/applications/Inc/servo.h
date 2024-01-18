/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-16     b         the first version
 *
 * 这是一个存放全局变量的文件，包含即用
 */
#ifndef APPLICATIONS_INC_SERVO_H_
#define APPLICATIONS_INC_SERVO_H_

#include <rtthread.h>

void servo_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);

#endif /* APPLICATIONS_INC_SERVO_H_ */
