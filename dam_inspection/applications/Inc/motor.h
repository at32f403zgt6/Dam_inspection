/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-24     b       the first version
 */
#ifndef APPLICATIONS_INC_MOTOR_H_
#define APPLICATIONS_INC_MOTOR_H_

#include <rtthread.h>

void motor_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);

#endif /* APPLICATIONS_INC_MOTOR_H_ */
