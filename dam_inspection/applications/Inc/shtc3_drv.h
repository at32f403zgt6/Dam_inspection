/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-14     孙       the first version
 */
#ifndef APPLICATIONS_INC_SHTC3_DRV_H_
#define APPLICATIONS_INC_SHTC3_DRV_H_

#include <rtthread.h>
uint8_t shtc3_init(uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);
void shtc3_mdelay_tim_change(rt_int32_t tim);

#define SHTC3_ADDRESS 0x70 /* 从机地址*/
#define Wakeup_h 0x35
#define Wakeup_l 0x17
#define Clock_Stretching_Disabled_Read_RH_First_h 0x58
#define Clock_Stretching_Disabled_Read_RH_First_l 0xE0
#define sleep_h 0xB0
#define sleep_l 0x98

#endif /* APPLICATIONS_INC_SHTC3_DRV_H_ */
