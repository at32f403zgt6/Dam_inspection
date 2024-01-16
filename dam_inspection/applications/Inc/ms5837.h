/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-16     黄       the first version
 */
#ifndef APPLICATIONS_INC_MS5837_H_
#define APPLICATIONS_INC_MS5837_H_

#include <rtthread.h>

#define ms5837_address 0xEC
#define Command_Reset  0x1E
#define Command_Base_Prom_read 0xA0
#define Command_Convert_D1  0x48
#define Command_Convert_D2  0x58
#define mdelay_tim_ms5837   500

//函数声明
void MS5837_Getdata(void);
unsigned long MS583703BA_getConversion(uint8_t command);
void MS583703BA_RESET(void);
void MS5837_init(void);
/*****************************************/
uint8_t Ms5837_init(uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);
void thread_ms5837_entry(void *parameter);

#endif /* APPLICATIONS_INC_MS5837_H_ */
