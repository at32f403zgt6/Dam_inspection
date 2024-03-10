/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-14     孙             the first version
 *
 * 这是一个存放全局变量的文件，包含即用
 */
#ifndef APPLICATIONS_INC_GLOBAL_VARIABLE_H_
#define APPLICATIONS_INC_GLOBAL_VARIABLE_H_

#include <rtthread.h>

float temperature;//温湿度传感器获得的温度
float RH;//温湿度传感器获得的湿度

//jy901s数据
float accx,accy,accz,angx,angy,angz,magx,magy,magz,roll,pitch,yaw,temperature;

uint32_t depth;//ms5837压力传感器所获得的深度

rt_uint16_t  current_adc_data[10];   //adc采集的推进器电流数据，由电源板采集后传入主控

#endif /* APPLICATIONS_INC_GLOBAL_VARIABLE_H_ */
