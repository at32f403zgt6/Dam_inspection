/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-07     huang       the first version
 */
#ifndef APPLICATIONS_INC_DATA_AGREEMENT_H_
#define APPLICATIONS_INC_DATA_AGREEMENT_H_

#include <rtthread.h>
#include <rtdevice.h>



#define TX_StartBit_ACC                     0xA1        //向上位机传输jy901s数据
#define TX_StartBit_DEP                     0xA2        //向上位机传输MS5837数据
#define TX_StartBit_TEM_WET                 0xA3        //向上位机传输shtc3数据
#define TX_StartBit_PWM1_8                  0xA4        //向上位机传输推进器电流数据
#define TX_Startbit_Cloud_Ang               0xA5        //向上位机云台角度数据


/***     用于PID调参          ***/
#define RX_StartBit_PIDP                    0xBB        //下位机接收p(id)参数
#define RX_StartBit_PIDI                    0xBC        //下位机接收p)i(d参数
#define RX_StartBit_PIDD                    0xBD        //下位机接收pi(d)参数



/***       手柄            ***/
#define RX_StartBit_Handle_basic            0xB1        //下位机接收手柄基本运动数据
#define RX_StartBit_Handle_up               0xB2        //下位机接收手柄升潜数据
#define RX_StartBit_Handle_light            0xB3        //下位机接收手柄照明数据
#define RX_StartBit_Handle_func             0xB5        //下位机接收按键功能控制数据




#define RX_StartBit_TEST_cam_control        0xB4        //下位机接收云台控制数据











#endif /* APPLICATIONS_INC_DATA_AGREEMENT_H_ */
