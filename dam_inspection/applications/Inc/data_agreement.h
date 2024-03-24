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

#define RT_DEVICE_FLAG_STREAM       0x040     /* 流模式      */
/* 接收模式参数 */
#define RT_DEVICE_FLAG_INT_RX       0x100     /* 中断接收模式 */
#define RT_DEVICE_FLAG_DMA_RX       0x200     /* DMA 接收模式 */
/* 发送模式参数 */
#define RT_DEVICE_FLAG_INT_TX       0x400     /* 中断发送模式 */
#define RT_DEVICE_FLAG_DMA_TX       0x800     /* DMA 发送模式 */


/******************************只是一条分割线**************************************/


#define S_PACKAGE_LEN                         40          //发送缓冲区长度
#define R_PACKAGE_LEN                         40          //接收缓冲区长度


#define TX_StartBit_ACC                     0xA1        //向上位机传输jy901s数据
#define TX_StartBit_DEP                     0xA2        //向上位机传输MS5837数据
#define TX_StartBit_TEM_WET                 0xA3        //向上位机传输shtc3数据
#define TX_StartBit_PWM1_8                  0xA4        //向上位机传输推进器电流数据
#define TX_Startbit_Cloud_Ang               0xA5        //向上位机传输云台角度数据


/***     用于PID调参          ***/
#define RX_StartBit_PID                     0xB6       //下位机接收pid参数
/***       手柄            ***/
#define RX_StartBit_Handle_basic            0xB1        //下位机接收手柄基本运动数据
#define RX_StartBit_Handle_up               0xB2        //下位机接收手柄升潜数据
#define RX_StartBit_Handle_light            0xB3        //下位机接收手柄照明数据
#define RX_StartBit_Handle_func             0xB5        //下位机接收按键功能控制数据
#define RX_StartBit_TEST_cam_control        0xB4        //下位机接收云台控制数据


/*********************接收到的数据*********************/
typedef struct
{
     int16_t go;
     int16_t move;
     int16_t yaw;
     int16_t pitch;
     int16_t roll;
}S_handle;


typedef struct
{
     int16_t up;
     int16_t down;
}S_up;

typedef struct
{
     uint8_t on;
     uint8_t off;
}S_light;

typedef struct
{
     uint8_t lockangle;
     uint8_t autotrip;
     uint8_t autovertical;
     uint8_t autorolling;
     uint8_t defogging;
     uint8_t unknow;
}S_mode;

typedef struct
{
    uint8_t cmd;
    int16_t angle;
}S_cam;









typedef struct
{
  float kp;
  float ki;
  float kd;
}S_pid_incircle;

typedef struct
{
   struct
   {
     float kp[6];
   }p;
   struct
   {
    float ki[6];
   }i;
   struct
   {
    float kd[6];
   }d;
}S_pid_outcircle;

typedef struct
{
       float kp;
       float ki;
       float kd;
}S_pid_depth;
















#endif /* APPLICATIONS_INC_DATA_AGREEMENT_H_ */

