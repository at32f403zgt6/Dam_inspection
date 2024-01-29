/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-24     b       the first version
 */
#include "mcu_communication.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "global_variable.h"

rt_uint8_t aRxBuffer;          //接收中断缓冲
rt_uint8_t Uart6_RxBuff[256] = {0};        //接收缓冲
rt_uint8_t Uart6_Rx_Cnt = 0;       //接收缓冲计数
rt_uint8_t Uart6_RxFlag = 0;

rt_uint8_t get_data[2];
rt_uint8_t com_flag;
rt_uint16_t current_adc_data[10];//adc采集电流数据

int fgetc(FILE * f)
{
  rt_uint8_t ch = 0;
  HAL_UART_Receive(&huart6,&ch, 1, 0xffff);
  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        if(com_flag)
        {
            if(com_flag % 2==1)    get_data[0] = Uart6_RxBuff[0];
            if(com_flag % 2==0)
            {
                get_data[0] = Uart6_RxBuff[0];
                current_adc_data[com_flag-1]=(get_data[0] << 8) | get_data[1];
            }
            com_flag++;
            if(com_flag==18) com_flag=0;
        }
        else
        {
            if (Uart6_RxBuff[0]=='s')
            com_flag=1;
        }
        HAL_UART_Receive_IT(&huart6, (uint8_t *)Uart6_RxBuff, 1);
    }
}

void com_init()
{
    HAL_UART_Receive_IT(&huart6, (uint8_t *)Uart6_RxBuff, 1);
}
