/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-07     huang      the first version
 */
//通讯数据包
#include "data_agreement.h"
#include "ms5837.h"
#include "jy901s_drv.h"
#include "shtc3_drv.h"
#include "move_control_fun.h"
#include "motor.h"
#include "servo.h"
#include "global_variable.h"


static uint8_t  sDGUS_SendBuf[DGUS_PACKAGE_LEN];


void data_packup(uint8_t startbit)
{
      sDGUS_SendBuf[0] = DGUS_FRAME_HEAD1;           //帧头
      sDGUS_SendBuf[1] = DGUS_FRAME_HEAD2;
      sDGUS_SendBuf[2] = 0x06;                       //长度
      sDGUS_SendBuf[3] = DGUS_CMD_W_CTRL;            //指令
      sDGUS_SendBuf[4] = RegAddr;                    //地址
      sDGUS_SendBuf[5] = (uint8_t)(Data>>8);         //数据
      sDGUS_SendBuf[6] = (uint8_t)(Data&0xFF);


      DGUS_CRC16(&sDGUS_SendBuf[3], sDGUS_SendBuf[2] - 2, &sDGUS_CRC_H, &sDGUS_CRC_L);
      sDGUS_SendBuf[7] = sDGUS_CRC_H;                //校验
      sDGUS_SendBuf[8] = sDGUS_CRC_L;


      rt_device_read();
}






