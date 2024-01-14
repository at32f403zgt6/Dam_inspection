/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-14     孙       the first version
 */
#include "shtc3_drv.h"
#include "global_variable.h"
#include <rtthread.h>
#include <rtdevice.h>

static struct rt_i2c_bus_device *i2c_bus1 = RT_NULL; /* I2C总线设备句柄*/

rt_int32_t mdelay_tim=500;
extern float temperature;//温湿度传感器获得的温度
extern float RH;//温湿度传感器获得的湿度
/* 写传感器寄存器数据*/
static rt_err_t write_reg(struct rt_i2c_bus_device *i2c_bus,
        rt_uint8_t *data,rt_uint8_t address)
{
    rt_uint8_t buf[2];
    struct rt_i2c_msg msgs;

    buf[0]=data[0];
    buf[1]=data[1];

    msgs.addr = address;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    if(rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else{
        rt_kprintf("Writing Command Error.");
        return -RT_ERROR;
    }
}

/* 读传感器寄存器数据*/
static rt_err_t read_reg(struct rt_i2c_bus_device *i2c_bus,
        rt_uint8_t len, rt_uint8_t *buf,rt_uint8_t address)
{
    struct rt_i2c_msg msgs;

    msgs.addr = address;
    msgs.flags = RT_I2C_RD;
    msgs.buf = buf;
    msgs.len = len;

    if(rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else{
        rt_kprintf("Reading command error.");
        return -RT_ERROR;
    }
}

rt_uint8_t crc_check(rt_uint16_t data,rt_uint8_t crc_data)
{
    uint8_t i,t,temp;
    uint8_t CRC_BYTE= 0xFF;
    temp = (data>>8) & 0xFF;
    for(t = 0;t < 2;t ++)
    {
        CRC_BYTE ^= temp;
        for(i = 0;i < 8;i ++)
        {
            if(CRC_BYTE & 0x80)
            {
                CRC_BYTE <<= 1;
                CRC_BYTE ^= 0x31;
            }else{
                CRC_BYTE <<= 1;
            }
        }
        if(t == 0)
        {
            temp = data & 0xFF;
        }
    }
     if(CRC_BYTE == crc_data)
     {
         temp = 1;
     }else{
         temp = 0;
     }
    return temp;
}

void thread_shtc3_entry(void *parameter)
{
    while(1)
    {
        //根据手册，写寄存器（配置）需间隔240us（max），读取数据须间隔12.1ms（max）
        rt_uint16_t rh_temp,t_temp;
        rt_uint8_t data_temp[6];
        rt_uint8_t temp[2];
        //1.唤醒
        temp[0]=Wakeup_h;
        temp[1]=Wakeup_l;
        write_reg(i2c_bus1,temp , SHTC3_ADDRESS);//唤醒
        //2.测量
        rt_thread_mdelay(1);//配置时间间隔
        temp[0]=Clock_Stretching_Disabled_Read_RH_First_h;
        temp[1]=Clock_Stretching_Disabled_Read_RH_First_l;
        write_reg(i2c_bus1,temp , SHTC3_ADDRESS);//湿度优先
        //3.读出
        rt_thread_mdelay(13);//数据读取时间间隔
        read_reg(i2c_bus1,6,data_temp,SHTC3_ADDRESS);
        if(crc_check(data_temp[0]<<8|data_temp[1],data_temp[2])==1)
        {
            rh_temp=(float)(data_temp[0]<<8|data_temp[1])*100/65536;
            RH=rh_temp;
        }
        else {
            rh_temp=0;
            rt_kprintf("RH error\n");
        }
        if(crc_check(data_temp[3]<<8|data_temp[4],data_temp[5])==1)
        {
            t_temp=(float)(data_temp[3]<<8|data_temp[4])*175/65536-45;
            temperature=t_temp;
        }
        else {
            t_temp=0;
            rt_kprintf("temperature error\n");
        }
        //rt_kprintf("%d.%d%\t%d.%d\n℃",(int)rh_temp/100,(int)rh_temp%100,(int)t_temp/100,(int)t_temp%100);
        //4.睡眠
        temp[0]=sleep_h;
        temp[1]=sleep_l;
        write_reg(i2c_bus1,temp , SHTC3_ADDRESS);

        rt_thread_mdelay(mdelay_tim);
    }
}

/*
 * 温湿度传感器数据读取间隔时间设置
 * 单位：ms
 */
void shtc3_mdelay_tim_change(rt_int32_t tim)
{
    mdelay_tim=tim;
}

/*
 * 温湿度传感器（shtc3）初始化，并分配线程
 * stack_size priority tick 栈，优先级，时间片
 * 使用i2c1总线
 * 成功返回1 否则为0 并打印错误信息
 */
uint8_t shtc3_init(uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    /* 查找I2C总线设备， 获取I2C总线设备句柄*/
    i2c_bus1 = rt_i2c_bus_device_find("i2c1");
    if (i2c_bus1 == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", "i2c1");
        return 0;
    }
    else
    {
        //创建一个动态线程
        static rt_thread_t thread_shtc3=NULL;
        thread_shtc3=rt_thread_create("th_shtc3", thread_shtc3_entry, NULL, stack_size, priority, tick);
        if(thread_shtc3!=RT_NULL)
        {
            rt_kprintf("thread_shtc3 create succeed...\n");
            rt_thread_startup(thread_shtc3);
        }
        return 1;
    }
}
