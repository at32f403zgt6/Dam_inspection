/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-16    黄         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "ms5837.h"
#include "global_variable.h"
#include "board.h"

static struct rt_i2c_bus_device *i2c_bus1 = RT_NULL; /* I2C总线设备句柄*/

extern uint32_t depth;

//定义数据手册中需要的参数
/*
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3  温度压力灵敏度系数 TCS
C4  温度系数的压力补偿 TCO
C5  参考温度 T|REF
C6  温度系数的温度 TEMPSENS
*/
uint32_t  Cal_C[7];         //用于存放以上6组数据

double OFF_;     //OFF 实际温度补偿
float Aux;       //计算中间量

uint64_t dT,TEMP;    //dT 实际和参考温度之差        TEMP 实际温度

uint64_t SENS;               //SENS 实际温度灵敏度
uint32_t D1_Pres,D2_Temp;   // 数字压力值,数字温度值
uint32_t TEMP2,T2,OFF2,SENS2;   //温度校验值

uint32_t Pressure;              //气压
uint32_t Atmdsphere_Pressure;//大气压
extern uint32_t Depth;  //深度


/* 写传感器寄存器 */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;
    rt_uint32_t buf_size = 1;

    buf[0] = reg; //cmd
    if (data != RT_NULL)
    {
        buf[1] = data[0];
        buf[2] = data[1];
        buf_size = 3;
    }

    msgs.addr = ms5837_address ;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = buf_size;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}



/* 读传感器寄存器数据 */
static rt_err_t read_reg(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr = ms5837_address+0x01;
    msgs.flags = RT_I2C_RD;
    msgs.buf = buf;
    msgs.len = len;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}





//复位函数
void MS583703BA_RESET(void)
{
    write_reg(i2c_bus1, Command_Reset, RT_NULL);
}

//初始化并获取c1-c6参数
void MS5837_init(void)
{
    uint8_t inth[2];

    uint8_t i;

    MS583703BA_RESET();  // Reset Device  复位MS5837
    rt_thread_mdelay(30);       //复位后延时(数据手册要求的不少于20ms)

  for (i=1;i<=6;i++)
    {
        write_reg(i2c_bus1, Command_Base_Prom_read + (i*2), RT_NULL);//发出6个PROM读入指令
        rt_hw_us_delay(5);//等待

        read_reg(i2c_bus1,2,inth);


    Cal_C[i] = (((uint16_t)inth[0]<< 8) | inth[1]);
    }

  //顺便得到大气压数据
  for(i=0;i<5;i++)
      {
      rt_thread_mdelay(1);
          MS5837_Getdata();   //获取大气压
      Atmdsphere_Pressure+=Pressure;
      }
      Atmdsphere_Pressure=Atmdsphere_Pressure/5;

}


//读取 MS5837转换结果 的函数
unsigned long MS583703BA_getConversion(uint8_t command)
{
    unsigned long conversion = 0;
    uint8_t temp[3];

    write_reg(i2c_bus1, command,RT_NULL);

    rt_thread_mdelay(10);

    write_reg(i2c_bus1,0,RT_NULL);
    read_reg(i2c_bus1,3,temp);

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

//获得最终深度数据
void MS5837_Getdata(void)
{
    D1_Pres= MS583703BA_getConversion(Command_Convert_D1);
    D2_Temp = MS583703BA_getConversion(Command_Convert_D2);
//一阶高低温校正计算
    if(D2_Temp > (((uint32_t)Cal_C[5])*256))
    {
        dT=D2_Temp - (((uint32_t)Cal_C[5])*256);
        TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;
        OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
        SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;
    }
    else
    {
        dT=(((uint32_t)Cal_C[5])*256) - D2_Temp;
        TEMP=2000-dT*((uint32_t)Cal_C[6])/8388608;
        OFF_=(uint32_t)Cal_C[2]*65536-((uint32_t)Cal_C[4]*dT)/128;
        SENS=(uint32_t)Cal_C[1]*32768-((uint32_t)Cal_C[3]*dT)/256;
    }
//二阶高低温校正计算
    if(TEMP<2000)
    {
      Aux = (2000-TEMP)*(2000-TEMP);
        T2 = 3*(dT*dT)/8589934592;
        OFF2 = 3*Aux/2;
        SENS2 = 5*Aux/8;

    }
    else
    {

      Aux = (TEMP-2000)*(TEMP-2000);
      T2 = 2*(dT*dT)/137438953472;
        OFF2 = 1*Aux/16;
        SENS2 = 0;
    }

 //最终结果
    OFF_ = OFF_ - OFF2;
    SENS = SENS - SENS2;
    TEMP=(TEMP-T2)/100;
    Pressure= ((D1_Pres*SENS/2097152-OFF_)/8192)/10;
    Depth=0.983615*(Pressure-Atmdsphere_Pressure);
}

//ms5837线程初始化，成功返回1 否则为0 并打印错误信息
uint8_t Ms5837_init(uint32_t stack_size,
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
        static rt_thread_t thread_ms5837=NULL;
        thread_ms5837=rt_thread_create("th_ms5837", thread_ms5837_entry, NULL, stack_size, priority, tick);
        if(thread_ms5837!=RT_NULL)
        {
            rt_kprintf("thread_ms5837 create succeed...\n");
            rt_thread_startup(thread_ms5837);
        }
        return 1;
    }
}

//ms5837入口函数
void thread_ms5837_entry(void *parameter)
{
    MS5837_init();
    while(1)
    {
    MS5837_Getdata();
    rt_kprintf("Depth : %u \n",depth);
    rt_thread_mdelay(mdelay_tim_ms5837);
    }
}






