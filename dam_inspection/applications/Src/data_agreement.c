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
#include "shtc3_drv.h"
#include "move_control_fun.h"
#include "motor.h"
#include "jy901s_drv_iic.h"
#include "servo.h"
#include "global_variable.h"
#include "usart.h"
#include <rtthread.h>
#include <rthw.h>
//#include "serial.h"





#define SAMPLE_UART_NAME       "uart1"    /* 串口设备名称 */
/* 查找串口设备 */
static rt_device_t serial;                /* 串口设备句柄 */

//p.s 对全局变量的赋值不要放到函数外


//发送
typedef union
{
    float value;                        //使用其他数据类型，修改此处并在下一行设置相应的字节数即可
    unsigned char data[4];
}U_FloatData;

typedef union
{
    rt_uint16_t value;
    unsigned char data[2];
}U_uint16Data;

/**************************************/

//接收
typedef union
{
    unsigned char rxbuf[4];              //使用其他数据类型，修改此处并在下一行设置相应的字节数即可
    float   value;
}R_FloatData;


typedef union
{
   unsigned char rxbuf[2];
   rt_uint16_t value;
}R_uint16Data;

typedef union
{
   unsigned char rxbuf[2];
   int16_t value;
}R_int16Data;



static void float_pack(int num_data,int length,uint8_t data_ID, uint8_t buf[],U_FloatData conv[])
{
    int i,j,k;

    buf[1]=data_ID;     //数据ID
    buf[2]=length;    //数据长度

  for (j = 0; j < num_data; j++)
  {
     for( i=3;i < 7;i++)
     {
         buf[i+j*4] = conv[j].data[i-3];
     }
  }

 for (k = 1; k <length+3 ; k++)
  {
     buf[length+3]+=buf[k]   ;   //和校验
  }
}

static void uint16_pack(int num_data,int length,uint8_t data_ID, uint8_t buf[],U_uint16Data conv2[])
{
    int i,j,k;

    buf[1]=data_ID;     //数据ID
    buf[2]=length;    //数据长度

  for (j = 0; j < num_data; j++)
  {
     for( i=3;i < 5;i++)
     {
         buf[i+j*2] = conv2[j].data[i-3];
     }
  }
  for (k = 1; k <length+3 ; k++)
  {
     buf[length+3]+=buf[k]   ;   //和校验
  }
}




void data_packup(uint8_t startbit)
{
      uint8_t buf[S_PACKAGE_LEN];

      //用于拆分数据
      U_FloatData conv[10];
      U_uint16Data conv2[10];

      buf[0]=0xFF;             //帧头

             switch(startbit)
                  {
                    case (TX_StartBit_ACC):
                          conv[0].value=accx;
                          conv[1].value=accy;
                          conv[2].value=accz;
                          conv[3].value=angx;
                          conv[4].value=angy;
                          conv[5].value=angz;
                          conv[6].value=pitch;
                          conv[7].value=yaw;
                          conv[8].value=roll;

                          float_pack(9, 36, TX_StartBit_ACC, buf, conv);
                              rt_device_write(serial, 0,buf,40);
                              break;

                    case(TX_StartBit_DEP ):
                            conv[0].value=Depth;
                            conv[1].value=Pressure;

                            float_pack(2, 8, TX_StartBit_DEP, buf, conv);
                             rt_device_write(serial, 0,buf,12);
                             break;

                    case(TX_StartBit_TEM_WET):
                            conv[0].value=temperature;
                            conv[1].value=RH;

                            float_pack(2, 8, TX_StartBit_TEM_WET, buf, conv);
                            rt_device_write(serial, 0, buf, 12);
                            break;

                    case(TX_StartBit_PWM1_8):
                          conv[0].value=accx;
                          conv[1].value=accy;
                          conv[2].value=accz;
                          conv[3].value=angx;
                          conv[4].value=angy;
                          conv[5].value=angz;
                          conv[6].value=pitch;
                          conv[7].value=yaw;

                          uint16_pack(8, 16, TX_StartBit_PWM1_8, buf, conv2);
                          rt_device_write(serial, 0, buf, 20);
                          break;

                    case(TX_Startbit_Cloud_Ang):
//                            conv2[0].value=???;
//                            uint16_pack(8, 16, TX_Startbit_Cloud_Ang, buf, conv2);
//                            rt_device_write(serial, 0, buf, 20);
                            break;

                    default:
                        break;
                  }
}

void Send_allpack()
{
    data_packup(TX_StartBit_ACC);
    data_packup(TX_StartBit_DEP);
    data_packup(TX_StartBit_TEM_WET);
    data_packup(TX_StartBit_PWM1_8);
    data_packup(TX_Startbit_Cloud_Ang);
}

/**********************接收函数部分******************************/

S_handle handle;
S_up up;
S_light light;
S_mode mode;
S_cam cam;


S_pid_incircle pid_incircle;
S_pid_outcircle pid_outcircle;
S_pid_depth pid_depth;



int16_t int16_convert(uint8_t a[],int start,int end)
{
    R_int16Data convert;
    for (int i = start; i <= end; i++)
    {
        convert.rxbuf[i-start] =a[i];
    }

    return convert.value;
}

uint16_t uint16_convert(uint8_t a[],int start,int end)
{
    R_uint16Data convert;
    for (int i = start; i <= end; i++)
    {
        convert.rxbuf[i-start] =a[i];
    }

    return convert.value;
}

float float_convert(uint8_t a[],int start,int end)
{
    R_FloatData convert;
    for ( int i = start; i <= end; i++)
    {
        convert.rxbuf[i-start] =a[i];
    }

    return convert.value;
}





/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;

/* 接收数据回调函数 */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    if (size > 0)
    {
        rt_sem_release(&rx_sem);
    }
    return RT_EOK;
}


//读取一个byte的数据
static uint8_t uart_sample_get_char(void)
{
    uint8_t ch;

    while (rt_device_read(serial, 0, &ch, 1) == 0)
    {
        rt_sem_control(&rx_sem, RT_IPC_CMD_RESET, RT_NULL);
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
    }
    return ch;
}


/* 数据解析线程 */
static void data_analyze(void)
{
    unsigned char ch;
    unsigned char data[250];
    //校验位
    static unsigned char plusbyte=0;
    static unsigned char cnt = 0;

    while (1)
    {
        ch = uart_sample_get_char();
        data[cnt++]=ch;
        /***先读取帧头和数据ID****/
        if(cnt<2) {continue;}

        if (data[0]!=0xFF)
        {
           cnt=0;
           rt_kprintf("数据帧头错误");
           continue;
        }

        /****视数据ID读取数据****/
        switch(data[1])
        {case RX_StartBit_Handle_basic:
                if(cnt<14) {continue;}
                else
                {
                    for (int i = 1;i <3+data[2] ;i++)
                            {
                              plusbyte += data[i];
                            }
                    if(plusbyte == data[13])
                            {
                              handle.go=int16_convert(data,3,4);
                              handle.move=int16_convert(data,5,6);
                              handle.yaw=int16_convert(data,7,8);
                              handle.pitch=int16_convert(data,9,10);
                              handle.roll=int16_convert(data,11,12);
                            }
                            break;
                }
         case RX_StartBit_Handle_up:
             if(cnt<8) {continue;}
                 else
                 {
                     for (int i = 1;i <3+data[2] ;i++)
                             {
                               plusbyte += data[i];
                             }
                     if(plusbyte == data[7])
                             {
                               up.up=int16_convert(data,3,4);
                               up.down=int16_convert(data,5,6);
                             }
                             break;
                 }
         case RX_StartBit_Handle_light:
                      if(cnt<6) {continue;}
                          else
                          {
                              for (int i = 1;i <3+data[2] ;i++)
                                      {
                                        plusbyte += data[i];
                                      }
                              if(plusbyte == data[5])
                                      {
                                        light.on=data[3];
                                        light.off=data[4];
                                      }
                                      break;
                          }
         case RX_StartBit_Handle_func:
                      if(cnt<10) {continue;}
                          else
                          {
                              for (int i = 1;i <3+data[2] ;i++)
                                      {
                                        plusbyte += data[i];
                                      }
                              if(plusbyte == data[9])
                                      {
                                        mode.lockangle=data[3];
                                        mode.autotrip=data[4];
                                        mode.autovertical=data[5];
                                        mode.autorolling=data[6];
                                        mode.defogging=data[7];
                                        mode.unknow=data[8];
                                      }
                                      break;
                          }
         case RX_StartBit_TEST_cam_control:
                       if(cnt<7) {continue;}
                           else
                           {
                               for (int i = 1;i <3+data[2] ;i++)
                                       {
                                         plusbyte += data[i];
                                       }
                               if(plusbyte == data[3+data[2]])
                                       {
                                           cam.cmd=data[3];
                                            if (cam.cmd==0x02){cam.angle=int16_convert(data,4,5);}
                                       }
                                       break;
                           }
         case RX_StartBit_PID:
                       if(cnt<100) {continue;}
                           else
                           {
                               for (int i = 1;i <3+data[2] ;i++)
                                       {
                                         plusbyte += data[i];
                                       }
                               if(plusbyte == data[99])
                                       {
                                        pid_incircle.kp=float_convert(data,3,6);
                                        pid_incircle.ki=float_convert(data,7,10);
                                        pid_incircle.kd=float_convert(data,11,14);
                                        pid_outcircle.p.kp[1]=float_convert(data,15,18);
                                        pid_outcircle.i.ki[1]=float_convert(data,19,22);
                                        pid_outcircle.d.kd[1]=float_convert(data,23,26);
                                        pid_outcircle.p.kp[2]=float_convert(data,27,30);
                                        pid_outcircle.i.ki[2]=float_convert(data,31,34);
                                        pid_outcircle.d.kd[2]=float_convert(data,35,38);
                                        pid_outcircle.p.kp[3]=float_convert(data,39,42);
                                        pid_outcircle.i.ki[3]=float_convert(data,43,46);
                                        pid_outcircle.d.kd[3]=float_convert(data,47,50);
                                        pid_outcircle.p.kp[4]=float_convert(data,51,54);
                                        pid_outcircle.i.ki[4]=float_convert(data,55,58);
                                        pid_outcircle.d.kd[4]=float_convert(data,59,62);
                                        pid_outcircle.p.kp[5]=float_convert(data,63,66);
                                        pid_outcircle.i.ki[5]=float_convert(data,67,70);
                                        pid_outcircle.d.kd[5]=float_convert(data,71,74);
                                        pid_outcircle.p.kp[6]=float_convert(data,75,78);
                                        pid_outcircle.i.ki[6]=float_convert(data,79,82);
                                        pid_outcircle.d.kd[6]=float_convert(data,83,86);
                                        pid_depth.kp=float_convert(data,87,90);
                                        pid_depth.ki=float_convert(data,91,94);
                                        pid_depth.kd=float_convert(data,95,98);
                                       }
                                       break;
                           }

        }
        plusbyte=0;
        cnt=0;
    }
}

static int uart_data_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    char uart_name[20];

    if (argc == 2)
    {
        rt_strncpy(uart_name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);
    }

    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_rx_ind);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", (void (*)(void *parameter))data_analyze, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}























