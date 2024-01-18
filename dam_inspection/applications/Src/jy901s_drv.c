/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-18     孙                 the first version
 */

#include "jy901s_drv.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "global_variable.h"

static struct rt_i2c_bus_device *i2c_bus1 = RT_NULL; /* I2C总线设备句柄*/
extern float accx,accy,accz,angx,angy,angz,magx,magy,magz,roll,pitch,yaw,temperature;
rt_int32_t jy901s_detim=100;//jy901s的数据读取时间间隔，单位ms

/* 写传感器寄存器数据*/
static rt_err_t write_reg(struct rt_i2c_bus_device *i2c_bus ,rt_uint8_t addr ,
        rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];
    //设备地址 -- 寄存器号
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR  ;
    msgs[0].buf = &reg;
    msgs[0].len = 1;
    //打算往寄存器里写什么
    msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if(rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else{
        rt_kprintf("Writing Command Error.");
        return -RT_ERROR;
    }
}

/* 读传感器寄存器数据*/
static rt_err_t read_reg(struct rt_i2c_bus_device *i2c_bus ,rt_uint8_t addr ,
        rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if(rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else{
        rt_kprintf("Reading command error.");
        return -RT_ERROR;
    }
}

/*
 * jy901s读取时间间隔更改
 * 单位ms
 */
void jy901s_detim_change(rt_int32_t tim)
{
    jy901s_detim=tim;
}

void time_handle(rt_uint8_t data_temp[4],rt_uint8_t time[8])
{
    time[0]=data_temp[0]>>4;
    time[1]=data_temp[0]&0x0f;
    time[2]=data_temp[1]>>4;
    time[3]=data_temp[1]&0x0f;
    time[4]=data_temp[2]>>4;
    time[5]=data_temp[2]&0x0f;
    time[6]=data_temp[3]>>4;
    time[7]=data_temp[3]&0x0f;
}

void thread_jy901_entry(void *parameter)
{
    while(1)
    {
        rt_uint8_t data_temp[34];
        rt_uint8_t time[8];
        float ax,ay,az,gx,gy,gz,hx,hy,hz,roll_temp,pitch_temp,yaw_temp,tem;//xyz
        read_reg(i2c_bus1,JY901_ADDR,YYMM,34,data_temp);
        time_handle(data_temp,time);
        ax=(float)(data_temp[8]|(data_temp[9]<<8))/32768*16*9.8;
        ay=(float)(data_temp[10]|(data_temp[11]<<8))/32768*16*9.8;
        az=(float)(data_temp[12]|(data_temp[13]<<8))/32768*16*9.8;
        gx=(float)(data_temp[14]|(data_temp[15]<<8))/32768*2000;
        gy=(float)(data_temp[16]|(data_temp[17]<<8))/32768*2000;
        gz=(float)(data_temp[18]|(data_temp[19]<<8))/32768*2000;
        hx=(float)(data_temp[20]|(data_temp[21]<<8));
        hy=(float)(data_temp[22]|(data_temp[23]<<8));
        hz=(float)(data_temp[24]|(data_temp[25]<<8));
        roll_temp=(float)(data_temp[26]|(data_temp[27]<<8))/32768*180;
        pitch_temp=(float)(data_temp[28]|(data_temp[29]<<8))/32768*180;
        yaw_temp=(float)(data_temp[30]|(data_temp[31]<<8))/32768*180;
        tem=(float)(data_temp[32]|(data_temp[33]<<8))/100;

        //rt_kprintf("20%d.%d.%d %d:%d:%d\n",(int)time[0],(int)time[1],(int)time[2]
        //                                  ,(int)time[3],(int)time[4],(int)time[5]);
        //rt_kprintf("%d,%d,%d\t",(int)ax,(int)ay,(int)az);
        //rt_kprintf("%d,%d,%d\t",(int)gx,(int)gy,(int)gz);
        //rt_kprintf("%d,%d,%d\t",(int)hx,(int)hy,(int)hz);
        //rt_kprintf("data:%d,%d,%d\n",(int)roll,(int)pitch,(int)yaw);
        //rt_kprintf("%d\n",(int)tem);

        accx=ax;accy=ay;accz=az;
        angx=gx;angy=gy;angz=gz;
        magx=hx;magy=hy;magz=hz;
        roll=roll_temp;pitch=pitch_temp;yaw=yaw_temp;
        temperature=tem;

        rt_thread_mdelay(jy901s_detim);
    }
}

/*
 * 加速度计校准
 * 注意！会阻塞一段时间，<4s
 */
void calibration_acc (void)
{
    //1.解锁 2.写入 3.等待校准完成 4.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    rt_uint8_t read_cmd_data[2];//读命令寄存器
    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,CALSW,2,&cmd_data[2]);//加计校准
    //等待校准完成
    while(1)
    {
        rt_thread_mdelay(100);
        read_reg(i2c_bus1,JY901_ADDR,CALSW,2,read_cmd_data);
        if(read_cmd_data[0]==0x00&&read_cmd_data[1]==0x00)
            break;
    }

    //调试信息
//    read_reg(i2c_bus2,JY901_ADDR,CALSW,2,read_cmd_data);
//    rt_kprintf("CALSW:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
//    read_reg(i2c_bus2,JY901_ADDR,SAVE,2,read_cmd_data);
//    rt_kprintf("SAVE:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
//    read_reg(i2c_bus2,JY901_ADDR,0X69,2,read_cmd_data);
//    rt_kprintf("unlovk:%d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
}

/*
 * 恢复出厂设置
 * 阻塞200ms（期间读取数据可能会失败）
 */
void factory_data_reset(void)
{
    //1.解锁 2.写入 3.等待 4.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    rt_uint8_t read_cmd_data[2];//读命令寄存器
    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,SAVE,2,&cmd_data[2]);//恢复出厂设置

    //调试信息
    //read_reg(i2c_bus2,JY901_ADDR,SAVE,2,read_cmd_data);
    //rt_kprintf("SAVE:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

    //等待恢复完成
    while(1)
    {
        rt_thread_mdelay(200);
        read_reg(i2c_bus1,JY901_ADDR,SAVE,2,read_cmd_data);
        if(read_cmd_data[0]==0x00&&read_cmd_data[1]==0x00)
            break;
    }

    //调试信息
    //read_reg(i2c_bus2,JY901_ADDR,SAVE,2,read_cmd_data);
    //rt_kprintf("SAVE:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

}
/*
 * 设置安装方向
 * 0--水平安装  1--垂直安装
 */
void set_direction(rt_uint8_t dir)
{
    //1.解锁 2.写入 3.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    //rt_uint8_t read_cmd_data[2];//读命令寄存器
    cmd_data[2]=dir;
    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,DIRECTION,2,&cmd_data[2]);//恢复出厂设置

    //调试信息
//    read_reg(i2c_bus2,JY901_ADDR,DIRECTION,2,read_cmd_data);
//    rt_kprintf("DIRECTION:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

    //等待完成
    //rt_thread_mdelay(100);
//    while(1)
//    {
//        rt_thread_mdelay(200);
//        read_reg(i2c_bus2,JY901_ADDR,SAVE,2,read_cmd_data);
//        if(read_cmd_data[0]==0x00&&read_cmd_data[1]==0x00)
//            break;
//    }

    //调试信息
//    read_reg(i2c_bus2,JY901_ADDR,DIRECTION,2,read_cmd_data);
//    rt_kprintf("DIRECTION:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
}

/*
 * 休眠设置
 * 重复调用可在休眠与解休眠切换
 * 未实现，原因：休眠后iic不可通讯
 */
void sleep_change(void)
{
    //1.解锁 2.写入 3.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    rt_uint8_t read_cmd_data[2];//读命令寄存器

    //调试信息
//    read_reg(i2c_bus2,JY901_ADDR,SLEEP,2,read_cmd_data);
//    rt_kprintf("DIRECTION:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,SLEEP,2,&cmd_data[2]);//休眠切换

    //调试信息
    read_reg(i2c_bus1,JY901_ADDR,SLEEP,2,read_cmd_data);
    rt_kprintf("DIRECTION:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
}

/*
 * 陀螺仪自动校准
 * 0--启动    1--关闭
 */
void Automatic_gyroscope_calibration(rt_uint8_t choose)
{
    //1.解锁 2.写入 3.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    rt_uint8_t read_cmd_data[2];//读命令寄存器

    //调试信息
    read_reg(i2c_bus1,JY901_ADDR,GYRO,2,read_cmd_data);
    rt_kprintf("GYRO:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

    cmd_data[2]=choose;
    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,GYRO,2,&cmd_data[2]);//设置

    //调试信息
    read_reg(i2c_bus1,JY901_ADDR,GYRO,2,read_cmd_data);
    rt_kprintf("GYRO:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

}

/*
 * LED指示灯开关
 * 0--开     1--关
 */
void led_change(rt_uint8_t change)
{
    //1.解锁 2.写入 3.退出
    rt_uint8_t cmd_data[6]={0x88,0xB5,0X01,0X00,0X00,0X00};//发送命令
    rt_uint8_t read_cmd_data[2];//读命令寄存器

    //调试信息
    read_reg(i2c_bus1,JY901_ADDR,LEDOFF,2,read_cmd_data);
    rt_kprintf("LEDOFF:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);

    cmd_data[2]=change;
    write_reg(i2c_bus1,JY901_ADDR,0X69,2,&cmd_data[0]); //解锁
    write_reg(i2c_bus1,JY901_ADDR,LEDOFF,2,&cmd_data[2]);//设置

    //调试信息
    read_reg(i2c_bus1,JY901_ADDR,LEDOFF,2,read_cmd_data);
    rt_kprintf("LEDOFF:  %d,%d\n",(int)read_cmd_data[0],(int)read_cmd_data[1]);
}

/*
 * 姿态传感器（jy901s）初始化，并分配线程
 * stack_size priority tick 栈，优先级，时间片
 * 使用i2c1总线
 * 成功返回1 否则为0 并打印错误信息
 */
uint8_t jy901s_init(rt_uint32_t stack_size,
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
        calibration_acc();//加速度计自动校准
        //led_change(0);
        //Automatic_gyroscope_calibration(0);
        //set_direction(0);
        //calibration_acc();
        //factory_data_reset();

        //创建一个动态线程
        static rt_thread_t thread_jy901=NULL;
        thread_jy901=rt_thread_create("th_jy901", thread_jy901_entry, NULL, stack_size, priority, tick);
        if(thread_jy901!=RT_NULL)
        {
            rt_kprintf("thread_jy901 create succeed...\n");
            rt_thread_startup(thread_jy901);
        }
        return 1;
    }
}
