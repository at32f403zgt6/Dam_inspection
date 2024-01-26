/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-24       孙       the first version
 */

#include <rtthread.h>
#include <math.h>
#include "move_control_fun.h"
#include "global_variable.h"
#include "motor.h"

/*
 * 注意：
 * 1.rov姿态角度需转换为累计角度，如180,270,1845等不能只用一周的角度-180~180
 * 2.电流需分正负，依推进器确定
 */


//俯仰角Pitch、翻滚角Roll、偏航角Yaw

static rt_thread_t th_pid_out = RT_NULL;
static rt_thread_t th_pid_in = RT_NULL;

//初始化各个pid数据结构体
//为代码简洁方便，用数组表示；分别为俯仰、翻滚、偏航，前后、上下、左右
static struct pid_set pid_out_parameter[6];
//8个推进器
static struct pid_set pid_in_parameter[8];

static rt_int32_t mdelay_tim_out=100;   //外环控制周期
static rt_int32_t mdelay_tim_in=10;    //内环控制周期

/*
 * 外环线程处理函数
 */
void thread_pid_out_entry(void *parameter)
{
    float rov_move_data[6];
    while(1)
    {
        rov_move_data_process(rov_move_data);
        for(rt_uint8_t i=0;i<6;++i)
        {
            //更新控制状态
            pid_out_parameter[i].actual_value=rov_move_data[i];
            //计算目标值与当前值的差值
            pid_out_parameter[i].err=pid_out_parameter[i].target_value-pid_out_parameter[i].actual_value;
            //进行积分运算,采用变积分过程
            if(fabsf(pid_out_parameter[i].err)>pid_out_parameter[i].index_of_err_up)
            {
                pid_out_parameter[i].index=0.0;
                pid_out_parameter[i].integral_value+=0.0;
            }
            else if(fabsf(pid_out_parameter[i].err)<pid_out_parameter[i].index_of_err_down)
            {
                pid_out_parameter[i].index=1.0;
                pid_out_parameter[i].integral_value+=pid_out_parameter[i].err;
            }
            else
            {
                pid_out_parameter[i].index=(pid_out_parameter[i].index_of_err_up-pid_out_parameter[i].err)
                        /(pid_out_parameter[i].index_of_err_up-pid_out_parameter[i].index_of_err_down);
                pid_out_parameter[i].integral_value+=pid_out_parameter[i].err;
            }
            //进行积分限幅
            if(pid_out_parameter[i].integral_value>pid_out_parameter[i].integral_value_limit)
            {
                pid_out_parameter[i].integral_value=pid_out_parameter[i].integral_value_limit;
            }
            else if (pid_out_parameter[i].integral_value<-pid_out_parameter[i].integral_value_limit)
            {
                pid_out_parameter[i].integral_value=-pid_out_parameter[i].integral_value_limit;
            }
            //进行pid运算
            pid_out_parameter[i].out_data=pid_out_parameter[i].parameter.kp*pid_out_parameter[i].err
                    +pid_out_parameter[i].index*pid_out_parameter[i].parameter.ki*pid_out_parameter[i].integral_value
                    +pid_out_parameter[i].parameter.kd*(pid_out_parameter[i].err-pid_out_parameter[i].err_last);
            //对运算结果进行限幅
            if(pid_out_parameter[i].out_data>pid_out_parameter[i].out_data_limit)
            {
                pid_out_parameter[i].out_data=pid_out_parameter[i].out_data_limit;
            }
            else if(pid_out_parameter[i].out_data<-pid_out_parameter[i].out_data_limit)
            {
                pid_out_parameter[i].out_data=-pid_out_parameter[i].out_data_limit;
            }
            //更新上次差值
            pid_out_parameter[i].err_last=pid_out_parameter[i].err;
        }
        //实施计算结果
        rov_move_out_to_in_process();
        rt_thread_mdelay(mdelay_tim_out);
    }
}

/*
 * 内环线程处理函数(推进器电流)
 */
void thread_pid_in_entry(void *parameter)
{
    while(1)
    {
        for(rt_uint8_t i=0;i<8;++i)
        {
            //更新目前的推进器电流值
            pid_in_parameter[i].actual_value=current_adc_data[i];
            //计算目标值与当前值的差值
            pid_in_parameter[i].err=pid_in_parameter[i].target_value-pid_in_parameter[i].actual_value;
            //进行积分运算,采用变积分过程
            if(fabsf(pid_in_parameter[i].err)>pid_in_parameter[i].index_of_err_up)
            {
                pid_in_parameter[i].index=0.0;
                pid_in_parameter[i].integral_value+=0.0;
            }
            else if(fabsf(pid_in_parameter[i].err)<pid_in_parameter[i].index_of_err_down)
            {
                pid_in_parameter[i].index=1.0;
                pid_in_parameter[i].integral_value+=pid_in_parameter[i].err;
            }
            else
            {
                pid_in_parameter[i].index=(pid_in_parameter[i].index_of_err_up-fabsf(pid_in_parameter[i].err))
                        /(pid_in_parameter[i].index_of_err_up-pid_in_parameter[i].index_of_err_down);
                pid_in_parameter[i].integral_value+=pid_in_parameter[i].err;
            }
            //进行积分限幅
            if(pid_in_parameter[i].integral_value>pid_in_parameter[i].integral_value_limit)
            {
                pid_in_parameter[i].integral_value=pid_in_parameter[i].integral_value_limit;
            }
            else if(pid_in_parameter[i].integral_value<-pid_in_parameter[i].integral_value_limit)
            {
                pid_in_parameter[i].integral_value=-pid_in_parameter[i].integral_value_limit;
            }
            //进行pid运算
            pid_in_parameter[i].out_data=pid_in_parameter[i].parameter.kp*pid_in_parameter[i].err+
                    pid_in_parameter[i].index*pid_in_parameter[i].parameter.ki*pid_in_parameter[i].integral_value+
                    pid_in_parameter[i].parameter.kd*(pid_in_parameter[i].err-pid_in_parameter[i].err_last);
            //对运算结果进行限幅
            if(pid_in_parameter[i].out_data>pid_in_parameter[i].out_data_limit)
            {
                pid_in_parameter[i].out_data=pid_in_parameter[i].out_data_limit;
            }
            else if(pid_in_parameter[i].out_data<-pid_in_parameter[i].out_data_limit)
            {
                pid_in_parameter[i].out_data=-pid_in_parameter[i].out_data_limit;
            }
            //更新上次差值
            pid_in_parameter[i].err_last=pid_in_parameter[i].err;
            //将结果更新到驱动器pwm
            motor_set((int16_t)i,pid_in_parameter[i].out_data);
        }
        rt_thread_mdelay(mdelay_tim_in);
    }
}

/*
 * 控制初始化函数,初始化pid执行程序
 * 续传入开辟线程的参数
 * 各线程采用同一优先级，时间片轮转控制
 *基本运动共6线程
 *return 1-success
 */
uint8_t mov_ctrl_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    th_pid_out=rt_thread_create("th_pitch", thread_pid_out_entry, NULL, stack_size, priority, tick);
    th_pid_in=rt_thread_create("th_roll", thread_pid_in_entry, NULL, stack_size, priority, tick);
    //设置输出限幅幅值，积分限幅幅值,变积分指数值误差分界值
    for(rt_uint8_t i=0;i<8;++i)
    {
        pid_in_parameter[i].out_data_limit=1000.0;
        pid_in_parameter[i].integral_value_limit=800.0;
        pid_in_parameter[i].index_of_err_up=400.0;
        pid_in_parameter[i].index_of_err_down=100.0;

        pid_out_parameter[i].out_data_limit=1000.0;
        pid_out_parameter[i].integral_value_limit=800.0;
        pid_out_parameter[i].index_of_err_up=10.0;
        pid_out_parameter[i].index_of_err_down=3.0;
    }
    if(th_pid_out==RT_NULL || th_pid_in==RT_NULL )
    {
        rt_kprintf("move thread create failed...\n");
        return 0;
    }
    else
    {
        rt_thread_startup(th_pid_out);
        rt_thread_startup(th_pid_in);
        rt_kprintf("move thread create succeed...\n");
        return 1;
    }
}

/*
 * 外环（角度、速度环）到内环数据的处理函数
 * 外环输出值为-1000~1000
 * rov推进器假设 水平运动左前顺时针为0.1.2.3；绕轴运动左前顺时针4.5.6.7
 * 需手动测试推进器adc采集的最大电流值，进行比例系数的确定 current_max=1000*k
 * 各推进器安装时正转方向（反推力） 0-右上 1-左上 2-右上 3-左上（俯视） 4.5.6.7-上（正视）
 */
void rov_move_out_to_in_process(void)
{
    float data_out_total_temp[8]={0};
    float k=CURRENT_ADC_MAX/1000.0;//此处需进行实测电流最大值修改
    //俯仰，推进器4.5,6.7 抬头为正
    data_out_total_temp[4]+=pid_out_parameter[0].out_data*k;
    data_out_total_temp[5]+=pid_out_parameter[0].out_data*k;
    data_out_total_temp[6]+=-pid_out_parameter[0].out_data*k;
    data_out_total_temp[7]+=-pid_out_parameter[0].out_data*k;
    //翻滚，推进器4.7,5.6 从后方看，顺时针为正
    data_out_total_temp[4]+=pid_out_parameter[1].out_data*k;
    data_out_total_temp[7]+=pid_out_parameter[1].out_data*k;
    data_out_total_temp[5]+=-pid_out_parameter[1].out_data*k;
    data_out_total_temp[6]+=-pid_out_parameter[1].out_data*k;
    //水平转动，推进器0.1.2.3 从上方看顺时针为正
    data_out_total_temp[0]+=pid_out_parameter[2].out_data*k;
    data_out_total_temp[3]+=pid_out_parameter[2].out_data*k;
    data_out_total_temp[1]+=-pid_out_parameter[2].out_data*k;
    data_out_total_temp[2]+=-pid_out_parameter[2].out_data*k;
    //前进后退，推进器0.1.2.3 向前为正
    data_out_total_temp[0]+=pid_out_parameter[3].out_data*k;
    data_out_total_temp[1]+=pid_out_parameter[3].out_data*k;
    data_out_total_temp[2]+=pid_out_parameter[3].out_data*k;
    data_out_total_temp[3]+=pid_out_parameter[3].out_data*k;
    //上下运动，推进器4.5.6.7 向上为正
    data_out_total_temp[4]+=pid_out_parameter[4].out_data*k;
    data_out_total_temp[5]+=pid_out_parameter[4].out_data*k;
    data_out_total_temp[6]+=pid_out_parameter[4].out_data*k;
    data_out_total_temp[7]+=pid_out_parameter[4].out_data*k;
    //左右平移，推进器0.1.2.3 向右为正
    data_out_total_temp[0]+=-pid_out_parameter[5].out_data*k;
    data_out_total_temp[1]+=pid_out_parameter[5].out_data*k;
    data_out_total_temp[2]+=pid_out_parameter[5].out_data*k;
    data_out_total_temp[3]+=-pid_out_parameter[5].out_data*k;
    for(rt_uint8_t i=0;i<8;++i)
    {
        //对各推进器目标电流值进行限幅
        if(data_out_total_temp[i]>CURRENT_ADC_MAX)
        {
            data_out_total_temp[i]=CURRENT_ADC_MAX;
        }
        else if(data_out_total_temp[i]<-CURRENT_ADC_MAX)
        {
            data_out_total_temp[i]=-CURRENT_ADC_MAX;
        }
        //设置内环各推进器目标电流值
        pid_in_parameter[i].target_value=data_out_total_temp[i];
    }
}

/*
 * 外环控制时间间隔修改函数
 */
void pid_tim_out_chg(rt_int32_t tim)
{
    mdelay_tim_out=tim;
}

/*
 * 内环控制时间间隔修改函数
 */
void pid_tim_in_chg(rt_int32_t tim)
{
    mdelay_tim_in=tim;
}

/*
 * 设置内环输出限幅幅值
 */
void pid_in_out_data_limit_set(float temp)
{
    for(rt_uint8_t i=0;i<8;++i)
    {
        pid_in_parameter[i].out_data_limit=temp;
    }
}

/*
 * 设置内环积分限幅幅值
 */
void pid_in_integral_value_limit_set(float temp)
{
    for(rt_uint8_t i=0;i<8;++i)
    {
        pid_in_parameter[i].integral_value_limit=temp;
    }
}

/*
 * 设置内环变积分指数值误差分界值
 * 参数1-误差大于该值后指数为零  参数2-误差小于该值后指数为1
 */
void pid_in_index_of_err_set(float temp_up,float temp_down)
{
    for(rt_uint8_t i=0;i<8;++i)
    {
        pid_in_parameter[i].index_of_err_up=temp_up;
        pid_in_parameter[i].index_of_err_down=temp_down;
    }
}
//****************************//
/*pid参数修改接口函数*/
/*
 * 改变外环pid
 * 传入[3][6]的数组
 */
//void pid_chg_out_total(float pid_out_temp[3][6])
//{
//    for(uint8_t i=0;i<3;++i)
//    {
//        for(uint8_t j=0;j<6;++j)
//        {
//            pid_out[i][j]=pid_out_temp[i][j];
//        }
//    }
//}

/*
 * 改变内环pid
 * 传入[3][6]的数组
 */
//void pid_chg_in_total(float pid_in_temp[3][6])
//{
//    for(uint8_t i=0;i<3;++i)
//    {
//        for(uint8_t j=0;j<6;++j)
//        {
//            pid_in[i][j]=pid_in_temp[i][j];
//        }
//    }
//}

/*
 * 分别改变内外环俯仰、翻滚、偏航，前后、上下、左右 pid
 */
//void pid_chg_choose(uint8_t )
