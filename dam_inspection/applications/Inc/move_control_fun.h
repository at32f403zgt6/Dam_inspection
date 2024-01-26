/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-24       孙       the first version
 */

#ifndef APPLICATIONS_INC_MOVE_CONTROL_FUN_H_
#define APPLICATIONS_INC_MOVE_CONTROL_FUN_H_

#define CURRENT_ADC_MAX 4096.0

struct pid_rov
{
    float kp;
    float ki;
    float kd;
};

struct pid_set
{
    float target_value;         //预期值
    float actual_value;         //实际值
    float err;                  //偏差值
    float err_last;             //上一次偏差值
    float integral_value;       //积分值
    float integral_value_limit; //积分值上限
    struct pid_rov parameter;   //pid参数 kp、ki、kd
    float out_data;             //输出值
    float out_data_limit;       //输出值限幅（正）
    float index;                //变积分指数
    float index_of_err_up;      //误差高于此值后积分指数为0
    float index_of_err_down;    //误差低于此值后积分指数为1
};




uint8_t mov_ctrl_init(rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);
void pid_tim_out_chg(rt_int32_t tim);
void pid_tim_in_chg(rt_int32_t tim);
void pid_in_out_data_limit_set(float temp);
void pid_in_integral_value_limit_set(float temp);
void pid_in_index_of_err_set(float temp_up,float temp_down);
void rov_move_out_to_in_process(void);

#endif /* APPLICATIONS_INC_MOVE_CONTROL_FUN_H_ */
