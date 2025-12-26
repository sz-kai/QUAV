/**
  ******************************************************************************
  * @file    task_control.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/12/10
  * @brief   控制任务源文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#include "task_control.h"
#include "task_guidance.h"
#include "task_integrated_navigation.h"

void task_control_init(void)
{
    
}

void task_control(void)
{
    /*1. 获取期望值*/
    const angle_setpoint_t *setpoint = get_setpoint();
    /*2. 获取测量值*/
    const pos_vel_t *pos_vel = get_position_estimate();
    const attitude_t *attitude = get_attitude_estimate();
    /*3. 计算误差*/
    float roll_err = setpoint->roll - attitude->euler.roll;
    float pitch_err = setpoint->pitch - attitude->euler.pitch;
    float yaw_err = setpoint->yaw - attitude->euler.yaw;
    /*yaw unwraping*/
    if (yaw_err > PI)
    {
        yaw_err -= 2 * PI;
    }
    else if (yaw_err < -PI)
    {
        yaw_err += 2 * PI;
    }
    
}
