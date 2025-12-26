/**
 ******************************************************************************
 * @file    attitude_control.h
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/03
 * @brief   姿态控制算法头文件
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#include "pro_include.h"
#include "pid.h"

/**
 * @brief 三轴控制力矩
 *
 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} control_torque_t;

/**
 * @brief 姿态控制句柄
 *
 */
typedef struct
{
    /*外环角度控制*/
    PID_Handle_t roll_pid_handle;
    PID_Handle_t pitch_pid_handle;
    PID_Handle_t yaw_pid_handle;
    /*内环角速度控制*/
    PID_Handle_t roll_rate_pid_handle;
    PID_Handle_t pitch_rate_pid_handle;
    PID_Handle_t yaw_rate_pid_handle;
} attitude_control_handle_t;

#endif /* __ATTITUDE_CONTROL_H */
