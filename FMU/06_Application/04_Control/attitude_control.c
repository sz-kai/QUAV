/**
 ******************************************************************************
 * @file    attitude_control.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/03
 * @brief   姿态控制算法实现,使用了串级PID控制算法
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "attitude_control.h"


/**
 * @brief 姿态控制初始化
 *
 */
void attitude_control_init(attitude_control_handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }
    /*2. 内环角速度控制*/
    /*外环roll*/
    PID_Init(&handle->roll_pid_handle, PID_MODE_P, PID_DERIVATIVE_ERROR,
             600.0f, 0.0f, 0.0f,
             0.0f, 600.0f);

    /*内环roll角速度*/
    PID_Init(&handle->roll_rate_pid_handle, PID_MODE_PID, PID_DERIVATIVE_MEASURE,
             80.0f, 200.0f, 2.0f,
             300.0f, 600.0f);
    /*外环pitch*/
    PID_Init(&handle->pitch_pid_handle, PID_MODE_P, PID_DERIVATIVE_ERROR,
             600.0f, 0.0f, 0.0f,
             0.0f, 600.0f);
    /*内环pitch角速度*/
    PID_Init(&handle->pitch_rate_pid_handle, PID_MODE_PID, PID_DERIVATIVE_MEASURE,
             80.0f, 200.0f, 2.0f,
             300.0f, 600.0f);
    /*外环yaw*/
    PID_Init(&handle->yaw_pid_handle, PID_MODE_P, PID_DERIVATIVE_ERROR,
             1500.0f, 0.0f, 0.0f,
             0.0f, 1000.0f);
    /*内环yaw角速度*/
    PID_Init(&handle->yaw_rate_pid_handle, PID_MODE_PID, PID_DERIVATIVE_MEASURE,
             350.0f, 250.0f, 2.0f,
             300.0f, 1000.0f);
}

/**
 * @brief 姿态控制外环循环
 *
 * @param handle - 姿态控制句柄指针
 * @param setpoint - 期望角度指针
 * @param measure - 测量角度,角速度指针
 * @param dt - 函数调用时间间隔
 * @param rate_sp - 输出,期望角速度指针
 */
void attitude_control_outer_loop(attitude_control_handle_t *handle,
                                 angle_setpoint_t *setpoint, attitude_t *measure, float dt,
                                 angle_rate_setpoint_t *rate_sp)
{
    if (handle == NULL || setpoint == NULL || measure == NULL || dt <= 0.0f || rate_sp == NULL)
    {
        return;
    }

    /*1. 外环控制，输出期望角速度*/
    rate_sp->roll = PID_Update(&handle->roll_pid_handle, setpoint->roll, measure->euler.roll, dt);
    rate_sp->pitch = PID_Update(&handle->pitch_pid_handle, setpoint->pitch, measure->euler.pitch, dt);
    rate_sp->yaw = PID_Update(&handle->yaw_pid_handle, setpoint->yaw, measure->euler.yaw, dt);
    /*2. 处理期望值,输出到内环*/
    rate_sp->roll = rate_sp->roll * 0.05f;
    rate_sp->pitch = rate_sp->pitch * 0.05f;
    rate_sp->yaw = rate_sp->yaw * 0.05f;
}

/**
 * @brief 姿态控制内环循环
 *
 * @param handle - 姿态控制句柄指针
 * @param setpoint - 期望角速度指针
 * @param measure - 测量角速度指针
 * @param dt - 函数调用时间间隔
 * @param out_torque - 输出,期望力矩指针
 */
void attitude_control_inner_loop(attitude_control_handle_t *handle,
                                 angle_rate_setpoint_t *setpoint, attitude_t *measure, float dt, float throttle,
                                 control_torque_t *out_torque)
{
    if (handle == NULL || setpoint == NULL || measure == NULL || dt <= 0.0f || out_torque == NULL)
    {
        return;
    }
    /*1. 油门值过小, 飞机停在地面时, 需要失能积分项*/
    if (throttle < THROTTLE_MIN)
    {
        PID_SetIntegralMode(&handle->roll_rate_pid_handle, PID_INT_RESET);
        PID_SetIntegralMode(&handle->pitch_rate_pid_handle, PID_INT_RESET);
        PID_SetIntegralMode(&handle->yaw_rate_pid_handle, PID_INT_RESET);
    }
    else
    {
        PID_SetIntegralMode(&handle->roll_rate_pid_handle, PID_INT_NORMAL);
        PID_SetIntegralMode(&handle->pitch_rate_pid_handle, PID_INT_NORMAL);
        PID_SetIntegralMode(&handle->yaw_rate_pid_handle, PID_INT_NORMAL);
    }
    /*1. 内环控制，输出期望角速度*/
    out_torque->roll = PID_Update(&handle->roll_rate_pid_handle, setpoint->roll, measure->gyro.x, dt);
    out_torque->pitch = PID_Update(&handle->pitch_rate_pid_handle, setpoint->pitch, measure->gyro.y, dt);
    out_torque->yaw = PID_Update(&handle->yaw_rate_pid_handle, setpoint->yaw, measure->gyro.z, dt);
}
