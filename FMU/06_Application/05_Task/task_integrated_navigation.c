/**
 ******************************************************************************
 * @file    task_integrated_navigation.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/30
 * @brief   组合导航任务
 ******************************************************************************
 * @attention
 *
 * 组合导航任务
 *
 ******************************************************************************
 */
#include "task_integrated_navigation.h"

static Attitude_Estimate_t att_est; /*姿态解算结构体*/
static Position_Estimate_t pos_est; /*位置估计结构体*/

static attitude_t att;
static pos_vel_t pos_vel;

/**
 * @brief  组合导航任务初始化
 * @note
 * @param  无
 * @retval 无
 */
void Task_IntegratedNavigation_Init(void)
{
    att_est.attitude_estimate_init_flag = false;

    pos_est.init_flag = false;
    pos_est.baro_offset = 0.0f;
    pos_est.baro_init_count = 0;
}

/**
 * @brief  组合导航任务
 * @note
 * @param  无
 * @retval 无
 */

void Task_IntegratedNavigation(void)
{
    /*姿态解算，更新三轴欧拉角*/
    attitude_estimate_update(&att_est, &flgt_ctl.sensor);
    att_est_get_data(&att_est, &flgt_ctl.attitude);
    /*位置估计，更新位置、速度，50hz*/
    if (LOOP_FREQ_SET(LOOP_50_Hz, Tick, LOOP_500_Hz))
    {
        position_estimate_update(&pos_est, &flgt_ctl);
        position_estimate_get_pos_vel(&pos_est, &flgt_ctl.pos_vel);
    }
}

/**
 * @brief 获取姿态估计结果
 *
 */
const attitude_t *get_attitude_estimate(void)
{
    /*关闭中断，确保数据一致性*/
    __disable_irq();
    for (int i = 0; i < 3; i++)
    {
        att.euler.axis_arr[i] = att_est.euler.axis_arr[i];
        att.gyro.axis_arr[i] = flgt_ctl.sensor.gyro.axis_arr[i];
        // for (int j = 0; j < 3; j++)
        // {
        //     att_est.rotation_matrix[i][j] = att_est->Mahony_Param.DCM[i][j];
        // }
    }
    /*开启中断*/
    __enable_irq();
    return &att;
}

/**
 * @brief 获取位置估计结果
 *
 */
const pos_vel_t *get_position_estimate(void)
{
    pos_vel.pos.x = pos_est.est_x[0];
    pos_vel.pos.y = pos_est.est_y[0];
    pos_vel.pos.z = pos_est.est_z[0];
    pos_vel.vel.x = pos_est.est_x[1];
    pos_vel.vel.y = pos_est.est_y[1];
    pos_vel.vel.z = pos_est.est_z[1];
    // pos_vel.pos_valid = pos_est.can_estimate_xy;
    // pos_vel.vel_valid = pos_est.can_estimate_xy;
    return &pos_vel;
}
