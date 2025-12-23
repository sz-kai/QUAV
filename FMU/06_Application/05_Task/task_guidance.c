/**
 ******************************************************************************
 * @file    task_guidance.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/08
 * @brief   制导任务，负责轨迹规划、目标点生成等，还负责飞行模式管理、起飞降落逻辑判断等
 *          本模块暂时只涉及依据rc数据生成目标点
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_guidance.h"
#include "task_ioc.h"
#include "rc_mapper.h"

/**
 * @brief 归一化后的rc数据
 *
 */
static rc_map_data_t rc_map_data;

/**
 * @brief 期望值(设定点)
 *
 */
static angle_setpoint_t angle_sp;

/**
 * @brief 期望滚转角计算
 *
 * @param rc_map - 归一化后的rc数据
 * @retval float - 期望滚转角
 */
static float setpoint_roll_calc(float rc_map)
{
    return rc_map * SP_ROLL_MAX;
}

/**
 * @brief 期望俯仰角计算
 *
 * @param rc_map - 归一化后的rc数据
 * @retval float - 期望俯仰角
 */
static float setpoint_pitch_calc(float rc_map)
{
    return rc_map * SP_PITCH_MAX;
}

/**
 * @brief 期望偏航角计算
 * @note  对于yaw解算角度与这里的sp，范围都是[-PI,PI],且存在wraping问题，
 *        对于unwraping问题，是在计算误差时处理的，具体见task_control.c
 * @param rc_map - 归一化后的rc数据
 * @retval float - 期望偏航角
 */
static void setpoint_yaw_calc(float rc_map, float *yaw_sp)
{
    float dt = 1.0f / GUIDANCE_FREQ_HZ;
    /*1. 设定点计算，实现死区、锁头操作*/
    /*设定死区，杆量超过10%才动作*/
    /*下面计算出的yaw期望值范围为[-infty,infty],需要与解算出的yaw角度范围[-PI,PI]对应起来*/
    if (rc_map > 0.09)
    {
        *yaw_sp += SP_YAW_RATE_MAX * (rc_map - 0.09) / (1.0f - 0.09) * dt; /*这里进行了积分操作*/
    }
    else if (rc_map < -0.09)
    {
        *yaw_sp += SP_YAW_RATE_MAX * (rc_map + 0.09) / (1.0f - 0.09) * dt; /*这里进行了积分操作*/
    }
    /*2. 限制在[-PI,PI]范围内*/
    if (*yaw_sp > PI)
    {
        *yaw_sp -= 2.0f * PI;
    }
    else if (*yaw_sp < -PI)
    {
        *yaw_sp += 2.0f * PI;
    }
}

void task_guidance_init(void)
{
    
}

/**
 * @brief 制导任务，获取期望值
 *
 */
void task_guidance(void)
{
    rc_raw_data_t raw_data;
    /*1. RC数据映射*/
    memcpy(&raw_data, get_rc_raw_data(), sizeof(rc_raw_data_t));
    rc_mapper(&rc_map_data, &raw_data);
    /*2. 期望值生成*/
    angle_sp.roll = setpoint_roll_calc(rc_map_data.roll);
    angle_sp.pitch = setpoint_pitch_calc(rc_map_data.pitch);
    setpoint_yaw_calc(rc_map_data.yaw, &angle_sp.yaw);
}

/**
 * @brief 获取期望点
 *
 * @retval const setpoint_t* - 指向setpoint的指针
 */
const angle_setpoint_t *get_setpoint(void)
{
    return &angle_sp;
}
