/**
  ******************************************************************************
  * @file    task_guidance.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/12/08
  * @brief   制导任务头文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

#ifndef __TASK_GUIDANCE_H
#define __TASK_GUIDANCE_H
#include "pro_include.h"

/**
 * @brief 最大期望滚转角(弧度)
 * 
 */
#define SP_ROLL_MAX 30.0f*PI/180.0f
/**
 * @brief 最大期望俯仰角(弧度)
 * 
 */
#define SP_PITCH_MAX 30.0f*PI/180.0f
/**
 * @brief 最大期望偏航角速度(弧度)
 * 
 */
#define SP_YAW_RATE_MAX 30.0f*PI/180.0f
/**
 * @brief 导航任务调用频率(Hz)，在计算期望偏航角时使用
 * 
 */
#define GUIDANCE_FREQ_HZ 100.0f

/*===============================================================================================*/
/*=========                                  函数声明                                   ==========*/
/*===============================================================================================*/

/**
 * @brief 制导任务，获取期望值
 *
 */
void task_guidance(void);

/**
 * @brief 获取期望点
 *
 * @retval const setpoint_t* - 指向setpoint的指针
 */
const angle_setpoint_t *get_setpoint(void);

#endif /* __TASK_GUIDANCE_H */
