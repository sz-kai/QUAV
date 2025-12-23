/**
  ******************************************************************************
  * @file    rc_mapper.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   遥控器数据映射头文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RC_MAPPER_H
#define __RC_MAPPER_H
/* Includes ------------------------------------------------------------------*/
#include "pro_include.h"


// /**
//  * @brief rc映射句柄
//  * 
//  */
// typedef struct
// {
//     rc_raw_data_t *p_rc_raw;/*原始rc数据指针*/
//     rc_map_data_t *p_rc_map;/*映射rc数据指针*/
//     /*期望值(setpoint)*/

//     float roll_sp;/*期望滚转角*/
//     float pitch_sp;/*期望俯仰角*/
//     float yaw_sp;/*期望偏航角*/
//     float roll_rate_sp;/*期望滚转角速率*/
//     float pitch_rate_sp;/*期望俯仰角速率*/
//     float yaw_rate_sp;/*期望偏航角速率*/
// } rc_map_handle_t;

// void rc_mapper_init(rc_map_handle_t *p_rc_map_handle,
//                     const rc_raw_data_t *p_rc_raw,
//                     rc_map_data_t *p_rc_map);

void rc_mapper(rc_map_data_t *p_rc_map, rc_raw_data_t *p_rc_raw);

#endif /* __RC_MAPPER_H */
