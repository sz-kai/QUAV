/**
 ******************************************************************************
 * @file    rc_mapper.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/07/21
 * @brief   遥控器数据映射
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "rc_mapper.h"


// /**
//  * @brief 归一化rc控制通道数据，开关通道映射
//  *
//  */
// static void rc_normalization(rc_map_data_t *p_rc_map, rc_raw_data_t *p_rc_raw)
// {

// }

// /**
//  * @brief
//  *
//  */
// void rc_mapper_init(rc_map_handle_t *p_rc_map_handle,
//                     const rc_raw_data_t *p_rc_raw,
//                     rc_map_data_t *p_rc_map)
// {
//     p_rc_map_handle->p_rc_map = p_rc_map;
//     p_rc_map_handle->p_rc_raw = p_rc_raw;
// }

/**
 * @brief 归一化rc控制通道数据，开关通道映射
 *
 * @param p_rc_map - 映射rc数据结构体指针
 * @param p_rc_raw - 原始rc数据结构体指针
 */
void rc_mapper(rc_map_data_t *p_rc_map, rc_raw_data_t *p_rc_raw)
{
    /*控制通道，channel 1~4*/
    /*1. 限幅*/
    p_rc_raw->channel[0] = LIMIT(p_rc_raw->channel[0], RC_MIN, RC_MAX);
    p_rc_raw->channel[1] = LIMIT(p_rc_raw->channel[1], RC_MIN, RC_MAX);
    p_rc_raw->channel[2] = LIMIT(p_rc_raw->channel[2], RC_MIN, RC_MAX);
    p_rc_raw->channel[3] = LIMIT(p_rc_raw->channel[3], RC_MIN, RC_MAX);
    /*2. 归一化映射*/
    p_rc_map->roll = (((float)(p_rc_raw->channel[0] - RC_MID)) /
                            (RC_MAX - RC_MIN)) *
                           2.0f; /*映射通道1*/
    p_rc_map->pitch = (((float)(p_rc_raw->channel[1] - RC_MID)) /
                            (RC_MAX - RC_MIN)) *
                           2.0f; /*映射通道2*/
    p_rc_map->throttle = (((float)(p_rc_raw->channel[2] - RC_MID)) /
                            (RC_MAX - RC_MIN)) *
                           2.0f; /*映射通道3*/
    p_rc_map->yaw = (((float)(p_rc_raw->channel[3] - RC_MID)) /
                            (RC_MAX - RC_MIN)) *
                           2.0f; /*映射通道4*/
    /*3. 限幅*/
    p_rc_map->roll = LIMIT(p_rc_map->roll, -1.0f, 1.0f);
    p_rc_map->pitch = LIMIT(p_rc_map->pitch, -1.0f, 1.0f);
    p_rc_map->throttle = LIMIT(p_rc_map->throttle, -1.0f, 1.0f);
    p_rc_map->yaw = LIMIT(p_rc_map->yaw, -1.0f, 1.0f);

    /*开关通道，channel 5~9*/
    /*开关1*/
    if (p_rc_raw->channel[4] > SWITCH_MIN - 20 && p_rc_raw->channel[4] < SWITCH_MIN + 20)
        p_rc_map->switch_A = 0;
    else if (p_rc_raw->channel[4] > SWITCH_MID - 20 && p_rc_raw->channel[4] < SWITCH_MID + 20)
        p_rc_map->switch_A = 1;
    else if (p_rc_raw->channel[4] > SWITCH_MAX - 20 && p_rc_raw->channel[4] < SWITCH_MAX + 20)
        p_rc_map->switch_A = 2;

    /*开关2*/
    if (p_rc_raw->channel[5] > SWITCH_MIN - 20 && p_rc_raw->channel[5] < SWITCH_MIN + 20)
        p_rc_map->switch_B = 0;
    else if (p_rc_raw->channel[5] > SWITCH_MID - 20 && p_rc_raw->channel[5] < SWITCH_MID + 20)
        p_rc_map->switch_B = 1;
    else if (p_rc_raw->channel[5] > SWITCH_MAX - 20 && p_rc_raw->channel[5] < SWITCH_MAX + 20)
        p_rc_map->switch_B = 2;

    /*开关3*/
    if (p_rc_raw->channel[6] > SWITCH_MIN - 20 && p_rc_raw->channel[6] < SWITCH_MIN + 20)
        p_rc_map->switch_C = 0;
    else if (p_rc_raw->channel[6] > SWITCH_MID - 20 && p_rc_raw->channel[6] < SWITCH_MID + 20)
        p_rc_map->switch_C = 1;
    else if (p_rc_raw->channel[6] > SWITCH_MAX - 20 && p_rc_raw->channel[6] < SWITCH_MAX + 20)
        p_rc_map->switch_C = 2;

    /*开关4*/
    if (p_rc_raw->channel[7] > SWITCH_MIN - 20 && p_rc_raw->channel[7] < SWITCH_MIN + 20)
        p_rc_map->switch_D = 0;
    else if (p_rc_raw->channel[7] > SWITCH_MID - 20 && p_rc_raw->channel[7] < SWITCH_MID + 20)
        p_rc_map->switch_D = 1;
    else if (p_rc_raw->channel[7] > SWITCH_MAX - 20 && p_rc_raw->channel[7] < SWITCH_MAX + 20)
        p_rc_map->switch_D = 2;

    /*开关5*/
    if (p_rc_raw->channel[8] > SWITCH_MIN - 20 && p_rc_raw->channel[8] < SWITCH_MIN + 20)
        p_rc_map->switch_E = 0;
    else if (p_rc_raw->channel[8] > SWITCH_MID - 20 && p_rc_raw->channel[8] < SWITCH_MID + 20)
        p_rc_map->switch_E = 1;
    else if (p_rc_raw->channel[8] > SWITCH_MAX - 20 && p_rc_raw->channel[8] < SWITCH_MAX + 20)
        p_rc_map->switch_E = 2;
}
