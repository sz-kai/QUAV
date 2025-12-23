/**
  ******************************************************************************
  * @file    position_estimate.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/10/19
  * @brief   估计位置和速度，px4(1.6.0-rc)早期使用的位置估计算法改编(一种互补滤波算法)
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
 #ifndef __POSITION_ESTIMATE_H__
 #define __POSITION_ESTIMATE_H__
#include "pro_include.h"

typedef struct
{
    double lat;/*纬度*/
    double lon;/*经度*/
    double alt;/*高度*/
    double lat_rad;/*纬度弧度*/
    double lon_rad;/*经度弧度*/
    double sin_lat;/*纬度正弦值*/
    double cos_lat;/*纬度余弦值*/
} ref_pos_t;

/*位置估计器参数*/
typedef struct
{
    float gps_w_xy_p;/*GPS水平位置修正权值*/
    float gps_w_z_p;/*GPS垂直位置修正权值*/

    float gps_w_xy_v;/*GPS水平速度修正权值*/
    float gps_w_z_v;/*GPS垂直速度修正权值*/

    float baro_w_p;/*气压计水平位置修正权值*/

    float acc_bias_w;/*加速度计零偏修正权值*/

    float w_xy_res_v;/*水平速度回退权值*/
} pos_est_param_t;

/*位置估计状态结构体*/
typedef struct
{
    bool init_flag; /*位置算法初始化标志*/
    float baro_offset; /*气压高度基准*/
    uint8_t baro_init_count; /*气压高度初始化计数*/
    float acc_bias[3]; /*加速度计零偏*/
    bool gps_valid;/*GPS有效标志*/
    bool est_reset;/*位置估计重置标志，当GPS从无效变为有效时，重置位置估计*/
    bool ref_set;/*参考位置设置标志*/
    uint8_t ref_set_count;/*参考位置设置计数*/

    bool can_estimate_xy;/*能否估计水平位置*/

    float eph;/*水平误差*/
    float epv;/*垂直误差*/

    bool use_gps_xy;/*是否使用GPS数据修正水平位置估计*/
    bool use_gps_z;/*是否使用GPS数据修正垂直位置估计*/

    pos_est_param_t param;/*位置估计器参数*/

    /*数据缓冲区相关*/
    uint8_t est_ptr;/*估计器缓冲区指针*/
    float pos_est_buff[EST_BUFF_SIZE][3][2];/*位置估计器缓冲区*/
    float rot_mat_buff[EST_BUFF_SIZE][3][3];/*旋转矩阵缓冲区*/
    /*估计器输出*/
    ref_pos_t ref;/*参考位置*/

    float est_x[2];/*x轴位置、速度估计值*/
    float est_y[2];/*y轴位置、速度估计值*/
    float est_z[2];/*z轴位置、速度估计值*/

    float est_x_prev[2];/*x轴位置、速度上一时刻估计值*/
    float est_y_prev[2];/*y轴位置、速度上一时刻估计值*/
    float est_z_prev[2];/*z轴位置、速度上一时刻估计值*/

    double est_lat;/*经度估计值*/
    double est_lon;/*经度估计值*/
} Position_Estimate_t;


// void position_estimate_init(Position_Estimate_t *pos_est);
void position_estimate_update(Position_Estimate_t *pos_est, __IO Flight_Control_State_t *flgt_ctl);
void position_estimate_get_pos_vel(Position_Estimate_t *pos_est, __IO pos_vel_t *pos_vel);

#endif

