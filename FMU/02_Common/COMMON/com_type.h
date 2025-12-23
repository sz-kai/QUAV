/**
 ******************************************************************************
 * @file    com_type.h
 * @author
 * @version V1.0.0
 * @data    2025/06/19
 * @brief   通用类型定义
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#ifndef __COM_TYPE_H
#define __COM_TYPE_H
#include "stm32f4xx.h"
#include "pro_config.h"
#include <stdbool.h>

// /*三轴16位数据*/
// typedef struct
// {
//     int16_t x;
//     int16_t y;
//     int16_t z;
// } Axis3i16_t;

// typedef union
// {
//     Axis3i16_t axis_s;
//     int16_t axis_arr[3];
// } Axis3i16_union_t;

 /*三轴16位数据*/
 typedef struct
 {
     int32_t x;
     int32_t y;
     int32_t z;
 } Axis3i32_t;

// typedef union
// {
//     Axis3i32_t axis_s;
//     int32_t axis_arr[3];
// } Axis3i32_union_t;

///*三轴32位数据*/
// typedef union
//{
//     struct
//     {
//         int32_t x;
//         int32_t y;
//         int32_t z;
//     };
//     int32_t axis_arr[3];
// } Axis3_i32_u;

// /*三轴64位数据*/
// typedef struct
// {
//     int64_t x;
//     int64_t y;
//     int64_t z;
// } Axis3i64_t;

// typedef union
// {
//     Axis3i64_t axis_s;
//     int64_t axis_arr[3];
// } Axis3i64_union_t;

/*三轴浮点数据*/
// typedef struct
// {
//     float x;
//     float y;
//     float z;
// } Axis3f_t;

 typedef union
 {
     struct
     {
         float x;
         float y;
         float z;
     };
     float axis_arr[3];
 } Axis3f_t;

// typedef union
// {
//     Axis3f_t axis_s;
//     float axis_arr[3];
// } Axis3f_union_t;

/*MPU6000原始数据结构体*/
// typedef struct
// {
//     Axis3i16_union_t AccRaw;/*原始数据*/
//     Axis3i16_union_t GyroRaw;
// } MPU6000RawData_t;

/*MPU6000校准数据结构体*/
// typedef struct
// {
//     Axis3f_union_t Acc;/*经处理的数据*/
//     Axis3f_union_t Gyro;
// } MPU6000CalData_t;

/*16位数据联合体*/
/*cortex-m采用小端存储，即低地址存低字节，高地址存高字节*/
// typedef union
// {
//     uint16_t data;
//     uint8_t data_arr[2];
// } u16_u8_union_t;

/*===============================================================================================*/
/*=========                              通用数据类型                                    ==========*/
/*===============================================================================================*/

/*三轴有符号16位数据*/
typedef union
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis_arr[3];
} Axis3_i16_u;

/*三轴无符号16位数据*/
typedef union
{
    struct
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
    };
    uint16_t axis_arr[3];
} Axis3_u16_u;

/*三轴有符号32位数据*/
typedef union
{
    struct
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int32_t axis_arr[3];
} Axis3_i32_u;

/*三轴无符号32位数据*/
typedef union
{
    struct
    {
        uint32_t x;
        uint32_t y;
        uint32_t z;
    };
    uint32_t axis_arr[3];
} Axis3_u32_u;

/*三轴有符号64位数据*/
typedef union
{
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis_arr[3];
} Axis3_i64_u;

/*三轴无符号64位数据*/
typedef union
{
    struct
    {
        uint64_t x;
        uint64_t y;
        uint64_t z;
    };
    uint64_t axis_arr[3];
} Axis3_u64_u;

/*三轴浮点数数据*/
typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis_arr[3];
} Axis3_f_u;

/*===============================================================================================*/
/*=========                              IMU数据                                       ==========*/
/*===============================================================================================*/

typedef struct
{
    Axis3_i16_u acc;
    Axis3_i16_u gyro;
} IMU_RawData_t;

typedef struct
{
    Axis3_f_u acc;
    Axis3_f_u gyro;
} IMU_Data_t;

// /*IMU数据结构体*/
// typedef struct
// {
//     Axis3i16_t acc;  /*加速度计*/
//     Axis3i16_t gyro; /*陀螺仪*/
// } imu_data_t;

/*===============================================================================================*/
/*=========                                 磁力计                                      ==========*/
/*===============================================================================================*/

typedef Axis3_i16_u MAG_RawData_t;
typedef Axis3_f_u MAG_Data_t;

/*===============================================================================================*/
/*=========                               气压计                                        ==========*/
/*===============================================================================================*/

/*气压计原始数据*/
typedef struct
{
    int32_t pressure;    /*大气压强，单位：帕斯卡*/
    int32_t temperature; /*温度，单位：摄氏度*/
} BARO_RawData_t;

/*气压计数据结构体*/
typedef struct
{
    float pressure;    /*大气压强，单位：帕斯卡*/
    float temperature; /*温度，单位：摄氏度*/
} BARO_Data_t;

/*===============================================================================================*/
/*=========                                      GPS                                   ==========*/
/*===============================================================================================*/

typedef struct
{
    uint64_t timestamp; // required for logger
    uint64_t time_utc_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t alt_ellipsoid;
    float s_variance_m_s;
    float c_variance_rad;
    float eph;
    float epv;
    float hdop;
    float vdop;
    int32_t noise_per_ms;
    int32_t jamming_indicator;
    float vel_m_s;
    float vel_n_m_s;
    float vel_e_m_s;
    float vel_d_m_s;
    float cog_rad;
    int32_t timestamp_time_relative;
    uint8_t fix_type;
    uint8_t vel_ned_valid;
    uint8_t satellites_used;
    uint8_t _padding0[5]; // required for logger

    __IO uint8_t _got_posllh; /*解析到位置数据标志*/
    __IO uint8_t _got_velned; /*解析到速度数据标志*/
} GPS_Data_t;

/*===============================================================================================*/
/*=========                                                                            ==========*/
/*===============================================================================================*/

/*三轴枚举*/
typedef enum
{
    ROLL,      /*滚转*/
    PITCH,     /*俯仰*/
    YAW,       /*偏航*/
    AXIS_COUNT /*轴数量*/
} Axis_e;

/*状态结构体*/
typedef struct
{
    float angle;        /*角度*/
    float angle_target; /*期望角度*/
    float rate;         /*角速度*/
    float rate_target;  /*期望角速度*/
} Axis_State_t;

/*三轴欧拉角*/
typedef union
{
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
    float axis_arr[3];
} Euler_u;

/*姿态解算结果数据结构体*/
typedef struct
{
    Euler_u euler;
    Axis3_f_u gyro;
    float rotation_matrix[3][3];
} attitude_t;

/*传感器数据结构体*/
typedef struct
{
    Axis3_f_u gyro;
    Axis3_f_u acc;
    Axis3_f_u mag;
    float baro_alt;
    GPS_Data_t gps;
} Sensor_t;

/*位置、速度估计数据结构体*/
typedef struct
{
    Axis3_f_u pos;
    Axis3_f_u vel;
    bool pos_valid; /*估计位置可用*/
    bool vel_valid; /*估计速度可用*/
} pos_vel_t;

/*===============================================================================================*/
/*=========                                     RC数据                                 ==========*/
/*===============================================================================================*/
// /*遥控器数据结构体(9通道)*/
// typedef struct
// {
//     uint16_t channel[RC_CHANNEL_NUM];
// } rc_data_raw_t;

/**
 * @brief 遥控器控制通道结构体
 *
 */
typedef struct
{
    /*归一化后的数据*/
    float roll;
    float pitch;
    float yaw;
    float throttle;
} rc_ctrl_t;

/**
 * @brief 遥控器开关状态结构体
 *
 */
typedef struct
{
    uint8_t switch_A;
    uint8_t switch_B;
    uint8_t switch_C;
    uint8_t switch_D;
    uint8_t switch_E;

    /*也可以为以具体功能命名*/
    // uint8_t arm;
} rc_switch_t;

/*遥控数据处理总结构体*/
typedef struct
{
    // rc_data_raw_t raw;   /*接收到的原始数据*/
    rc_ctrl_t ctrl; /*控制通道数据*/
    rc_switch_t sw; /*开关状态数据*/
} rc_data_t;

/*遥控器数据结构体(9通道)*/
typedef struct
{
    uint16_t channel[RC_CHANNEL_NUM];
} rc_raw_data_t;

/**
 * @brief 映射后rc数据结构体
 *
 */
typedef struct
{
    float roll;
    float pitch;
    float throttle;
    float yaw;
    uint8_t switch_A;
    uint8_t switch_B;
    uint8_t switch_C;
    uint8_t switch_D;
    uint8_t switch_E;
} rc_map_data_t;

/*===============================================================================================*/
/*=========                                 期望值                                      ==========*/
/*===============================================================================================*/

/*使用串级PID控制,角度期望值由遥控器给出,角速度期望值由外环输出,这里给出角度与角速度期望值两个结构体*/

/**
 * @brief 角度期望值(设定点)结构体
 *
 */
typedef struct
{
    float roll;       /*期望滚转角*/
    float pitch;      /*期望俯仰角*/
    float yaw;        /*期望偏航角*/
} angle_setpoint_t;

/**
 * @brief 角速度期望值(设定点)结构体
 * 
 */
typedef struct
{
    float roll;  /*期望滚转角速率*/
    float pitch; /*期望俯仰角速率*/
    float yaw;   /*期望偏航角速率*/
} angle_rate_setpoint_t;



/**********************************飞行控制数据******************************************/

// /*三轴结构体*/
// typedef struct
// {
//     float x;
//     float y;
//     float z;
// } Axis_t;
// /*三轴联合体*/
// typedef union
// {
//     struct
//     {
//         float x;
//         float y;
//         float z;
//     };
//     float axis_arr[3];
// } Axis_union_t;

#endif /* __COM_TYPE_H */
