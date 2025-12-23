/**
  ******************************************************************************
  * @file    com_data.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   全局数据
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_DATA_H
#define __COM_DATA_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "com_type.h"
#include "ringbuff.h"



/**********************************定时器相关******************************************/
extern uint32_t Tick;
/*主循环频率更新标志位*/
extern __IO FlagStatus Main_Loop_Update_Flag;

/**********************************RC数据******************************************/

/*遥控数据处理总结构体*/
extern __IO rc_data_t rc_data;

/**********************************数据记录*************************************/

extern ringbuff_t SD_W_RingBuffMgr;

/**********************************传感器任务相关*************************************/
/******************IMU相关************************/


/*imu原始数据*/
extern IMU_RawData_t imu_raw_data;
/*imu校准数据*/
extern IMU_Data_t imu_cal_data;
/*imu滤波数据*/
extern IMU_Data_t imu_filter_data;

/******************磁力计相关************************/

/*磁力计原始数据*/
extern MAG_RawData_t mag_raw_data;
/*磁力计转换数据(高斯)*/
extern MAG_Data_t mag_gauss_data;
/*磁力计校准数据*/
extern MAG_Data_t mag_cal_data;
/*磁力计滤波数据*/
extern MAG_Data_t mag_filter_data;


/******************气压计相关************************/


/*气压计原始数据*/
extern BARO_RawData_t baro_raw_data;
/*气压计滤波数据*/
extern BARO_Data_t baro_filter_data;

/******************GPS相关************************/



/**********************************飞行控制数据******************************************/

/*飞行控制状态结构体*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT]; /*三轴控制状态*/
    rc_data_t rc_data;/*rc数据*/
    Sensor_t sensor;/*传感器数据*/
    attitude_t attitude; /*姿态解算数据(包括欧拉角、旋转矩阵)*/
    pos_vel_t pos_vel;/*位置、速度估计数据*/
    // angle_setpoint_t setpoint;/*期望值(设定点)*/
    /*其他状态*/
    float throttle; /*油门*/
} Flight_Control_State_t;
/*飞行控制状态结构体*/
extern __IO Flight_Control_State_t flgt_ctl;

/*飞行状态结构体(用于创建静态只读的飞机状态变量)*/
/*由于Flight_Control_State_t flgt_ctl的全局可读可改特性创建了这个结构体*/
/*为保持与旧程序的兼容仍保留了flgt_ctl*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT]; /*三轴控制状态*/
    rc_raw_data_t rc_raw;/*rc原始数据*/
    rc_data_t* rc;/*rc数据*/
    Sensor_t* sensor;/*传感器数据*/
    attitude_t* attitude; /*姿态解算数据(包括欧拉角、旋转矩阵)*/
    pos_vel_t* pos_vel;/*位置、速度估计数据*/
    // angle_setpoint_t* setpoint;/*期望值(设定点)*/
    /*其他状态*/
    float* throttle; /*油门*/
} vehicle_state_t;

#endif /* __COM_DATA_H */
