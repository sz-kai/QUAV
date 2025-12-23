/**
  ******************************************************************************
  * @file    ProConfig.h
  * @author  
  * @version 
  * @data    2025/06/18
  * @brief   项目配置，定义了项目“应该是什么样的”，如PID参数是多少、主循环频率是多少、功能开关
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRO_CONFIG_H
#define __PRO_CONFIG_H

/*****************************主循环频率及各任务循环频率**************************************/
/*循环频率*/
#define LOOP_1000_Hz 1000
#define LOOP_500_Hz 500
#define LOOP_250_Hz 250
#define LOOP_200_Hz 200
#define LOOP_100_Hz 100
#define LOOP_50_Hz 50
#define LOOP_25_Hz 25
#define LOOP_10_Hz 10
#define LOOP_5_Hz 5
/*主循环频率*/
#define LOOP_MAIN_RATE LOOP_1000_Hz
/*循环频率设定*/
/*Freq:设定频率,Tick:计数器,LOOP_RATE:循环频率*/
#define LOOP_FREQ_SET(SET_Freq,Tick,LOOP_RATE) ((Tick%(LOOP_RATE/SET_Freq))==0)
/**********************************中断优先级配置**********************************/
/*TIM6定时中断*/
#define TIM6_DAC_IRQ_PRIORITY 15
#define TIM6_DAC_IRQ_SUB_PRIORITY 0
/*DMA1_Stream2_IRQ*/
#define DMA1_S2_IRQ_PRIORITY 1
#define DMA1_S2_IRQ_SUB_PRIORITY 1
/*DMA1_Stream6_IRQ*/
#define DMA1_S6_IRQ_PRIORITY 1
#define DMA1_S6_IRQ_SUB_PRIORITY 1
/*DMA2_Stream1_IRQ，用于USART6_RX，双机通信*/
#define DMA2_C5_S1_IRQ_PRIORITY 1
#define DMA2_C5_S1_IRQ_SUB_PRIORITY 1
/*DMA2_Stream6_IRQ，用于USART6_TX，双机通信*/
#define DMA2_C5_S6_IRQ_PRIORITY 1
#define DMA2_C5_S6_IRQ_SUB_PRIORITY 1



/**********************************RC配置**********************************/
/*遥控器支持通道数*/
#define RC_CHANNEL_NUM 9
/*摇杆行程最大值*/
#define RC_MAX 2000
/*摇杆最小值*/
#define RC_MIN 1000
/*中值*/
#define RC_MID 1500
/*开关行程最大值*/
#define SWITCH_MAX 2000
/*开关行程最小值*/
#define SWITCH_MIN 1000
/*开关中值*/
#define SWITCH_MID 1500
/**********************************PID模块参数**********************************/
/*PID参数*/
/*角度环*/
#define ANGLE_ROLL_KP 600.0f
#define ANGLE_PITCH_KP 600.0f
#define ANGLE_YAW_KP 1500.0f
/*角度环只有P*/
#define ANGLE_ROLL_KI 
#define ANGLE_PITCH_KI 
#define ANGLE_YAW_KI 

#define ANGLE_ROLL_KD 
#define ANGLE_PITCH_KD 
#define ANGLE_YAW_KD 
/*角速度环*/
#define RATE_ROLL_KP 80.0f
#define RATE_PITCH_KP 80.0f
#define RATE_YAW_KP 350.0f

#define RATE_ROLL_KI 200.0f
#define RATE_PITCH_KI 200.0f
#define RATE_YAW_KI 250.0f

#define RATE_ROLL_KD 2.0f
#define RATE_PITCH_KD 2.0f
#define RATE_YAW_KD 0.0f

/*采样时间*/
#define ANG_VEL_SAMPLE_TIME 0.002f

/*yaw期望角速度灵敏度，越大飞机转的越快，无单位，非真实角速度*/
#define YAW_RATE_SENSITIVITY 672.0f
/*角速度积分限幅*/
#define ANG_VEL_INTEGRAL_MAX 300.0f
/*roll期望值最大值(°)*/
#define ROLL_ANGLE_TARGET_MAX 30.0f
/*pitch期望值最大值(°)*/
#define PITCH_ANGLE_TARGET_MAX 30.0f
/*角度(roll、pitch)比例输出限幅*/
#define ANGLE_P_OUTPUT_MAX 600.0f

/**********************************传感器数据处理**********************************/
/*陀螺仪滤波器截止频率*/
#define GYRO_FILTER_CUTOFF_FREQUENCY 50.0f
/*陀螺仪滤波器采样频率*/
#define GYRO_FILTER_SAMPLE_FREQUENCY 500.0f
/*加速度计滤波器截止频率*/
#define ACC_FILTER_CUTOFF_FREQUENCY 50.0f
/*加速度计滤波器采样频率*/
#define ACC_FILTER_SAMPLE_FREQUENCY 500.0f

/**********************************姿态解算相关**********************************/
/*是否使用GPS修正偏航角，1:使用，0:不使用*/
#define USE_GPS 0
// /*是否使用磁力计，1:使用，0:不使用*/
// #define USE_MAG 0
// /*是否使用加速度计，1:使用，0:不使用*/
// #define USE_ACC 0
/*采样周期(姿态解算算法运行周期),500hz*/
#define ATTITUDE_ESTIMATION_SAMPLE_TIME 0.002f

/*Mahony算法参数*/
#define MAHONY_KP 0.25f
#define MAHONY_KI 0.01f

/**********************************位置解算相关**********************************/
/***********************1.气压计数据处理*****************************/
/*气压计滤波器截止频率*/
#define CONFIG_BARO_FILTER_CUTOFF_FREQUENCY 50.0f
/*气压计滤波器采样频率*/
#define CONFIG_BARO_FILTER_SAMPLE_FREQUENCY 500.0f
/*气压转高度(barometric公式)*/
/*标准大气压强*/
#define CONFIG_BARO_STANDARD_PRESSURE 101325.0f
#define CONFIG_BARO_ALTITUDE_CONVERSION_FACTOR 44330.8f
#define CONFIG_BARO_ALTITUDE_CONVERSION_EXPONENT 1.0f/5.255f

/***********************2.位置估计器相关*****************************/
/*位置估计器参数*/
/*修正权值，越大越信任测量值(GPS或气压计)，越小越信任预测值(加速度积分)*/
/*水平位置修正权值*/
#define CONFIG_POS_EST_GPS_XY_W_P 1.0f
/*垂直位置修正权值*/
#define CONFIG_POS_EST_GPS_Z_W_P 0.00001f
/*水平速度修正权值*/
#define CONFIG_POS_EST_GPS_XY_W_V 2.0f
/*垂直速度修正权值*/
#define CONFIG_POS_EST_GPS_Z_W_V 1.8f
/*气压计垂直位置修正权值*/
#define CONFIG_POS_EST_BARO_W_P 0.5f
/*加速度计零偏修正权值*/
#define CONFIG_POS_EST_ACC_BIAS_W 0.05f
/*位置估计器精度过低时，水平速度回退权值(回退至0)*/
#define CONFIG_POS_EST_W_XY_RES_V 0.5f


/*位置估计器缓冲区大小(存储最近N个位置估计、IMU、姿态等数据，用于索引同一时刻的数据用于解算，消除数据滞后)*/
#define EST_BUFF_SIZE 25
// /*旋转矩阵缓冲区大小(存储最近N个旋转矩阵数据，相当于旋转矩阵的估计器缓冲区，用于索引同一时刻的数据用于解算，消除数据滞)*/
// #define ROT_MAT_BUFF_SIZE 25
/*GPS数据延迟(ms)*/
#define GPS_DELAY 150.0f
/*位置估计器调用间隔(ms)，注意，更改该值需要同时更改估计器调用频率*/
#define POS_EST_INTERVAL 20.0f
/*旋转矩阵数据更新间隔(ms)(姿态估计器调用间隔)，注意，更改该值需要同时更改旋转矩阵数据更新频率*/
#define ATT_EST_INTERVAL 10.0f
/**********************************日志消息记录配置**********************************/
/*使能日志记录，1:使能，0:不使能*/
#define LOG_RECORD_ENABLE 1
/*IMU原始数据，1:记录，0:不记录*/
#define LOG_RECORD_IMU_RAW 0
/*IMU校准数据，1:记录，0:不记录*/
#define LOG_RECORD_IMU_CAL 0
/*IMU滤波数据，1:记录，0:不记录*/
#define LOG_RECORD_IMU_FILTER 0
/*姿态解算数据，1:记录，0:不记录*/
#define LOG_RECORD_ATTITUDE 1
/*加速度计、磁力计解算姿态数据，1:记录，0:不记录*/
#define LOG_RECORD_ACC_MAG_ATTITUDE 1
/*陀螺仪解算姿态数据，1:记录，0:不记录*/
#define LOG_RECORD_GYRO_ATTITUDE 1
/*GPS数据，1:记录，0:不记录*/
#define LOG_RECORD_GPS 0
/*系统状态信息，1:记录，0:不记录*/
#define LOG_RECORD_STATUS 0
/*磁力计原始数据，1:记录，0:不记录*/
#define LOG_RECORD_MAG_RAW 0
/*磁力计高斯数据，1:记录，0:不记录*/
#define LOG_RECORD_MAG_GAUSS 0
/*磁力计校准数据，1:记录，0:不记录*/
#define LOG_RECORD_MAG_CAL 0


/*===============================================================================================*/
/*=========                                  位置控制相关                                ==========*/
/*===============================================================================================*/
/**********************************飞机状态**********************************/
/*油门限幅,影响飞机悬停与升降快慢*/
#define THROTTLE_MAX 1200.0f
/*最小油门*/
#define THROTTLE_MIN 150.0f



#endif /* __PRO_CONFIG_H */

