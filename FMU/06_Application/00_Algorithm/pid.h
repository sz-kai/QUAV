#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <string.h> // for memset
#include <math.h>

/* --- 枚举定义 --- */

/**
 * @brief PID 控制模式
 */
typedef enum
{
    PID_MODE_NONE = 0, /* 不输出 */
    PID_MODE_P,        /* 仅 P */
    PID_MODE_PI,       /* P + I */
    PID_MODE_PD,       /* P + D */
    PID_MODE_PID       /* P + I + D */
} PID_Mode_e;

/**
 * @brief 积分器工作模式 (解决地面/空中逻辑)
 */
typedef enum
{
    PID_INT_NORMAL = 0, /* 正常积分 */
    PID_INT_FREEZE,     /* 暂停积分 (保持当前值，不累加误差) - 适用于执行器饱和或特殊状态 */
    PID_INT_RESET       /* 强制归零并停止积分 - 适用于地面怠速 */
} PID_Integration_Mode_e;

/**
 * @brief 微分器工作模式(误差微分/测量微分)
 * 
 */
typedef enum
{
    PID_DERIVATIVE_ERROR = 0, /* 误差微分 */
    PID_DERIVATIVE_MEASURE,   /* 测量微分 */
} PID_Derivative_Mode_e;

/**
 * @brief PID 句柄结构体
 */
typedef struct
{
    /* --- 参数 (Gains & Limits) --- */
    float Kp;
    float Ki;
    float Kd;
    
    float Output_Limit;     /* 总输出限幅 */
    float Integral_Limit;   /* 积分限幅 */
    // float DeadZone;         /* 死区 (误差在此范围内视为0) */
    
    float D_Term_Filter;    /* D项低通滤波系数 [0.0 ~ 1.0]. 1.0表示无滤波，0.1表示强滤波. 推荐 0.1~0.3 */

    /* --- 配置 (Config) --- */
    PID_Mode_e Mode;                   /* 控制模式 (P/PI/PD/PID) */
    PID_Integration_Mode_e Int_Mode;   /* 积分器模式 */
    PID_Derivative_Mode_e Derivative_Mode; /* 微分器模式 */

    /* --- 运行时状态 (Runtime State) --- */
    float Integral;     /* 积分累积值 */
    float Last_Error;   /* 上次误差 */
    float Last_Measure; /* 上次测量值 */
    float Last_D_Term;  /* 上次 D 项输出 (用于滤波) */
    float Output;       /* 当前输出 */

} PID_Handle_t;

/* --- 函数原型 --- */

/**
 * @brief 基础初始化
 */
void PID_Init(PID_Handle_t *hpid,
              PID_Mode_e mode,
              PID_Derivative_Mode_e der_mode,
              float Kp, float Ki, float Kd,
              float Max_I, float Max_Out);

/**
 * @brief 设置 D 项滤波器系数 (飞控建议设置)
 * @param alpha: 0.0-1.0, 越小滤波越强(延迟越大). 
 *        经验公式: alpha = dt / (dt + 1/(2*pi*fc)) 
 *        简单设定: 0.1 左右通常用于 400Hz 循环
 */
void PID_SetFilter(PID_Handle_t *hpid, float d_lpf_alpha);

/**
 * @brief 动态设置增益 (用于增益调度)
 */
void PID_SetGains(PID_Handle_t *hpid, float Kp, float Ki, float Kd);

/**
 * @brief 设置积分器状态 (用于解决地面/起飞逻辑)
 */
void PID_SetIntegralMode(PID_Handle_t *hpid, PID_Integration_Mode_e int_mode);

/**
 * @brief 清除内部状态 (积分、历史误差清零，但不改变参数和模式)
 */
void PID_ClearState(PID_Handle_t *hpid);

/**
 * @brief PID 计算核心
 */
float PID_Update(PID_Handle_t *hpid, float setpoint, float measure, float dt);

#endif /* __PID_H */
