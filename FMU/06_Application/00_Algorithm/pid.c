#include "pid.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

/**
 * @brief 限制函数内部宏
 */
static float PID_Constrain(float value, float max)
{
    if (value > max)
        return max;
    if (value < -max)
        return -max;
    return value;
}

/**
 * @brief 初始化 PID 控制器
 * @note  使用积分需要调用PID_SetIntegralMode设置积分器工作模式
 *        使用微分输出滤波需要调用PID_SetFilter设置D项低通滤波系数
 * @param hpid - PID 句柄指针
 * @param mode - 控制模式，在 PID_Mode_e 中定义
 * @param Kp - 比例系数
 * @param Ki - 积分系数
 * @param Kd - 微分系数
 * @param Max_I - 积分限幅
 * @param Max_Out - 输出限幅
 */
void PID_Init(PID_Handle_t *hpid,
              PID_Mode_e mode,
              PID_Derivative_Mode_e der_mode,
              float Kp, float Ki, float Kd,
              float Max_I, float Max_Out)
{
    if (hpid == NULL)
        return;

    /* 清除所有数据 */
    memset(hpid, 0, sizeof(PID_Handle_t));

    hpid->Mode = mode;
    hpid->Int_Mode = PID_INT_NORMAL; /* 默认正常积分 */

    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;

    hpid->Integral_Limit = Max_I;
    hpid->Output_Limit = Max_Out;
    // hpid->DeadZone = 0.0f;/*默认无死区*/

    hpid->D_Term_Filter = 1.0f;                   /* 默认无滤波 */
    hpid->Derivative_Mode = der_mode; /* 默认误差微分 */
}

/**
 * @brief 设置 PID 控制器的 D 项低通滤波系数
 *
 * @param hpid - PID 句柄指针
 * @param d_lpf_alpha - D 项滤波系数 [0.0 ~ 1.0]. 1.0表示无滤波，0.1表示强滤波. 推荐 0.1~0.3
 */
void PID_SetFilter(PID_Handle_t *hpid, float d_lpf_alpha)
{
    if (hpid == NULL)
        return;
    if (d_lpf_alpha < 0.0f)
        d_lpf_alpha = 0.0f;
    if (d_lpf_alpha > 1.0f)
        d_lpf_alpha = 1.0f;
    hpid->D_Term_Filter = d_lpf_alpha;
}

/**
 * @brief 设置 PID 控制器的比例、积分、微分系数
 *
 * @param hpid - PID 句柄指针
 * @param Kp - 比例系数
 * @param Ki - 积分系数
 * @param Kd - 微分系数
 */
void PID_SetGains(PID_Handle_t *hpid, float Kp, float Ki, float Kd)
{
    if (hpid == NULL)
        return;
    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;
}

/**
 * @brief 设置 PID 控制器的积分器工作模式
 *
 * @param hpid - PID 句柄指针
 * @param int_mode - 积分器模式，在 PID_Integration_Mode_e 中定义
 */
void PID_SetIntegralMode(PID_Handle_t *hpid, PID_Integration_Mode_e int_mode)
{
    if (hpid == NULL)
        return;

    /* 状态切换逻辑 */
    if (int_mode == PID_INT_RESET)
    {
        hpid->Integral = 0.0f; /*失能积分器时，需要将积分值清零*/
    }

    hpid->Int_Mode = int_mode;
}

/**
 * @brief 清除 PID 控制器的运行时状态
 *
 * @param hpid - PID 句柄指针
 */
void PID_ClearState(PID_Handle_t *hpid)
{
    if (hpid == NULL)
        return;
    hpid->Integral = 0.0f;
    hpid->Last_Error = 0.0f;
    hpid->Last_Measure = 0.0f;
    hpid->Last_D_Term = 0.0f;
    hpid->Output = 0.0f;
}

/**
 * @brief 计算 PID 控制器的输出
 *
 * @param hpid - PID 句柄指针
 * @param setpoint - 设定值
 * @param measure - 当前测量值
 * @param dt - 时间间隔 (单位：秒)
 * @return float - 计算得到的输出值
 */
float PID_Update(PID_Handle_t *hpid, float setpoint, float measure, float dt)
{
    if (hpid == NULL || hpid->Mode == PID_MODE_NONE)
        return 0.0f;
    if (dt <= 0.0f)
        return hpid->Output; // 防止除零

    float error = setpoint - measure;

    // /* 死区处理 (可选) */
    // if (ABS(error) < hpid->DeadZone)
    // {
    //     error = 0.0f;
    // }

    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;

    /* --- P 项计算 --- */
    if (hpid->Mode == PID_MODE_P || hpid->Mode == PID_MODE_PI ||
        hpid->Mode == PID_MODE_PD || hpid->Mode == PID_MODE_PID)
    {
        p_out = hpid->Kp * error;
    }

    /* --- I 项计算 --- */
    if (hpid->Mode == PID_MODE_PI || hpid->Mode == PID_MODE_PID)
    {
        /* 根据积分器模式处理 */
        if (hpid->Int_Mode == PID_INT_NORMAL)
        {
            hpid->Integral += error * dt;

            
        }
        else if (hpid->Int_Mode == PID_INT_RESET)
        {
            hpid->Integral = 0.0f;
        }
        /* PID_INT_FREEZE 模式下，Integral 保持不变 */

        i_out = hpid->Ki * hpid->Integral;
        /* 积分抗饱和 */
        hpid->Integral = PID_Constrain(i_out, hpid->Integral_Limit);
    }

    /* --- D 项计算 --- */
    if (hpid->Mode == PID_MODE_PD || hpid->Mode == PID_MODE_PID)
    {
        float delta;

        if (hpid->Derivative_Mode == PID_DERIVATIVE_MEASURE)
        {
            /* 微分先行：对测量值微分 (抑制设定值突变引起的冲击) */
            delta = -(measure - hpid->Last_Measure);
        }
        else
        {
            /* 误差微分 */
            delta = (error - hpid->Last_Error);
        }

        float raw_d = (delta / dt) * hpid->Kd;

        /* D项低通滤波: Output = Alpha * New + (1-Alpha) * Old */
        /* 飞控极其重要：防止噪音被放大 */
        if (hpid->D_Term_Filter < 1.0f && hpid->D_Term_Filter > 0.0f)
        {
            d_out = hpid->D_Term_Filter * raw_d + (1.0f - hpid->D_Term_Filter) * hpid->Last_D_Term;
        }
        else
        {
            d_out = raw_d;
        }

        hpid->Last_D_Term = d_out; // 保存滤波后的D项
    }

    /* --- 总输出 --- */
    float total_out = p_out + i_out + d_out;

    total_out = PID_Constrain(total_out, hpid->Output_Limit);

    /* --- 更新历史状态 --- */
    hpid->Last_Error = error;
    hpid->Last_Measure = measure;
    hpid->Output = total_out;

    return total_out;
}
