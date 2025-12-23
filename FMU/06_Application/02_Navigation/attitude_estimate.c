/**
 ******************************************************************************
 * @file    attitude_estimate.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/24
 * @brief
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "attitude_estimate.h"
#include "task_datalog.h"

#if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE

/*角速度积分解算姿态*/
Euler_Test_t gyro_euler;
/*加速度、磁力计解算姿态*/
Euler_Test_t acc_mag_euler;
/*初始姿态解算标志*/
static bool ae_init_f = false;
#endif

/**
 * @brief  检查加速度计数据是否正常
 * @note
 * @param  无
 * @retval 是否正常,true为正常,false为异常
 */
static bool Check_Acc_Data(const Axis3_f_u acc)
{
    /*三轴加速度平方和*/
    float sqs = acc.x * acc.x + acc.y * acc.y + acc.z * acc.z;
    /*计算比例因子*/
    float factor = sqs / (9.8f * 9.8f);
    /*如果比例因子大于0.8且小于1.2，则认为加速度计数据正常*/
    return factor > 0.8f && factor < 1.2f;
}

/**
 * @brief  检查磁力计数据是否正常
 * @note
 * @param  无
 * @retval 是否正常,true为正常,false为异常
 */
static bool Check_Mag_Data(const Axis3_f_u mag)
{
    /*检查磁力计数据是否正常*/
    return mag.x != 0.0f && mag.y != 0.0f && mag.z != 0.0f;
}

extern float el_roll;
extern float el_pitch;
extern float el_yaw;
/**
 * @brief  四元数转欧拉角(北东地-前右下)
 * @note
 * @param  无
 * @retval 无
 */
static void quaternion_to_euler(const Mahony_Param_t *M_P, volatile Euler_u *el)
{
    /*虽然可以由四元数和旋转矩阵推导欧拉角，但由于旋转矩阵是由四元数推导而来，存在浮点误差，所以这里直接用四元数计算欧拉角*/
#if 1
    /*滚转角(-180°~180°)*/
    el->roll = atan2f(2.0f * (M_P->q0 * M_P->q1 + M_P->q2 * M_P->q3),
                      1.0f - 2.0f * (M_P->q1 * M_P->q1 + M_P->q2 * M_P->q2)) * RADIAN_TO_ANGLE;
    /*俯仰角(-90°~90°)*/
    float sinp = 2.0f * (M_P->q0 * M_P->q2 - M_P->q3 * M_P->q1);
    /*防止因浮点误差导致sinp超出asinf定义域范围*/
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    el->pitch = asinf(sinp) * RADIAN_TO_ANGLE;
    /*偏航角(-180°~180°)*/
    el->yaw = atan2f(2.0f * (M_P->q0 * M_P->q3 + M_P->q1 * M_P->q2),
                     1.0f - 2.0f * (M_P->q2 * M_P->q2 + M_P->q3 * M_P->q3)) * RADIAN_TO_ANGLE;
#endif
#if 0
    /*通过方向余弦矩阵求取(机体到地理)*/
    el->roll = atan2f(M_P->DCM[2][1], M_P->DCM[2][2]) * RADIAN_TO_ANGLE;
    /*防止因浮点误差导致asinf定义域范围*/
    float sinp = M_P->DCM[2][0];
    if (M_P->DCM[2][0] > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (M_P->DCM[2][0] < -1.0f)
    {
        sinp = -1.0f;
    }
    el->pitch = -asinf(sinp) * RADIAN_TO_ANGLE;
    el->yaw = atan2f(M_P->DCM[1][0], M_P->DCM[0][0]) * RADIAN_TO_ANGLE;
    // if (el->yaw < 0.0f)
    // {
    //     el->yaw += 360.0f;
    // }
    //    if (isnan(el->roll) || isnan(el->pitch) || isnan(el->yaw))
    //    {
    //        __NOP(); /*在这里打断点*/
    //    }
#endif
}

#if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE

/**
 * @brief  加速度、磁力计解算姿态
 * @note
 * @param  无
 * @retval 无
 */
static void attitude_estimate_acc_mag_update(volatile Axis3_f_u *acc, volatile Axis3_f_u *mag, volatile Euler_Test_t *el_out)
{
    /*
     * 1. 计算 Roll (横滚角)
     * 保持原逻辑，但要注意传感器方向定义。
     * 假设：前右下(FRD)坐标系，Z轴向下。
     */
    el_out->roll = atan2f(-acc->y, -acc->z);

    /*
     * 2. 计算 Pitch (俯仰角) [修正点]
     * 原代码: atan2f(acc->x, -acc->z) 是错误的，因为它忽略了Y轴分量。
     * 正确公式需要使用 sqrt(y^2 + z^2) 作为分母，解耦 Roll 的影响。
     */
    el_out->pitch = atan2f(acc->x, sqrtf(acc->y * acc->y + acc->z * acc->z));

    /*
     * 3. 磁力计倾斜补偿
     * 利用计算出的 Roll 和 Pitch 将磁力计数据投影到水平面
     * 注意：sin/cos 函数必须使用弧度值，目前 el_out->roll/pitch 还是弧度，顺序正确。
     */
    float sin_r = sinf(el_out->roll);
    float cos_r = cosf(el_out->roll);
    float sin_p = sinf(el_out->pitch);
    float cos_p = cosf(el_out->pitch);

    // 投影到水平面的 X 分量 (北向分量)
    float mag_x = mag->x * cos_p +
                  mag->y * sin_r * sin_p +
                  mag->z * cos_r * sin_p;

    // 投影到水平面的 Y 分量 (东向分量)
    float mag_y = mag->y * cos_r - mag->z * sin_r;

    /*
     * 4. 计算 Yaw (偏航角) [修正点]
     * 使用 atan2f 直接计算，范围为 [-180, 180]
     * 在 NED (北东地) 坐标系中：
     * Yaw = atan2(Mag_East, Mag_North) = atan2(mag_y, mag_x)
     */
    el_out->yaw = -atan2f(mag_y, mag_x) * RADIAN_TO_ANGLE;

    /* 5. 将 Roll 和 Pitch 转换为角度 */
    el_out->roll *= RADIAN_TO_ANGLE;
    el_out->pitch *= RADIAN_TO_ANGLE;
}

/**
 * @brief  角速度积分解算姿态
 * @note
 * @param  无
 * @retval 无
 */
static void attitude_estimate_gyro_update(__IO Axis3_f_u *gyro, float dt, __IO Euler_Test_t *el_out)
{
    el_out->tick = GetTick();
    el_out->roll = el_out->roll + gyro->x * dt * RADIAN_TO_ANGLE;
    el_out->pitch = el_out->pitch + gyro->y * dt * RADIAN_TO_ANGLE;
    el_out->yaw = el_out->yaw + gyro->z * dt * RADIAN_TO_ANGLE;
}
#endif

// static void attitude_estimate_write_ringbuff(__IO Flight_Control_State_t *flgt_ctl)
//{
// #if LOG_RECORD_ATTITUDE
//     log_attitude_t log_attitude_data;
//     log_attitude_data.roll = flgt_ctl->euler.roll;
//     log_attitude_data.pitch = flgt_ctl->euler.pitch;
//     log_attitude_data.yaw = flgt_ctl->euler.yaw;
// #endif
// #if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
//     log_acc_mag_attitude_t log_acc_mag_attitude_data;
//     log_acc_mag_attitude_data.roll = acc_mag_euler.roll;
//     log_acc_mag_attitude_data.pitch = acc_mag_euler.pitch;
//     log_acc_mag_attitude_data.yaw = acc_mag_euler.yaw;
// #endif
// #if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
//     /* 准备校准数据 */
//     log_gyro_attitude_t log_gyro_attitude_data;
//     log_gyro_attitude_data.roll = gyro_euler.roll;
//     log_gyro_attitude_data.pitch = gyro_euler.pitch;
//     log_gyro_attitude_data.yaw = gyro_euler.yaw;
// #endif
//     // #if 1
//     //     uint32_t tick = GetTick();
//     //     cal_data_buf.tick = tick;
//     //     cal_data_buf.mag_x = mag_gauss_data.x;
//     //     cal_data_buf.mag_y = mag_gauss_data.y;
//     //     cal_data_buf.mag_z = mag_gauss_data.z;

//    //     raw_data_buf.tick = tick;
//    //     raw_data_buf.mag_x = mag_raw_data.x;
//    //     raw_data_buf.mag_y = mag_raw_data.y;
//    //     raw_data_buf.mag_z = mag_raw_data.z;
//    // #endif

//    /* 使用封装函数推送校准数据日志 */
// #if LOG_RECORD_ATTITUDE
//    Push_Log_Packet_To_RingBuff(LOG_ID_ATTITUDE, &log_attitude_data, sizeof(log_attitude_data), &SD_W_RingBuffMgr);
// #endif
// #if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
//    Push_Log_Packet_To_RingBuff(LOG_ID_ACC_MAG_ATTITUDE, &log_acc_mag_attitude_data, sizeof(log_acc_mag_attitude_data), &SD_W_RingBuffMgr);
// #endif
// #if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
//    Push_Log_Packet_To_RingBuff(LOG_ID_GYRO_ATTITUDE, &log_gyro_attitude_data, sizeof(log_gyro_attitude_data), &SD_W_RingBuffMgr);
// #endif
//}

/**
 * @brief  姿态解算更新(mahony互补滤波)
 * @note
 * @param  att_est:姿态解算结构体
 * @param  flgt_ctl:飞行控制状态,更新三轴欧拉角
 * @retval 无
 */
void attitude_estimate_update(Attitude_Estimate_t *att_est, __IO Sensor_t *sensor)
{
    /*mahony姿态解算*/
    att_est->Mahony_Param.use_acc = false;
    att_est->Mahony_Param.use_mag = false;
    if (att_est->attitude_estimate_init_flag == false)
    {
        Mahony_Init(&att_est->Mahony_Param, sensor->acc, sensor->mag);
        quaternion_to_euler(&att_est->Mahony_Param, &att_est->euler);
        att_est->attitude_estimate_init_flag = true;
        return;
    }
    /*检查加速度计数据是否正常，当飞机持续运动时，由于存在向心加速度，此时以重力为参考得出的角度不在准确，所以不使用加速度计数据*/
    if (Check_Acc_Data(sensor->acc) == true)
    {
        att_est->Mahony_Param.use_acc = true;
    }
    // else
    // {
    //     __NOP();/*在这里打断点*/
    // }
    if (Check_Mag_Data(sensor->mag) == true)
    {
        att_est->Mahony_Param.use_mag = true;
    }
    // else
    // {
    //     __NOP();/*在这里打断点*/
    // }
    Mahony_Update(&att_est->Mahony_Param, sensor->acc, sensor->mag, sensor->gyro);
    quaternion_to_euler(&att_est->Mahony_Param, &att_est->euler);

#if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
    /*姿态解算结果对比*/
    attitude_estimate_acc_mag_update(&flgt_ctl.sensor.acc, &flgt_ctl.sensor.mag, &acc_mag_euler);
    /*加速度计初始积分值*/
    if (ae_init_f == false)
    {
        gyro_euler.roll = acc_mag_euler.roll;
        gyro_euler.pitch = acc_mag_euler.pitch;
        gyro_euler.yaw = acc_mag_euler.yaw;
        ae_init_f = true;
        return;
    }
    attitude_estimate_gyro_update(&flgt_ctl.sensor.gyro, ATTITUDE_ESTIMATION_SAMPLE_TIME, &gyro_euler);
#endif
    //    attitude_estimate_write_ringbuff(att_est);
}

/**
 * @brief  获取旋转矩阵(机体坐标系到地理坐标系)
 * @note
 * @param  att_est:姿态解算结构体
 * @param  rotation_matrix:旋转矩阵
 * @retval 无
 */
void att_est_get_rotation_matrix(const Attitude_Estimate_t *att_est, __IO float rotation_matrix[3][3])
{
    memcpy(rotation_matrix, &att_est->Mahony_Param.DCM, sizeof(float[3][3]));
}

/**
 * @brief  获取欧拉角(北东地-前右下)
 * @note
 * @param  无
 * @retval 无
 */
void att_est_get_euler(const Attitude_Estimate_t *att_est, __IO Euler_u *el)
{
    // memcpy(el, &att_est->euler, sizeof(Euler_u));
    el->roll = att_est->euler.roll;
    el->pitch = att_est->euler.pitch;
    el->yaw = att_est->euler.yaw;
}

/**
 * @brief  获取姿态解算结果(包括欧拉角、旋转矩阵)
 * @note
 * @param  att_est:姿态解算结构体
 * @param  attitude:姿态解算结果
 * @retval 无
 */
void att_est_get_data(const Attitude_Estimate_t *att_est, __IO attitude_t *attitude)
{
    // memcpy(&attitude->euler, &att_est->euler, sizeof(Euler_u));
    // memcpy(&attitude->rotation_matrix, &att_est->Mahony_Param.DCM, sizeof(float[3][3]));
    for (int i = 0; i < 3; i++)
    {
        attitude->euler.axis_arr[i] = att_est->euler.axis_arr[i];
        for (int j = 0; j < 3; j++)
        {
            attitude->rotation_matrix[i][j] = att_est->Mahony_Param.DCM[i][j];
        }
    }
}
