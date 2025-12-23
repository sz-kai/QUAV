/**
 ******************************************************************************
 * @file    position_estimate.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/10/18
 * @brief   估计位置和速度，px4(1.6.0-rc)早期使用的位置估计算法改编(一种互补滤波算法)
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "position_estimate.h"
#include "attitude_estimate.h"
#include "ubx.h"

/*GPS数据*/
// extern struct vehicle_gps_position_s *_gps_position; /*通过Loop_GPS_Parse解算的GPS数据会存储在这里*/
// extern uint8_t _got_posllh;                          /*解析到位置数据标志*/
// extern uint8_t _got_velned;                          /*解析到速度数据标志*/

/*气压高度初始化计数,用于计算气压高度基准*/
static const uint8_t BARO_INIT_COUNT = 100; /*定义一个常量优先使用这种定义*/

/*重力加速度*/
static const float CONSTANTS_ONE_G = 9.80665f /* m/s^2*/;

/*GPS最大允许误差*/
static const float max_eph_epv = 20.0f;
static const float min_eph_epv = 2.0f;

/*地球半径*/
static const float CONSTANTS_RADIUS_OF_EARTH = 6371000.0f;

/*最小有效权值*/
#define MIN_VALID_W 0.00001f

/**
 * @brief  经纬度到地理坐标系的投影转换初始化
 * @note
 * @param  无
 * @retval 无
 */
void pos_est_map_ll_to_ne_init(Position_Estimate_t *pos_est)
{
    pos_est->ref.lat_rad = pos_est->ref.lat * ANGLE_TO_RADIAN;
    pos_est->ref.lon_rad = pos_est->ref.lon * ANGLE_TO_RADIAN;
    pos_est->ref.sin_lat = sin(pos_est->ref.lat_rad);
    pos_est->ref.cos_lat = cos(pos_est->ref.lat_rad);
}

/**
 * @brief  经纬度到地理坐标系的投影转换(由经纬度获取北东地坐标系xy坐标)
 *         实际是将球面上的点以"同方位等距投影"的方式投影到平面上，投影中心为参考位置
 * @note
 * @param  pos_est:位置估计结构体
 * @param  lat:纬度
 * @param  lon:经度
 * @param  gps_proj:北东地坐标系xy坐标
 * @retval 无
 */
void pos_est_map_ll_to_ne(Position_Estimate_t *pos_est, double lat, double lon, float gps_proj[2])
{
    double lat_rad = lat * ANGLE_TO_RADIAN;
    double lon_rad = lon * ANGLE_TO_RADIAN;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - pos_est->ref.lon_rad);
    /*当前位置与参考位置夹角(与地心)*/
    double angle = acos(pos_est->ref.sin_lat * sin_lat + pos_est->ref.cos_lat * cos_lat * cos_d_lon);
    /*消除奇点*/
    double k = (fabs(angle) < EPSILON) ? 1.0 : (angle / sin(angle));
    /*计算x、y轴位置*/
    gps_proj[0] = k * (pos_est->ref.cos_lat * sin_lat - pos_est->ref.sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    gps_proj[1] = k * cos_lat * sin(lon_rad - pos_est->ref.lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
}

/**
 * @brief  地理坐标系转经纬度
 * @note
 * @param  pos_est:位置估计结构体
 * @param  x:x轴位置
 * @param  y:y轴位置
 * @param  lat:纬度
 * @param  lon:经度
 * @retval 无
 */
void pos_est_map_ne_to_ll(ref_pos_t *ref, float x, float y, double *lat, double *lon)
{
    double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);
    double lat_rad;
    double lon_rad;
    if (fabs(c) > EPSILON)
    {
        lat_rad = asin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
        lon_rad = (ref->lon_rad +
                   atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));
    }
    else
    {
        lat_rad = ref->lat_rad;
        lon_rad = ref->lon_rad;
    }
    *lat = lat_rad * RADIAN_TO_ANGLE;
    *lon = lon_rad * RADIAN_TO_ANGLE;
}

// /**
//  * @brief  参考位置初始化
//  * @note
//  * @param  pos_est:位置估计结构体
//  * @param  lat:纬度
//  * @param  lon:经度
//  * @param  alt:高度
//  * @retval 无
//  */
// void position_estimate_ref_init(Position_Estimate_t *pos_est, double lat, double lon, float gps_proj[2])
// {
// }

/**
 * @brief  位置估计初始化
 * @note
 * @param  pos_est:位置估计结构体
 * @retval 无
 */
void position_estimate_init(Position_Estimate_t *pos_est)
{
    /*位置估计器参数初始化*/
    /*1.修正权值，越大越信任测量值(GPS或气压计)，越小越信任预测值(加速度积分)*/
    pos_est->param.gps_w_xy_p = CONFIG_POS_EST_GPS_XY_W_P;
    pos_est->param.gps_w_z_p = CONFIG_POS_EST_GPS_Z_W_P;
    pos_est->param.gps_w_xy_v = CONFIG_POS_EST_GPS_XY_W_V;
    pos_est->param.gps_w_z_v = CONFIG_POS_EST_GPS_Z_W_V;
    pos_est->param.baro_w_p = CONFIG_POS_EST_BARO_W_P;

    pos_est->param.acc_bias_w = CONFIG_POS_EST_ACC_BIAS_W; /*加速度计零偏修正权值*/
    pos_est->param.w_xy_res_v = CONFIG_POS_EST_W_XY_RES_V; /*位置估计器精度过低时，水平速度回退权值(回退至0)*/

    // pos_est->init_flag = false;
    pos_est->ref_set = false;
    pos_est->ref_set_count = 0;

    /*缓冲区清零*/
    memset(pos_est->pos_est_buff, 0.0f, sizeof(pos_est->pos_est_buff));
    memset(pos_est->rot_mat_buff, 0.0f, sizeof(pos_est->rot_mat_buff));

    /*初始值*/
    memset(pos_est->est_x_prev, 0.0f, sizeof(pos_est->est_x_prev)); /*这里设置成初始预测值比较好*/
    memset(pos_est->est_y_prev, 0.0f, sizeof(pos_est->est_y_prev));
    memset(pos_est->est_z_prev, 0.0f, sizeof(pos_est->est_z_prev));
    memset(&pos_est->ref, 0.0f, sizeof(pos_est->ref));
}

/**
 * @brief  位置、速度预测(基于加速度积分)
 * @note
 * @param  无
 * @retval 无
 */
static void inertial_filter_prediction(float dt, float x[2], float acc)
{
    if (isfinite(dt))
    {
        if (!isfinite(acc))
        {
            acc = 0.0f;
        }
        x[0] += x[1] * dt + 0.5f * acc * dt * dt; /*零阶保持离散化*/
        x[1] += acc * dt;
    }
}

/**
 * @brief  位置、速度修正(互补滤波(与上面的预测函数共同构成)，本质上是一个二阶龙波格(Luenberger)观测器)
 * @note
 * @param
 * @retval 无
 */
static void inertial_filter_correction(float e, float dt, float x[2], uint8_t i, float w)
{
    if (isfinite(e) && isfinite(dt) && isfinite(w))
    {
        /*iNav中使用的预测修正算法(包括上面的预测函数)*/
        /*官方说是一种互补滤波算法，这里的权值设置逻辑并没有搞懂*/
        /*预测+修正函数合起来更像一个Luenberger观测器(两个输出p和v)，但这里权值设置如果按照极点配置来说，还是很奇怪*/
        float ewdt = e * w * dt;
        x[i] += ewdt;
        if (i == 0)
        {
            x[1] += w * ewdt;
        }
    }
}

/**
 * @brief  位置估计更新(基于互补滤波算法)
 * @note
 * @param  position_estimate:位置估计结构体
 * @param  flgt_ctl:飞行控制状态
 * @retval 无
 */
void position_estimate_update(Position_Estimate_t *pos_est, __IO Flight_Control_State_t *flgt_ctl)
{
    float gps_corr[3][2] = {0.0f};
    float gps_w_xy = 0;
    float gps_w_z = 0;
    float rot_mat[3][3] = {0.0f};      /*存储当前时刻的旋转矩阵*/
    float rot_mat_hist[3][3] = {0.0f}; /*存储历史时刻的旋转矩阵*/
    if (pos_est == NULL || flgt_ctl == NULL)
    {
        return;
    }
    if (!pos_est->init_flag)
    {
        /*初始化参数和气压高度基准*/
        if (isfinite(flgt_ctl->sensor.baro_alt))
        {
            /*计算气压高度基准*/
            /*配合position_estimate_update调用频率，确保baro_alt为新数据*/
            pos_est->baro_offset += flgt_ctl->sensor.baro_alt;
            pos_est->baro_init_count++;
            if (pos_est->baro_init_count >= BARO_INIT_COUNT)
            {
                pos_est->baro_offset /= pos_est->baro_init_count;
                /*参数初始化*/
                position_estimate_init(pos_est);
                pos_est->est_z[0] = 0.0f; // 以初始高度为0
                pos_est->init_flag = true;
            }
        }
        return;
    }
    memcpy(rot_mat, flgt_ctl->attitude.rotation_matrix, sizeof(rot_mat)); /*获取当前时刻的旋转矩阵*/
    /*修正加速度计零偏(这是一个重要的反馈回路，其中的偏置通过GPS修正)*/
    float acc_body[3] = {
        flgt_ctl->sensor.acc.x - pos_est->acc_bias[0],
        flgt_ctl->sensor.acc.y - pos_est->acc_bias[1],
        flgt_ctl->sensor.acc.z - pos_est->acc_bias[2]};
    float accel_earth[3] = {0.0f, 0.0f, 0.0f};
    /*转换到地理坐标系*/
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            accel_earth[i] += rot_mat[i][j] * acc_body[j];
        }
    }
    /*通过加速度积分计算速度和位置需要消除重力加速度*/
    accel_earth[2] += CONSTANTS_ONE_G; /*这里得到的加速度会用于预测速度和位置*/

    /*GPS有效性判断*/
    if (pos_est->gps_valid)
    {
        /*有效时，检测到gps精度过差时，设定gps_valid为false，这里的检测条件不如GPS无效时严格，以实现滞后效果*/
        /*实现一个滞后区间，以不同的条件来判断GPS是否有效*/
        if (flgt_ctl->sensor.gps.eph >= max_eph_epv &&
            flgt_ctl->sensor.gps.epv >= max_eph_epv)
        {
            /*eph和epv越大，GPS精度越低*/
            /*这里取得值是px4固件中的值，与across的不同*/
            /*GPS无效*/
            pos_est->gps_valid = false;
        }
    }
    else
    {
        /*GPS无效时，检测eph小于最大阈值的70%且卫星数据大于3时，再次使用GPS数据*/
        if (flgt_ctl->sensor.gps.eph <= 0.7f * max_eph_epv &&
            flgt_ctl->sensor.gps.epv <= 0.7f * max_eph_epv &&
            flgt_ctl->sensor.gps.satellites_used >= 3)
        {

            /*GPS有效*/
            pos_est->gps_valid = true;
            /*重置位置估计*/
            pos_est->est_reset = true;
        }
    }

    /*当GPS有效时*/
    if (pos_est->gps_valid)
    {
        /*转换GPS数据*/
        double lat = flgt_ctl->sensor.gps.lat * 1e-7; /*float只有6-7位有效数字，会损失精度，0.00001 度的差距在地面上大约是 1.1米*/
        double lon = flgt_ctl->sensor.gps.lon * 1e-7;
        double alt = flgt_ctl->sensor.gps.alt * 1e-3;
        /*如果参考位置未设置，则设置参考位置*/
        if (!pos_est->ref_set)
        {
            pos_est->ref_set_count++;
            /*GPS数据有效且持续一段时间后*/
            if (pos_est->ref_set_count > 10)
            {
                /*设置参考位置后才开始估计x、y轴位置、速度，初始位置为0，速度为GPS速度(此时飞机可能已经起飞)*/
                /*z轴位置、速度估计值的初始化和估计主要依赖于气压计，在position_estimate_update第一次运行时就设置为0*/
                pos_est->est_x[0] = 0.0f;
                pos_est->est_x[1] = flgt_ctl->sensor.gps.vel_n_m_s;
                pos_est->est_y[0] = 0.0f;
                pos_est->est_y[1] = flgt_ctl->sensor.gps.vel_e_m_s;
                /*记录参考位置经纬高，其中，由于设置参考位置时飞机可能已起飞，所以高度需要减去当前估计的高度*/
                pos_est->ref.lat = lat;
                pos_est->ref.lon = lon;
                /*使用加法，是因为alt是GPS给出的绝对海拔高度，恒正，而z_est是估计的飞机高度，北东地坐标系下为负*/
                pos_est->ref.alt = alt + pos_est->est_z[0];
                pos_est->ref_set = true;
                /*经纬度映射初始化*/
                pos_est_map_ll_to_ne_init(pos_est);
            }
        }
        /*参考位置已设置*/
        else
        {
            float gps_proj[2] = {0.0f, 0.0f};
            /*经纬度映射至x、y轴位置*/
            pos_est_map_ll_to_ne(pos_est, lat, lon, gps_proj);
            /*需要重置位置估计时*/
            if (pos_est->est_reset)
            {
                /*重置位置估计*/
                pos_est->est_reset = false;
                pos_est->est_x[0] = gps_proj[0];
                pos_est->est_x[1] = flgt_ctl->sensor.gps.vel_n_m_s;
                pos_est->est_y[0] = gps_proj[1];
                pos_est->est_y[1] = flgt_ctl->sensor.gps.vel_e_m_s;
            }
            /*获取当前时刻的GPS数据*/
            /*计算当前时刻的GPS数据在缓冲区中的索引*/
            int est_i = pos_est->est_ptr - 1 - MIN(EST_BUFF_SIZE - 1, MAX(0, (int)(GPS_DELAY / POS_EST_INTERVAL)));
            if (est_i < 0)
            {
                est_i += EST_BUFF_SIZE;
            }
            /*计算修正量*/
            /*位置*/
            gps_corr[0][0] = gps_proj[0] - pos_est->pos_est_buff[est_i][0][0];
            gps_corr[1][0] = gps_proj[1] - pos_est->pos_est_buff[est_i][1][0];
            gps_corr[2][0] = pos_est->ref.alt - alt - pos_est->pos_est_buff[est_i][2][0];
            /*速度*/
            gps_corr[0][1] = flgt_ctl->sensor.gps.vel_n_m_s - pos_est->pos_est_buff[est_i][0][1];
            gps_corr[1][1] = flgt_ctl->sensor.gps.vel_e_m_s - pos_est->pos_est_buff[est_i][1][1];
            gps_corr[2][1] = flgt_ctl->sensor.gps.vel_d_m_s - pos_est->pos_est_buff[est_i][2][1];
            /*获取GPS数据对应旋转矩阵*/
            memcpy(rot_mat_hist, pos_est->rot_mat_buff[est_i], sizeof(rot_mat_hist));
            /*GPS修正权值计算，这个权值与基础权值(用于调参)构成最终权值*/
            gps_w_xy = min_eph_epv / MAX(min_eph_epv, flgt_ctl->sensor.gps.eph); /*水平权值自适应，_gps_position->eph越大，GPS数据权值越小*/
            gps_w_z = min_eph_epv / MAX(min_eph_epv, flgt_ctl->sensor.gps.epv);  /*垂直权值自适应，_gps_position->epv越大，GPS数据权值越小*/
        }
    }
    else
    {
        /*GPS无效时*/
        pos_est->ref_set_count = 0;
        memset(gps_corr, 0.0f, sizeof(gps_corr));
    }

    /*计算气压计修正量*/
    float baro_corr = pos_est->baro_offset - flgt_ctl->sensor.baro_alt - pos_est->est_z[0];

    float est_dt = POS_EST_INTERVAL / 1000.0f; /*s*/
    /*eph、epv随着估计器迭代而增加，以量化估计器可靠性(模拟积分漂移)*/
    if (pos_est->eph < 0.000001f)
    {
        pos_est->eph = 0.001f; /*后续，eph会因gps提供的eph而更新，若gps_eph过小，会导致eph几乎不变，无法量化无GPS情况下的漂移*/
    }
    if (pos_est->eph < max_eph_epv)
    {
        pos_est->eph *= (1.0f + est_dt); /*指数增长*/
    }
    if (pos_est->epv < 0.000001f)
    {
        pos_est->epv = 0.001f; /*后续，epv会因gps提供的epv而更新，若gps_epv过小，会导致epv几乎不变，无法量化无GPS情况下的漂移*/
    }
    if (pos_est->epv < max_eph_epv)
    {
        pos_est->epv += (0.005f * est_dt); /*由于垂直方向有气压计修正，所以epv增长较慢(气压漂移)*/
    }

    /*判定是否使用GPS数据修正位置估计*/
    /*参考位置初始化、GPS数据可用且权值大于最小阈值(通过权值可控制是否使用GPS数据修正位置估计)*/
    pos_est->use_gps_xy = pos_est->ref_set && pos_est->gps_valid &&
                          pos_est->param.gps_w_xy_p > MIN_VALID_W &&
                          flgt_ctl->sensor.gps._got_posllh;
    pos_est->use_gps_z = pos_est->ref_set && pos_est->gps_valid &&
                         pos_est->param.gps_w_z_p > MIN_VALID_W &&
                         flgt_ctl->sensor.gps._got_posllh;
    if (pos_est->use_gps_xy && pos_est->use_gps_z)
    {
        flgt_ctl->sensor.gps._got_posllh = 0;
        flgt_ctl->sensor.gps._got_velned = 0;
    }

    /*判断估计器能否估计水平位置(无GPS且当模拟精度较小时或可以使用GPS时(use_gps_xy))*/
    pos_est->can_estimate_xy = pos_est->use_gps_xy || pos_est->eph < 0.7f * max_eph_epv;

    /*计算综合修正权值*/
    float gps_w_xy_p_final = pos_est->param.gps_w_xy_p * gps_w_xy;
    float gps_w_z_p_final = pos_est->param.gps_w_z_p * gps_w_z;
    float gps_w_xy_v_final = pos_est->param.gps_w_xy_v * gps_w_xy;
    float gps_w_z_v_final = pos_est->param.gps_w_z_v * gps_w_z;

    /*通过长期稳定的GPS数据，修正气压计长期偏置(直接修正基准，而不是修正每一个气压值)*/
    /*当GPS测量与估计器测量有误差时，对于z轴，很大部分是由于气压计长期偏置引起的*/
    /*通过修正基准，使气压计测量高度更高或更低，从而修正估计器中由气压计偏置引起的误差*/
    /*这里通过一个积分项来修正气压计长期偏置*/
    if (pos_est->use_gps_z)
    {
        /*积分项*/
        float offs_corr = gps_corr[2][0] * gps_w_z_p_final * est_dt;
        /*修正气压计长期偏置*/
        pos_est->baro_offset += offs_corr;
        /*baro_corr前面已计算，由于修正了baro_offset，所以需要同步修正*/
        baro_corr += offs_corr;
    }

    /*通过gps修正加速度计偏置*/
    float acc_bias_corr[3] = {0.0f, 0.0f, 0.0f};
    /*这里计算的是地理坐标系中的偏置*/
    if (pos_est->use_gps_xy)
    {
        /*位置部分*的是权值的平放，可能是为了匹配加速度单位,gps_corr(m)*s^(-1)*s^(-1)=m/s^2*/
        acc_bias_corr[0] = -gps_corr[0][0] * gps_w_xy_p_final * gps_w_xy_p_final;
        acc_bias_corr[0] += -gps_corr[0][1] * gps_w_xy_v_final;
        acc_bias_corr[1] = -gps_corr[1][0] * gps_w_xy_p_final * gps_w_xy_p_final;
        acc_bias_corr[1] += -gps_corr[1][1] * gps_w_xy_v_final;
    }
    if (pos_est->use_gps_z)
    {
        acc_bias_corr[2] = -gps_corr[2][0] * gps_w_z_p_final * gps_w_z_p_final;
        acc_bias_corr[2] += -gps_corr[2][1] * gps_w_z_v_final;
    }
    /*将地理坐标系中的偏置转换到机体坐标系*/
    for (uint8_t i = 0; i < 3; i++)
    {
        float c = 0;
        for (uint8_t j = 0; j < 3; j++)
        {
            c += rot_mat[j][i] * acc_bias_corr[j]; /*注意旋转矩阵是机体坐标系到地理坐标系的旋转矩阵*/
        }
        if (isfinite(c))
        {
            pos_est->acc_bias[i] += c * pos_est->param.acc_bias_w * est_dt;
        }
    }

    /*通过气压计修正加速度计偏置*/
    float baro_bias_corr[3] = {0.0f, 0.0f, 0.0f};
    baro_bias_corr[2] = -baro_corr * pos_est->param.baro_w_p * pos_est->param.baro_w_p;
    for (uint8_t i = 0; i < 3; i++)
    {
        float c = 0;
        for (uint8_t j = 0; j < 3; j++)
        {
            c += rot_mat[j][i] * baro_bias_corr[j];
        }
        if (isfinite(c))
        {
            pos_est->acc_bias[i] += c * pos_est->param.acc_bias_w * est_dt;
        }
    }

    /*高度预测与修正*/
    /*高度、速度预测(基于加速度积分)*/
    inertial_filter_prediction(est_dt, pos_est->est_z, accel_earth[2]);
    /*如果预测值异常，则使用上一时刻的预测值*/
    if (!isfinite(pos_est->est_z[0]) || !isfinite(pos_est->est_z[1]))
    {
        pos_est->est_z[0] = pos_est->est_z_prev[0];
        pos_est->est_z[1] = pos_est->est_z_prev[1];
    }
    /*修正(气压计修正)*/
    inertial_filter_correction(baro_corr, est_dt, pos_est->est_z, 0, pos_est->param.baro_w_p);
    /*GPS修正*/
    if (pos_est->use_gps_z)
    {
        /*更新epv*/
        pos_est->epv = MIN(pos_est->epv, flgt_ctl->sensor.gps.epv);
        inertial_filter_correction(gps_corr[2][0], est_dt, pos_est->est_z, 0, gps_w_z_p_final);
        inertial_filter_correction(gps_corr[2][1], est_dt, pos_est->est_z, 1, gps_w_z_v_final);
    }
    /*水平位置预测*/
    if (pos_est->can_estimate_xy)
    {
        /*当估计器精度较高时，使用加速度积分预测水平位置*/
        inertial_filter_prediction(est_dt, pos_est->est_x, accel_earth[0]);
        inertial_filter_prediction(est_dt, pos_est->est_y, accel_earth[1]);
        /*如果预测值异常，则使用上一时刻的预测值*/
        if (!isfinite(pos_est->est_x[0]) || !isfinite(pos_est->est_x[1]) || !isfinite(pos_est->est_y[0]) || !isfinite(pos_est->est_y[1]))
        {
            pos_est->est_x[0] = pos_est->est_x_prev[0];
            pos_est->est_x[1] = pos_est->est_x_prev[1];
            pos_est->est_y[0] = pos_est->est_y_prev[0];
            pos_est->est_y[1] = pos_est->est_y_prev[1];
        }
        /*可以使用GPS数据修正水平位置*/
        if (pos_est->use_gps_xy)
        {
            /*更新epx*/
            pos_est->eph = MIN(pos_est->eph, flgt_ctl->sensor.gps.eph);
            /*修正水平位置*/
            inertial_filter_correction(gps_corr[0][0], est_dt, pos_est->est_x, 0, gps_w_xy_p_final);
            inertial_filter_correction(gps_corr[0][1], est_dt, pos_est->est_x, 1, gps_w_xy_v_final);
            inertial_filter_correction(gps_corr[1][0], est_dt, pos_est->est_y, 0, gps_w_xy_p_final);
            inertial_filter_correction(gps_corr[1][1], est_dt, pos_est->est_y, 1, gps_w_xy_v_final);
        }
    }
    /*无法使用GPS时，逐渐将估计器速度回退至0，保证估计误差有界*/
    else
    {
        /*将纠正误差设置为-pos_est->est_x[1]，相当于将期望速度设置为0*/
        inertial_filter_correction(-pos_est->est_x[1], est_dt, pos_est->est_x, 1, pos_est->param.w_xy_res_v);
        inertial_filter_correction(-pos_est->est_y[1], est_dt, pos_est->est_y, 1, pos_est->param.w_xy_res_v);
    }
    /*将估计结果存入缓冲区*/
    pos_est->pos_est_buff[pos_est->est_ptr][0][0] = pos_est->est_x[0];
    pos_est->pos_est_buff[pos_est->est_ptr][0][1] = pos_est->est_x[1];
    pos_est->pos_est_buff[pos_est->est_ptr][1][0] = pos_est->est_y[0];
    pos_est->pos_est_buff[pos_est->est_ptr][1][1] = pos_est->est_y[1];
    pos_est->pos_est_buff[pos_est->est_ptr][2][0] = pos_est->est_z[0];
    pos_est->pos_est_buff[pos_est->est_ptr][2][1] = pos_est->est_z[1];
    /*保存有效估计值*/
    memcpy(pos_est->est_x_prev, pos_est->est_x, sizeof(pos_est->est_x_prev)); /*sizeof(目标)可以防止溢出，更安全*/
    memcpy(pos_est->est_y_prev, pos_est->est_y, sizeof(pos_est->est_y_prev));
    memcpy(pos_est->est_z_prev, pos_est->est_z, sizeof(pos_est->est_z_prev));
    /*旋转矩阵存入缓冲区*/
    memcpy(pos_est->rot_mat_buff[pos_est->est_ptr], rot_mat, sizeof(rot_mat));
    /*估计器指针自增*/
    pos_est->est_ptr++;
    if (pos_est->est_ptr >= EST_BUFF_SIZE)
    {
        pos_est->est_ptr = 0;
    }
    /*将估计结果转换为经纬度,供外部系统(如地面站)使用*/
    if (pos_est->can_estimate_xy)
    {
        pos_est_map_ne_to_ll(&pos_est->ref, pos_est->est_x[0], pos_est->est_y[0], &pos_est->est_lat, &pos_est->est_lon);
    }
}

/**
 * @brief  获取位置、速度估计值
 * @note
 * @param  pos_est:位置估计结构体
 * @param  pos_vel:位置、速度结构体
 * @retval 无
 */
void position_estimate_get_pos_vel(Position_Estimate_t *pos_est, __IO pos_vel_t *pos_vel)
{
    if (pos_est == NULL || pos_vel == NULL)
    {
        return;
    }
    pos_vel->pos.x = pos_est->est_x[0];
    pos_vel->pos.y = pos_est->est_y[0];
    pos_vel->pos.z = pos_est->est_z[0];
    pos_vel->vel.x = pos_est->est_x[1];
    pos_vel->vel.y = pos_est->est_y[1];
    pos_vel->vel.z = pos_est->est_z[1];
    pos_vel->pos_valid = pos_est->can_estimate_xy;
    pos_vel->vel_valid = pos_est->can_estimate_xy;
}
