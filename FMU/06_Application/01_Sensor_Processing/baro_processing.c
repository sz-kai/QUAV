/**
 ******************************************************************************
 * @file    baro_processing.c
 * @author
 * @version V1.0.0
 * @data    2025/10/20
 * @brief   气压计数据处理，包括气压计数据转换和低通滤波)
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "baro_processing.h"

/*===============================================================================================*/
/*=========                              私有定义                                       ==========*/
/*===============================================================================================*/



// /*气压计处理句柄静态实例*/
// static struct baro_proc_handle_s baro_proc_handle;
/*===============================================================================================*/
/*=========                              API函数                                        ==========*/
/*===============================================================================================*/

/**
 * @brief  气压计数据处理初始化
 * @note
 * @param  handle: 气压计处理句柄
 * @param  c_f: 低通滤波器截止频率
 * @param  dt: 采样周期
 * @retval 无
 */
void baro_processing_init(baro_proc_handle_t* handle, float c_f, float dt)
{
    if (handle == NULL)
    {
        return;
    }
    /*初始化一阶滤波器*/
    LPF1_Init(&handle->lpf_state, c_f, dt);
    /*初始化海拔高度*/
    handle->altitude_m = 0.0f;
}

// /**
//  * @brief  获取气压计处理句柄
//  * @note
//  * @param  无
//  * @retval 气压计处理句柄
//  */
// baro_proc_handle_t baro_processing_get_handle(void)
// {
//     memset(&baro_proc_handle, 0, sizeof(struct baro_proc_handle_s));//清零
//     return &baro_proc_handle;
// }

/**
 * @brief  气压计数据处理,包括气压计数据转换和低通滤波
 * @note
 * @param  ms5611_data: 气压计数据
 * @param  lpf: 低通滤波器状态结构体
 * @retval 海拔高度(m)
 */
bool baro_processing_update(baro_proc_handle_t* handle, const BARO_RawData_t *raw_data)
{
    if (handle == NULL || raw_data == NULL)
    {
        return false;
    }
    if (!isfinite(raw_data->pressure))
    {
        return false;
    }
    /*气压计数据转换，将气压值(Pa)转换为海拔高度(m)*/
    float altitude = (1.0f - powf((float)raw_data->pressure / BARO_STANDARD_PRESSURE, BARO_ALTITUDE_CONVERSION_EXPONENT)) * BARO_ALTITUDE_CONVERSION_FACTOR;

    /*低通滤波*/
    handle->altitude_m = LPF1_Update(&handle->lpf_state, altitude);

    return true;
}

/**
  * @brief  获取海拔高度
  * @note   
  * @param  handle: 气压计处理句柄
  * @retval 海拔高度(m)
  */
float baro_processing_get_altitude(baro_proc_handle_t* handle)
{
    if (handle == NULL)
    {
        return 0.0f;
    }
    return handle->altitude_m;
}
