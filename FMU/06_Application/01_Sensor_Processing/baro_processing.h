/**
  ******************************************************************************
  * @file    baro_processing.h
  * @author  
  * @version V1.0.0
  * @data    2025/10/20
  * @brief   气压计数据处理，包括
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#ifndef __BARO_PROCESSING_H
#define __BARO_PROCESSING_H
#include "pro_include.h"
#include "filter.h"
#include "ms5611.h"

/**
 * @brief 气压计处理句柄结构体
 *        其中封装处理过程中的所有状态
 */
typedef struct
{
    LPF1_State_t lpf_state;
    // ms5611_handle_t baro_handle;
    float altitude_m;  
}baro_proc_handle_t;


// /*气压计滤波器截止频率*/
// #define BARO_FILTER_CUTOFF_FREQUENCY CONFIG_BARO_FILTER_CUTOFF_FREQUENCY
// /*气压计滤波器采样频率*/
// #define BARO_FILTER_SAMPLE_FREQUENCY CONFIG_BARO_FILTER_SAMPLE_FREQUENCY
/*标准大气压强*/
#define BARO_STANDARD_PRESSURE CONFIG_BARO_STANDARD_PRESSURE
#define BARO_ALTITUDE_CONVERSION_FACTOR CONFIG_BARO_ALTITUDE_CONVERSION_FACTOR
#define BARO_ALTITUDE_CONVERSION_EXPONENT CONFIG_BARO_ALTITUDE_CONVERSION_EXPONENT





void baro_processing_init(baro_proc_handle_t* handle, float c_f, float dt);
bool baro_processing_update(baro_proc_handle_t* handle, const BARO_RawData_t *raw_data);
float baro_processing_get_altitude(baro_proc_handle_t* handle);

#endif /* __BARO_PROCESSING_H */
