/**
 ******************************************************************************
 * @file    ms5611.h
 * @author
 * @version V2.0.0
 * @date    2023-10-27
 * @brief   MS5611 pressure sensor driver header file.
 *          
 ******************************************************************************
 */

#ifndef __MS5611_H
#define __MS5611_H
#include "stm32f4xx.h"






/* OSR (Over Sampling Ratio) settings */
typedef enum
{
    MS5611_OSR_256 = 0x00,  // Conversion time: 0.60 ms
    MS5611_OSR_512 = 0x02,  // Conversion time: 1.17 ms
    MS5611_OSR_1024 = 0x04, // Conversion time: 2.28 ms
    MS5611_OSR_2048 = 0x06, // Conversion time: 4.54 ms
    MS5611_OSR_4096 = 0x08  // Conversion time: 9.04 ms
} ms5611_osr_t;

/**
 * @brief Hardware interface function pointer types
 */
typedef uint8_t (*ms5611_spi_txrx_t)(uint8_t tx_data);
typedef void (*ms5611_cs_ctrl_t)(uint8_t state);
typedef void (*ms5611_delay_ms_t)(uint32_t ms);
typedef uint32_t (*ms5611_get_time_ms_t)(void);

/*结构体前向声明，用于隐藏结构体定义，只暴露指针类型*/
struct ms5611_handle_s;
typedef struct ms5611_handle_s* ms5611_handle_t;

/**
 * @brief Structure to hold final sensor data.
 */
typedef struct
{
    int32_t pressure_pa;      // Pressure in Pascals (e.g., 101325 Pa)
    int32_t temperature_degC; // Temperature in degrees Celsius * 100 (e.g., 25.12°C -> 2512)
} ms5611_data_t;







int8_t ms5611_init(ms5611_handle_t dev);
ms5611_handle_t ms5611_get_handle(void);
int8_t ms5611_read(ms5611_handle_t dev);
void ms5611_get_data(ms5611_handle_t dev, ms5611_data_t *data);
int32_t ms5611_get_altitude(ms5611_handle_t dev);
int32_t ms5611_get_temperature(ms5611_handle_t dev);

#endif /* __MS5611_H */


