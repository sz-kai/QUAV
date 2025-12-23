/**
 ******************************************************************************
 * @file    ms5611.c
 * @author
 * @version V1.0.0
 * @data    2025/10/18
 * @brief   MS5611驱动源文件，采用信息隐藏原则，只暴露必要的接口(只支持单实例)
 ******************************************************************************
 * @attention
 *
 * 移植驱动，更改ms5611_port.c(移植层，porting layer)即可。
 * 更改ms5611的配置在本文件中。
 *
 ******************************************************************************
 */

#include "ms5611.h"
#include "ms5611_port.h"
#include "bsp_systick.h" // Your delay functions
#include "string.h"

/*===============================================================================================*/
/*=========                                私有定义                                     ==========*/
/*===============================================================================================*/



/*状态机状态定义，MS5611每次开始转换数据需要等待一段时间，状态机机制与任务调用间隔配合可以避免阻塞式等待*/
typedef enum
{
    MS5611_STATE_WAIT_PRESSURE,    // 等待压力数据
    MS5611_STATE_WAIT_TEMPERATURE, // 等待温度数据
} ms5611_state_t;


/**
 * @brief MS5611 device handle structure.
 *        This should be initialized by the user and passed to driver functions.
 */
struct ms5611_handle_s
{
    // PROM calibration data
    uint16_t c1, c2, c3, c4, c5, c6;

    // Over-sampling ratio setting
    ms5611_osr_t osr;

    // User-provided hardware interface functions
    ms5611_spi_txrx_t spi_txrx; // Pointer to SPI transmit/receive function
    ms5611_cs_ctrl_t cs_ctrl;   // Pointer to CS control function (0 for low, 1 for high)
    ms5611_delay_ms_t delay_ms; // Pointer to millisecond delay function
    ms5611_get_time_ms_t get_time_ms;

    ms5611_state_t state;
    uint32_t conversion_start_time;
    uint32_t raw_pressure;    // D1
    uint32_t raw_temperature; // D2
    // Last valid data
    ms5611_data_t last_data;
};

/*ms5611静态实例*/
static struct ms5611_handle_s ms5611_handle;

/* MS5611 Commands */
#define CMD_RESET 0x1E 
#define CMD_ADC_READ   0x00
#define CMD_ADC_CONV 0x40
#define CMD_ADC_D1 0x00
#define CMD_ADC_D2 0x10
#define CMD_PROM_RD 0xA0

/* Private function prototypes */
static void ms5611_write_cmd(ms5611_handle_t dev, uint8_t cmd);
static int8_t ms5611_read_prom(ms5611_handle_t dev);
static void ms5611_start_conversion(ms5611_handle_t dev, uint8_t cmd_adc);
static uint32_t ms5611_read_conversion_result(ms5611_handle_t dev);
static uint16_t ms5611_get_conversion_time_ms(ms5611_osr_t osr);
static void ms5611_calculate(ms5611_handle_t dev, uint32_t D1, uint32_t D2, ms5611_data_t *data);




/*===============================================================================================*/
/*=========                                API                                         ==========*/
/*===============================================================================================*/


/**
 * @brief  初始化MS5611句柄
 * @note
 * @param  dev:MS5611句柄
 * @retval 0:成功
 * @retval -1:失败
 */
void ms5611_port_init(ms5611_handle_t dev)
{
    // 1. Initialize low-level hardware (GPIO, SPI)
    bsp_ms5611_cs_init();
    // Assuming your SPI is already initialized elsewhere (e.g., bsp_spi_init())

    // 2. Populate the device handle with pointers to BSP functions
    dev->cs_ctrl = bsp_ms5611_cs_ctrl;
    dev->spi_txrx = bsp_ms5611_spi_txrx;
    dev->delay_ms = bsp_ms5611_delay_ms;
    dev->get_time_ms = bsp_ms5611_get_time_ms;
}

/**
  * @brief  获取ms5611句柄
  * @note   
  * @param  无
  * @retval ms5611句柄
  */
ms5611_handle_t ms5611_get_handle(void)
{
    memset(&ms5611_handle, 0, sizeof(struct ms5611_handle_s));//清零
    return &ms5611_handle;
}

/**
 * @brief  MS5611初始化
 * @note   调用前需要调用ms5611_port_init初始化句柄
 * @param  dev:MS5611句柄
 * @retval 0:成功
 * @retval -1:失败
 */
int8_t ms5611_init(ms5611_handle_t dev)
{
    ms5611_port_init(dev);
    if (!dev || !dev->spi_txrx || !dev->cs_ctrl || !dev->delay_ms || !dev->get_time_ms)
    {
        return -1;
    }
    // Reset the sensor
    ms5611_write_cmd(dev, CMD_RESET);
    dev->delay_ms(3);

    // Read PROM coefficients
    if (ms5611_read_prom(dev) != 0)
    {
        return -1; // Failed to read PROM
    }

    // Set a default OSR if not set by user
    dev->osr = MS5611_OSR_4096;

    /*开始一次转换*/
    ms5611_start_conversion(dev, CMD_ADC_D1);
    dev->conversion_start_time = dev->get_time_ms();
    // 初始化状态机
    dev->state = MS5611_STATE_WAIT_PRESSURE;

    return 0;
}

/**
  * @brief  读取气压计数据
  * @note   
  * @param  dev:气压计句柄
  * @retval 1:成功
  * @retval 0:失败
  */

int8_t ms5611_read(ms5611_handle_t dev)
{
    if(!dev)
    {
        return -1;
    }
    uint8_t new_data_ready = 0;
    uint32_t current_time = dev->get_time_ms();

    switch (dev->state)
    {
    case MS5611_STATE_WAIT_PRESSURE:
        // 检查压力转换时间是否足够
        if (current_time - dev->conversion_start_time >= ms5611_get_conversion_time_ms(dev->osr))
        {
            // 读取压力结果
            dev->raw_pressure = ms5611_read_conversion_result(dev);

            // 开始温度转换
            ms5611_start_conversion(dev, CMD_ADC_D2);
            dev->conversion_start_time = current_time;
            dev->state = MS5611_STATE_WAIT_TEMPERATURE;
        }
        break;

    case MS5611_STATE_WAIT_TEMPERATURE:
        // 检查温度转换时间是否足够
        if (current_time - dev->conversion_start_time >= ms5611_get_conversion_time_ms(dev->osr))
        {
            // 读取温度结果
            dev->raw_temperature = ms5611_read_conversion_result(dev);

            // 计算最终值
            ms5611_calculate(dev, dev->raw_pressure, dev->raw_temperature, &dev->last_data);

            // 标记有新数据
            new_data_ready = 1;

            // 开始压力转换 
            ms5611_start_conversion(dev, CMD_ADC_D1);
            dev->conversion_start_time = current_time;
            dev->state = MS5611_STATE_WAIT_PRESSURE;
        }
        break;
    }
    return new_data_ready;
}

/**
  * @brief  获取气压计数据
  * @note   
  * @param  dev:气压计句柄
  * @param  data:数据,包括压力和温度
  * @retval 无
  */

void ms5611_get_data(ms5611_handle_t dev, ms5611_data_t *data)
{
    if(!dev || !data)
    {
        return;
    }
    // 简单地复制最后一次计算的有效数据
    *data = dev->last_data;
}

/**
  * @brief  获取高度数据
  * @note   
  * @param  无
  * @retval 无
  */
int32_t ms5611_get_altitude(ms5611_handle_t dev)
{
    return dev->last_data.pressure_pa;
}

/**
  * @brief  获取温度数据
  * @note   
  * @param  无
  * @retval 温度数据
  */
int32_t ms5611_get_temperature(ms5611_handle_t dev)
{
    return dev->last_data.temperature_degC;
}


/*===============================================================================================*/
/*=========                               私有函数                                      ==========*/
/*===============================================================================================*/

/**
 * @brief  写命令到MS5611
 * @note   CS低电平，SPI发送命令，CS高电平
 * @param  dev:MS5611句柄
 * @param  cmd:命令
 * @retval 无
 */
static void ms5611_write_cmd(ms5611_handle_t dev, uint8_t cmd)
{
    dev->cs_ctrl(0);
    dev->spi_txrx(cmd);
    dev->cs_ctrl(1);
}
/**
 * @brief  读取128位工厂校准数据(PROM)
 * @note
 * @param  dev:MS5611句柄
 * @retval 无
 */
static int8_t ms5611_read_prom(ms5611_handle_t dev)
{
    uint16_t prom_data[8];
    for (uint8_t i = 0; i < 8; i++)
    {
        dev->cs_ctrl(0); // CS low
        dev->spi_txrx(CMD_PROM_RD + (i * 2));
        uint8_t msb = dev->spi_txrx(0x00);
        uint8_t lsb = dev->spi_txrx(0x00);
        prom_data[i] = (msb << 8) | lsb;
        dev->cs_ctrl(1); // CS high
    }

    // Store coefficients
    dev->c1 = prom_data[1];
    dev->c2 = prom_data[2];
    dev->c3 = prom_data[3];
    dev->c4 = prom_data[4];
    dev->c5 = prom_data[5];
    dev->c6 = prom_data[6];

    // 简单检查如果PROM读取成功(系数应该不为0)
    if (dev->c1 == 0 && dev->c2 == 0 && dev->c3 == 0)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief Starts an ADC conversion without waiting.
 */
static void ms5611_start_conversion(ms5611_handle_t dev, uint8_t cmd_adc)
{
    dev->cs_ctrl(0);
    dev->spi_txrx(CMD_ADC_CONV + cmd_adc + dev->osr);
    dev->cs_ctrl(1);
}

/**
 * @brief Reads the result of a completed conversion.
 */
static uint32_t ms5611_read_conversion_result(ms5611_handle_t dev)
{
    dev->cs_ctrl(0);
    dev->spi_txrx(CMD_ADC_READ);
    uint32_t adc_val = 0;
    adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
    adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
    adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
    dev->cs_ctrl(1);
    return adc_val;
}

/**
 * @brief Returns the required conversion time in milliseconds for a given OSR.
 */
static uint16_t ms5611_get_conversion_time_ms(ms5611_osr_t osr)
{
    switch (osr)
    {
    case MS5611_OSR_256:
        return 1;
    case MS5611_OSR_512:
        return 2;
    case MS5611_OSR_1024:
        return 3;
    case MS5611_OSR_2048:
        return 5;
    case MS5611_OSR_4096:
        return 10;
    default:
        return 10;
    }
}

// /**
//  * @brief  读取原始ADC值(压力或温度)
//  * @note
//  * @param  dev:MS5611句柄
//  * @param  cmd_adc:命令
//  * @retval ADC值
//  */
// static uint32_t ms5611_read_adc(ms5611_handle_t *dev, uint8_t cmd_adc)
// {
//     // Start conversion
//     dev->cs_ctrl(0);
//     dev->spi_txrx(CMD_ADC_CONV + cmd_adc + dev->osr);
//     dev->cs_ctrl(1);

//     // Wait for conversion to complete
//     switch (dev->osr)
//     {
//     case MS5611_OSR_256:
//         dev->delay_ms(1);
//         break;
//     case MS5611_OSR_512:
//         dev->delay_ms(2);
//         break;
//     case MS5611_OSR_1024:
//         dev->delay_ms(3);
//         break;
//     case MS5611_OSR_2048:
//         dev->delay_ms(5);
//         break;
//     case MS5611_OSR_4096:
//         dev->delay_ms(10);
//         break;
//     }

//     // Read ADC result
//     dev->cs_ctrl(0);
//     dev->spi_txrx(CMD_ADC_READ);
//     uint32_t adc_val = 0;
//     adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
//     adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
//     adc_val = (adc_val << 8) | dev->spi_txrx(0x00);
//     dev->cs_ctrl(1);

//     return adc_val;
// }

/**
 * @brief  计算补偿温度和压力
 * @note
 * @param  dev:MS5611句柄
 * @param  D1:压力ADC值
 * @param  D2:温度ADC值
 * @param  data:数据
 * @retval 无
 */
static void ms5611_calculate(ms5611_handle_t dev, uint32_t D1, uint32_t D2, ms5611_data_t *data)
{
    int32_t dT;
    int64_t OFF, SENS;
    int32_t TEMP;

    // Temperature calculation
    dT = D2 - ((int64_t)dev->c5 << 8);
    TEMP = 2000 + ((int64_t)dT * dev->c6 >> 23);

    // Second order temperature compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000)
    {
        T2 = ((int64_t)dT * dT) >> 31;
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        if (TEMP < -1500)
        {
            OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }
    }

    TEMP -= T2;

    // Pressure calculation
    OFF = ((int64_t)dev->c2 << 17) + (((int64_t)dev->c4 * dT) >> 6);
    SENS = ((int64_t)dev->c1 << 16) + (((int64_t)dev->c3 * dT) >> 7);

    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    // Store final data
    data->temperature_degC = TEMP;
    data->pressure_pa = P;
}
