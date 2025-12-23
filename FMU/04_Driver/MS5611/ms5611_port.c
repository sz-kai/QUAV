/**
  ******************************************************************************
  * @file    ms5611_port.c
  * @author  
  * @version V1.0.0
  * @data    2025/10/18
  * @brief   MS5611移植层文件，连接硬件抽象层(hal层，本项目中暂时称bsp层)与驱动层，
  *         需要更改的配置已在文件中集中标明(搜索need to change)，包括与硬件相关的宏
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */


#include "ms5611_port.h"
#include "bsp_spi.h"   // Your SPI functions
#include "bsp_systick.h" // Your delay functions

/*********************************几个与硬件相关的宏需要更改************************************/
// Define your CS pin here for clarity
/*need to change*/
#define MS5611_CS_PORT    GPIOD
#define MS5611_CS_PIN     GPIO_Pin_7
#define MS5611_CS_CLK_EN() RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)

/**
 * @brief BSP-level function to control the CS pin.
 * @param state 0 for low (active), 1 for high (inactive).
 */
void bsp_ms5611_cs_ctrl(uint8_t state)
{
    if (state) {
        GPIO_SetBits(MS5611_CS_PORT, MS5611_CS_PIN);
    } else {
        GPIO_ResetBits(MS5611_CS_PORT, MS5611_CS_PIN);
    }
}

/**
 * @brief BSP-level wrapper for your SPI transfer function.
 * @param tx_data Data to send.
 * @return Data received.
 */
uint8_t bsp_ms5611_spi_txrx(uint8_t tx_data)
{
    // Assuming you have a function like this in your bsp_spi.c
    /*need to change,需要关联SPI的收发函数*/
    return SPI_TR_Byte(tx_data);
}

/**
 * @brief BSP-level wrapper for your delay function.
 * @param ms Milliseconds to delay.
 */
void bsp_ms5611_delay_ms(uint32_t ms)
{
    /*need to change,需要关联延时函数*/
    delay_ms(ms);
}

/**
 * @brief BSP-level wrapper for your get time function.
 * @return Current time in milliseconds.
 */
uint32_t bsp_ms5611_get_time_ms(void)
{
    /*need to change,需要关联获取时间函数*/
    return GetTick();
}

/**
 * @brief Configures the CS GPIO pin. This should be called once at startup.
 */
void bsp_ms5611_cs_init(void)
{
    /*need to change,这里配置CS引脚，可根据实际硬件修改*/
    GPIO_InitTypeDef GPIO_InitStructure;
    MS5611_CS_CLK_EN();

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = MS5611_CS_PIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(MS5611_CS_PORT, &GPIO_InitStructure);
    
    // Set CS high initially
    bsp_ms5611_cs_ctrl(1);
}

