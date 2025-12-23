#ifndef __MS5611_PORT_H
#define __MS5611_PORT_H
#include "stm32f4xx.h"


void bsp_ms5611_cs_init(void);
void bsp_ms5611_cs_ctrl(uint8_t state);
uint8_t bsp_ms5611_spi_txrx(uint8_t tx_data);
void bsp_ms5611_delay_ms(uint32_t ms);
uint32_t bsp_ms5611_get_time_ms(void);

#endif /* __MS5611_PORT_H */

