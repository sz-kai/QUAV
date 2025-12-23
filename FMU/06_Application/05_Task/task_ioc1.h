#ifndef __TASK_IOC_H
#define __TASK_IOC_H

#include "ioc_protocol1.h"

/* 任务周期定义 (ms) */
#define TASK_IOC_PERIOD_MS 10

/* 外部依赖声明 (假设存在) */
// extern void USART_SendByte(uint8_t data);
// extern uint8_t USART_ReadByte(void);

/* 函数声明 */

/**
 * @brief 初始化IOC任务
 */
void Task_IOC_Init(void);

/**
 * @brief IOC任务主循环/入口
 * @note 需在主循环或RTOS任务中周期调用
 */
void Task_IOC_Entry(void);

/**
 * @brief 接收中断回调函数
 * @param data 接收到的字节
 * @note 在串口中断中调用此函数
 */
void Task_IOC_RxCallback(uint8_t data);

#endif /* __TASK_IOC_H */
