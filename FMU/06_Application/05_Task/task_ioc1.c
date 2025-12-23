#include "task_ioc1.h"
#include "ioc_protocol1.h"
#include <stdio.h>

/* 协议实例 */
static IOC_Protocol_t g_ioc_protocol;

/* 局部变量 */
static uint32_t g_last_heartbeat_time = 0;

/* 外部函数声明 (根据实际项目修改) */
extern uint32_t HAL_GetTick(void);                         /* 获取系统滴答 */
extern void BSP_USART_Send(uint8_t *p_data, uint16_t len); /* 串口发送函数 */

/* 内部函数声明 */
static void Task_IOC_ProcessMessage(IOC_Frame_t *frame);
static void Task_IOC_SendHeartbeat(void);

/**
 * @brief 初始化IOC任务
 */
void Task_IOC_Init(void)
{
    /* 初始化协议栈 */
    IOC_Protocol_Init(&g_ioc_protocol);

    g_last_heartbeat_time = HAL_GetTick();
}

/**
 * @brief 接收中断回调函数
 * @param data 接收到的字节
 * @note 在串口中断中调用此函数
 */
void Task_IOC_RxCallback(uint8_t data)
{
    /* 将接收到的数据压入协议栈FIFO */
    IOC_Protocol_PushByte(&g_ioc_protocol, data);
}

/**
 * @brief IOC任务主循环/入口
 * @note 需在主循环或RTOS任务中周期调用
 */
void Task_IOC_Entry(void)
{
    /* 1. 解析接收到的数据 */
    while (IOC_Protocol_Parse(&g_ioc_protocol))
    {
        /* 获取解析成功的帧 */
        IOC_Frame_t *frame = IOC_Protocol_GetFrame(&g_ioc_protocol);
        if (frame != NULL)
        {
            Task_IOC_ProcessMessage(frame);
        }
    }

    /* 2. 定时发送心跳包 (例如每500ms) */
    if (HAL_GetTick() - g_last_heartbeat_time >= 500)
    {
        Task_IOC_SendHeartbeat();
        g_last_heartbeat_time = HAL_GetTick();
    }
}

/**
 * @brief 处理解析到的消息
 * @param frame 数据帧指针
 */
static void Task_IOC_ProcessMessage(IOC_Frame_t *frame)
{
    switch (frame->cmd_id)
    {
    case IOC_CMD_HEARTBEAT:
        /* 收到心跳包，可以回复或重置看门狗 */
        // printf("Received Heartbeat\n");
        break;

    case IOC_CMD_MOTOR_SET:
        if (frame->len == sizeof(IOC_Msg_MotorSet_t))
        {
            IOC_Msg_MotorSet_t *motor_cmd = (IOC_Msg_MotorSet_t *)frame->payload;
            /* 处理电机控制指令 */
            // Motor_SetSpeed(motor_cmd->motor_1, motor_cmd->motor_2, ...);
        }
        break;

    case IOC_CMD_SYS_STATUS:
        /* 处理系统状态查询 */
        break;

    default:
        break;
    }
}

/**
 * @brief 发送心跳包
 */
static void Task_IOC_SendHeartbeat(void)
{
    uint8_t tx_buff[32];
    IOC_Msg_Heartbeat_t heartbeat;
    uint16_t len;

    heartbeat.timestamp = HAL_GetTick();
    heartbeat.status = 0x01; /* 示例状态 */

    /* 打包数据 */
    len = IOC_Protocol_Pack(IOC_CMD_HEARTBEAT, (uint8_t *)&heartbeat, sizeof(IOC_Msg_Heartbeat_t), tx_buff);

    /* 发送数据 */
    if (len > 0)
    {
        BSP_USART_Send(tx_buff, len);
    }
}
