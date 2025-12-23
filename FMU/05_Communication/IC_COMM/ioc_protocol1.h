#ifndef __IOC_PROTOCOL1_H
#define __IOC_PROTOCOL1_H

#include <stdint.h>
#include <stdbool.h>

/* 协议常量定义 */
#define IOC_FRAME_HEADER_0 0xAA /* 帧头字节0 */
#define IOC_FRAME_HEADER_1 0x55 /* 帧头字节1 */
#define IOC_MAX_PAYLOAD_LEN 128 /* 最大载荷长度 */
#define IOC_RX_BUFF_SIZE 256    /* 接收环形缓冲区大小 */

/* 命令ID定义 */
typedef enum
{
    IOC_CMD_HEARTBEAT = 0x01, /* 心跳包 */
    IOC_CMD_MOTOR_SET = 0x02, /* 电机控制 */
    IOC_CMD_MOTOR_FB = 0x03,  /* 电机反馈 */
    IOC_CMD_IMU_DATA = 0x04,  /* IMU数据 */
    IOC_CMD_SYS_STATUS = 0x05 /* 系统状态 */
} IOC_CmdID_t;

/* 解析状态机枚举 */
typedef enum
{
    IOC_STATE_WAIT_HEADER0,
    IOC_STATE_WAIT_HEADER1,
    IOC_STATE_WAIT_LEN,
    IOC_STATE_WAIT_ID,
    IOC_STATE_WAIT_PAYLOAD,
    IOC_STATE_WAIT_CRC
} IOC_ParseState_t;

/* 协议帧结构体 (用于接收解析) */
typedef struct
{
    uint8_t len;                          /* 数据长度 */
    uint8_t cmd_id;                       /* 命令ID */
    uint8_t payload[IOC_MAX_PAYLOAD_LEN]; /* 数据载荷 */
    uint8_t crc;                          /* 校验位 */
} IOC_Frame_t;

/* 环形缓冲区结构体 */
typedef struct
{
    uint8_t buffer[IOC_RX_BUFF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} IOC_RingBuff_t;

/* 协议句柄结构体 */
typedef struct
{
    IOC_RingBuff_t rx_fifo; /* 接收FIFO */
    IOC_ParseState_t state; /* 解析状态 */
    IOC_Frame_t rx_frame;   /* 当前接收帧 */
    uint8_t rx_index;       /* 接收索引 */

    /* 统计信息 */
    uint32_t rx_packets; /* 接收包计数 */
    uint32_t rx_errors;  /* 接收错误计数 */
} IOC_Protocol_t;

/* 数据包结构定义 - 业务层使用 */
#pragma pack(1)

/* 心跳包数据 */
typedef struct
{
    uint32_t timestamp;
    uint8_t status;
} IOC_Msg_Heartbeat_t;

/* 电机控制数据 */
typedef struct
{
    int16_t motor_1;
    int16_t motor_2;
    int16_t motor_3;
    int16_t motor_4;
} IOC_Msg_MotorSet_t;

#pragma pack()

/* 函数声明 */

/**
 * @brief 初始化协议栈
 * @param protocol 协议句柄指针
 */
void IOC_Protocol_Init(IOC_Protocol_t *protocol);

/**
 * @brief 将接收到的字节压入FIFO
 * @param protocol 协议句柄指针
 * @param data 接收到的字节
 */
void IOC_Protocol_PushByte(IOC_Protocol_t *protocol, uint8_t data);

/**
 * @brief 解析协议数据
 * @param protocol 协议句柄指针
 * @return true: 解析到一帧完整数据, false: 未解析到
 */
bool IOC_Protocol_Parse(IOC_Protocol_t *protocol);

/**
 * @brief 打包数据帧
 * @param cmd_id 命令ID
 * @param p_data 数据指针
 * @param len 数据长度
 * @param tx_buff 发送缓冲区
 * @return 打包后的总长度
 */
uint16_t IOC_Protocol_Pack(uint8_t cmd_id, uint8_t *p_data, uint8_t len, uint8_t *tx_buff);

/**
 * @brief 获取解析到的帧数据
 * @param protocol 协议句柄指针
 * @return 帧数据指针
 */
IOC_Frame_t *IOC_Protocol_GetFrame(IOC_Protocol_t *protocol);

#endif /* __IOC_PROTOCOL_H1 */
