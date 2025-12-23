/**
 ******************************************************************************
 * @file    ioc_protocol.h
 * @author  kai
 * @version
 * @data    2025/06/30
 * @brief   Inter-Chip Communication Protocol，定义协议格式，分别:
 *         1. 定义帧头、检验码等常量
 *         2. 定义所有可能的命令ID
 *         3. 使用struct定义每个命令对应的负载数据结构
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

/*数据包帧格式*/
/*******************************************************************************
 * 帧头1 | 帧头2 | 消息长度 | 有效载荷(n) |    校验和    |
 * 0xAA | 0x55  |  n     |    0xXX    | CK_A | CK_B |
 *              |-----CRC校验区域------|
 *
 * 有效载荷uint8_t payload[n]:
 * payload[0]: 消息类型
 * payload[1+]: 消息内容
 *
 *******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IOC_PROTOCOL_H
#define __IOC_PROTOCOL_H
/* Includes ------------------------------------------------------------------*/
#include "pro_include.h"
/* Exported define ------------------------------------------------------------*/

/*重新定IOC_DEBUG函数*/
// #define DEBUG_IOC_ENABLE
#ifdef DEBUG_IOC_ENABLE
#define IOC_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define IOC_DEBUG(format, ...)
#endif

/*IOC(双机通信一帧)数据缓冲区大小*/
#define IOC_BUFFER_SIZE 128

/*数据包帧头*/
#define IOC_SYNC1 0xAA
#define IOC_SYNC2 0x55

/*消息类型枚举*/
typedef enum
{
    IOC_MSG_TYPE_HEARTBEAT = 0x00, /*心跳包*/
    IOC_MSG_TYPE_GPS_DATA = 0x01,  /*GPS数据包*/
    IOC_MSG_TYPE_IMU_DATA = 0x02,  /*IMU数据包*/
    IOC_MSG_TYPE_RC_DATA = 0x03,   /*遥控器数据包*/
    IOC_MSG_TYPE_PWM_DATA = 0x04,  /*PWM数据包*/
} IOC_msg_type_t;

// /*心跳包*/
// #define IOC_MSG_TYPE_HEARTBEAT 0x00
// /*GPS数据包*/
// #define IOC_MSG_TYPE_GPS_DATA 0x01
// /*IMU数据包*/
// #define IOC_MSG_TYPE_IMU_DATA 0x02
// /*遥控器数据包*/
// #define IOC_MSG_TYPE_RC_DATA 0x03
// /*PWM数据包*/
// #define IOC_MSG_TYPE_PWM_DATA 0x04

/*有效载荷长度*/
#define IOC_PAYLOAD_PWM_LENGTH RC_CHANNEL_NUM
#define IOC_PAYLOAD_RC_LENGTH IOC_PAYLOAD_PWM_LENGTH * 2 + 1
#define IOC_PAYLOAD_HEARTBEAT_LENGTH 2

/*消息包长度*/
#define IOC_MSG_PWM_LENGTH IOC_PAYLOAD_PWM_LENGTH + 5
#define IOC_MSG_RC_LENGTH IOC_PAYLOAD_RC_LENGTH + 5
#define IOC_MSG_HEARTBEAT_LENGTH IOC_PAYLOAD_HEARTBEAT_LENGTH + 5

/* Exported types ------------------------------------------------------------*/

/*IIC协议解析器状态*/
typedef enum
{
    IOC_STATE_SYNC1 = 0,
    IOC_STATE_SYNC2,
    IOC_STATE_LENGTH,
    IOC_STATE_PAYLOAD,
    IOC_STATE_CCK_A,
    IOC_STATE_CCK_B
} IOC_parser_state_t;

typedef struct
{
    /* 环形缓冲区管理结构体 */
    ringbuff_t rx_rbbuff_mgr;
    /*ioc接收数据环形缓冲区*/
    uint8_t *rx_rbbuff;
    /*ioc发送数据缓冲区*/
    uint8_t *tx_buff;
    /*ioc接收数据缓冲区*/
    uint8_t *rx_frame; /*用于存储解析到的帧*/
    /*IOC-协议解析状态*/
    IOC_parser_state_t parser_state; /*记得初始化IOC_STATE_SYNC1*/
    /*发送完成标志*/
    FlagStatus send_done_flag;
    /*遥控器数据解码完成标志*/
    FlagStatus rc_decode_done;
    /*心跳包解码完成标志*/
    FlagStatus heartbeat_decode_done;
    /*有效载荷长度*/
    uint16_t rx_frame_len;
    /*有效载荷索引*/
    uint16_t rx_frame_index;
} IOC_handle_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ioc_init(IOC_handle_t *handle,
              uint8_t *rx_rbbuff, uint8_t *tx_buff, uint8_t *rx_frame);
bool ioc_parser(IOC_handle_t *handle);
IOC_msg_type_t ioc_decode_msg_type(IOC_handle_t *handle);
void ioc_decode_rc(IOC_handle_t *handle, rc_raw_data_t *rc_raw);
void ioc_decode_heartbeat(IOC_handle_t *handle, bool *heartbeat_switch);
#endif /* __IOC_PROTOCOL_H */
