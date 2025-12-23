/**
 ******************************************************************************
 * @file    task_ioc.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/06
 * @brief   负责与从处理器(IO Processor)通信，处理接收到的RC数据包、心跳包等，发送PWM数据包等
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

#include "task_ioc.h"

/*===================================================================*/
/*=========                     ioc句柄                    ==========*/
/*===================================================================*/

static IOC_handle_t ioc_handle;
/*分配内存*/
#define IOC_BUFF_SIZE 128
/*接收数据环形缓冲区*/
static uint8_t ioc_rx_rbbuff[IOC_BUFF_SIZE];
/*发送数据缓冲区*/
static uint8_t ioc_tx_buff[IOC_BUFF_SIZE];
/*接收数据缓冲区，存放解析后的帧*/
static uint8_t ioc_rx_frame[IOC_BUFF_SIZE];

/*===================================================================*/
/*=========                  解析到的数据                    ==========*/
/*===================================================================*/

/*rc原始数据*/
static rc_raw_data_t rc_raw;
/*安全开关*/
bool safe_switch;

/**
 * @brief 与IO Processor通信初始化
 *
 */
void task_ioc_init(void)
{
    ioc_init(&ioc_handle,
             ioc_rx_rbbuff, ioc_tx_buff, ioc_rx_frame);
}

/**
 * @brief
 *
 */
void task_ioc(void)
{
    /*1. 解析接收到的数据*/
    if (ioc_parser(&ioc_handle))
    {
        switch (ioc_decode_msg_type(&ioc_handle))
        {
        case IOC_MSG_TYPE_HEARTBEAT:
            ioc_decode_heartbeat(&ioc_handle, &safe_switch);
            break;
        case IOC_MSG_TYPE_GPS_DATA:
            break;
        case IOC_MSG_TYPE_IMU_DATA:
            break;
        case IOC_MSG_TYPE_RC_DATA:
            ioc_decode_rc(&ioc_handle, &rc_raw);
            break;
        case IOC_MSG_TYPE_PWM_DATA:
            break;
        default:
            break;
        }
    }
}

/**
 * @brief 获取rc数据(外部不可更改)
 *
 * @retval const rc_raw_data_t* - 指向rc_raw_data_t结构体的指针
 */
const rc_raw_data_t *get_rc_raw_data(void)
{
    /*提取RC数据*/
    return &rc_raw;
}

///**
// * @brief  发送IOC数据包
// * @note
// * @param  无
// * @retval 无
// */
// static void IOC_Send_Packet(uint8_t *data, uint16_t length)
//{
//    /*这里将数据放入了全局变量中，防止发送未完成，局部变量被释放，发送数据错误*/
//    memcpy(IOC_TX_Buff, data, length);
//    USART_DMA_Send(DMA2_Stream6, IOC_TX_Buff, length);
//}

// /**
//  * @brief  发送心跳包
//  * @note
//  * @param  无
//  * @retval 无
//  */
// void IOC_Send_Heartbeat(void)
// {
//     uint8_t heartbeat_packet[IOC_MSG_HEARTBEAT_LENGTH];
//     IOC_PUT_u8(heartbeat_packet, 0, IOC_SYNC1);
//     IOC_PUT_u8(heartbeat_packet, 1, IOC_SYNC2);
//     IOC_PUT_u8(heartbeat_packet, 2, IOC_PAYLOAD_HEARTBEAT_LENGTH);
//     IOC_PUT_u8(heartbeat_packet, 3, IOC_MSG_TYPE_HEARTBEAT);
//     IOC_PUT_u8(heartbeat_packet, 4, 0x00);
//     /*计算校验和*/
//     uint16_t crc = crc_calculate(&heartbeat_packet[2], IOC_PAYLOAD_HEARTBEAT_LENGTH + 1);
//     IOC_PUT_u16(heartbeat_packet, 5, crc);
//     /*发送心跳数据包*/
//     IOC_Send_Packet(heartbeat_packet, IOC_MSG_HEARTBEAT_LENGTH);
// }

// /**
//  * @brief  发送电机值给协处理器
//  * @note
//  * @param  无
//  * @retval 无
//  */
// void IC_Comm_Send_Motor_Value(void)
// {
//     if (RC_Data_Decode_Done == SET)
//     {
//         RC_Data_Decode_Done = RESET;
//         /*提取RC数据*/
//         IOC_Extract_RC_Data();
//         /*发送电机数据包*/
//         IOC_Send_Motor_Value();
//         IOC_Send_Heartbeat();
//     }
// }
