/**
 ******************************************************************************
 * @file    ic_protocol.c
 * @author  kai
 * @version
 * @data    2025/06/30
 * @brief   Inter-Chip Communication，实现协议打包和解包逻辑
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ioc_protocol.h"

// #include "pro_common.h"
#include "checksum.h"
// #include <string.h>
// #include "bsp_usart.h"
// #include "bsp_systick.h"
// #include "com_data.h"
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// /* 环形缓冲区管理结构体 */
// ringbuff_t IOC_RX_RingBuffMgr;

// /*IOC-协议解析状态*/
// IOC_parser_state_t IOC_Parser_State = IOC_STATE_SYNC1;

/*发送完成标志*/
// FlagStatus IOC_Send_Done_Flag = SET;
// /*遥控器数据解码完成标志*/
// FlagStatus RC_Data_Decode_Done = RESET;
// /*心跳包解码完成标志*/
// FlagStatus Heartbeat_Decode_Done = RESET;

/**
 * @defgroup 全局变量
 * @brief
 * @{
 */

// /*安全开关*/
// uint8_t safety_switch;

// /*电机控制值*/
// uint16_t motor_value[4];
/**
 * @}
 */

/**
 * @defgroup IOC_RX_Buffer
 * @brief 数据包接收相关状态变量与缓冲区
 * @{
 */
// /*IOC(双机通信一帧)数据缓冲区*/
// uint8_t IOC_RX_Buff[IOC_BUFFER_SIZE];
/*发送数据缓冲区*/
// /*需要通过usart发送到数据，最好放入全局变量中，以防发送未完成，局部变量被释放，发送数据错误*/
// static uint8_t IOC_TX_Buff[IOC_BUFFER_SIZE];
/*接收数据缓冲区，存放解析过程中接收的数据*/
// uint8_t IOC_RX_Buff_Temp[IOC_BUFFER_SIZE];
/*有效载荷长度*/
// uint16_t IOC_Payload_Length;
/*有效载荷索引*/
// uint16_t IOC_Payload_Index;
/**
 * @}
 */

/**
 * @defgroup IOC_PUT_DATA
 * @brief 将数据放入缓冲区，这样编写优势见笔记<<将数据放入缓冲区>>
 * @{
 */
/*将数据放入缓冲区*/
#define IOC_PUT_u8(buf, offset, data) buf[offset] = (uint8_t)(data)
#define IOC_PUT_u16(buf, offset, data)                                          \
    do                                                                          \
    {                                                                           \
        (buf)[(offset)] = (uint8_t)((data) & 0xFF);            /* 写入低字节 */ \
        (buf)[(offset) + 1] = (uint8_t)(((data) >> 8) & 0xFF); /* 写入高字节 */ \
    } while (0)
/*下面的数据存放方式存在字节对齐问题(见笔记)，可能造成硬件错误*/
// #define IOC_PUT_u16(buf,offset,data)   *(uint16_t *)(&buf[offset]) = (uint16_t)(data)
// #define IOC_PUT_u32(buf,offset,data)   *(uint32_t *)(&buf[offset]) = (uint32_t)(data)
// #define IOC_PUT_i8(buf,offset,data)   buf[offset] = (int8_t)(data)
// #define IOC_PUT_i16(buf,offset,data)   *(int16_t *)(&buf[offset]) = (int16_t)(data)
// #define IOC_PUT_i32(buf,offset,data)   *(int32_t *)(&buf[offset]) = (int32_t)(data)
// #define IOC_PUT_float(buf,offset,data)   *(float *)(&buf[offset]) = (float)(data)
// #define IOC_PUT_double(buf,offset,data)   *(double *)(&buf[offset]) = (double)(data)
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  IOC-环形缓冲区初始化
 * @note
 * @param  无
 * @retval 无
 */
static void ioc_rb_init(IOC_handle_t *handle)
{
    rb_init(handle->rx_rbbuff, sizeof(handle->rx_rbbuff), &handle->rx_rbbuff_mgr);
}

/**
 * @brief  IOC-环形缓冲区清空
 * @note
 * @param  无
 * @retval 无
 */
static void ioc_rb_clear(IOC_handle_t *handle)
{
    rb_clear(&handle->rx_rbbuff_mgr);
}

/**
 * @brief  IOC-环形缓冲区溢出判断
 * @note
 * @param  无
 * @retval 溢出为SET，反之为RESET
 */
static FlagStatus ioc_rb_is_overflow(IOC_handle_t *handle)
{
    return handle->rx_rbbuff_mgr.overflow;
}

/**
 * @brief  IOC-环形缓冲区数据读取
 * @note
 * @param  无
 * @retval 无
 */
static uint8_t ioc_rb_pop(IOC_handle_t *handle)
{
    uint8_t ret;
    rb_pop(&handle->rx_rbbuff_mgr, &ret);
    return ret;
}

/**
 * @brief  IOC-环形缓冲区是否有新数据
 * @note
 * @param  无
 * @retval 有新数据为true，反之为false
 */
static bool ioc_rb_has_new(IOC_handle_t *handle)
{
    return !rb_IsEmpty(&handle->rx_rbbuff_mgr);
}

/**
 * @brief  IOC-协议解析状态机初始化
 * @note
 * @param  无
 * @retval 无
 */
static void ioc_parser_init(IOC_handle_t *handle)
{
    handle->parser_state = IOC_STATE_SYNC1;
    handle->rx_frame_len = 0;
}

/**
 * @brief  IOC收发初始化(串口初始化后调用)
 * @note
 * @param  无
 * @retval 无
 */
void ioc_init(IOC_handle_t *handle,
              uint8_t *rx_rbbuff, uint8_t *tx_buff, uint8_t *rx_frame)
{
    handle->rx_rbbuff = rx_rbbuff;
    handle->rx_frame = rx_frame;
    handle->tx_buff = tx_buff;

    ioc_rb_init(handle);
    ioc_parser_init(handle);
}

/**
 * @brief  重置数据包相关的状态变量与缓冲区
 * @note
 * @param  无
 * @retval 无
 */
static void ioc_payload_reset(IOC_handle_t *handle)
{
    /*将数据包相关的状态变量重置到已知状态*/
    handle->rx_frame_len = 0;
    handle->rx_frame_index = 0;
    memset(handle->rx_frame, 0, sizeof(handle->rx_frame));
}

/**
 * @brief  处理接收到的HEARTBEAT数据,提取安全开关的值
 * @note
 * @param
 * @retval 无
 */
// static void IOC_Process_Heartbeat(IOC_handle_t *handle, uint8_t *payload)
// {
//     safety_switch = payload[0];
//     handle->heartbeat_switch = safety_switch;
//     handle->heartbeat_decode_done = SET;
// }

/**
 * @brief  发送IOC数据包
 * @note
 * @param  无
 * @retval 无
 */
static void ioc_send_packet(IOC_handle_t *handle, uint8_t *data, uint16_t length)
{

    while (handle->send_done_flag == RESET)
    {
    };
    handle->send_done_flag = RESET;
    /*这里将数据放入了全局变量中，防止发送未完成，局部变量被释放，发送数据错误*/
    /*需要将转移数据放在检查发送完成标志之后，防止连续发送不同数据包时竞态条件导致的数据覆盖*/
    memcpy(handle->tx_buff, data, length);
    USART_DMA_Send(DMA2_Stream6, handle->tx_buff, length);
}

/**
 * @brief  处理接收到的数据
 * @note
 * @param  data: 接收到的数据
 * @retval 无
 */
bool ioc_parser(IOC_handle_t *handle)
{
    uint16_t crc;
    /*检查溢出*/
    if (ioc_rb_is_overflow(handle) == SET)
    {
        ioc_rb_clear(handle);
        IOC_DEBUG("ioc_rb_is_overflow\r\n");
    }
    // if (ioc_rb_has_new(handle))
    /*由于主循环中包括日志记录，可能极耗时间，使用if时可能导致解析器效率不足，即使将ioc_parser也放在主循环中*/
    /*也可能因日志记录任务耗费大量时间而导致缓冲区溢出*/
    while (ioc_rb_has_new(handle))
    {
        /*读取缓冲区*/
        uint8_t data;
        data = ioc_rb_pop(handle);
        switch (handle->parser_state)
        {
        case IOC_STATE_SYNC1:
            if (data == IOC_SYNC1)
            {
                handle->parser_state = IOC_STATE_SYNC2;
            }
            else
            {
                IOC_DEBUG("IOC_STATE_SYNC1 error\r\n");
            }
            break;
        case IOC_STATE_SYNC2:
            if (data == IOC_SYNC2)
            {
                handle->parser_state = IOC_STATE_LENGTH;
            }
            else
            {
                ioc_parser_init(handle);
                IOC_DEBUG("IOC_STATE_SYNC2 error\r\n");
            }
            break;
        case IOC_STATE_LENGTH:
            ioc_payload_reset(handle);
            handle->rx_frame_len = data;
            handle->rx_frame_index = 0;
            handle->rx_frame[handle->rx_frame_index++] = data;
            handle->parser_state = IOC_STATE_PAYLOAD;
            break;
        case IOC_STATE_PAYLOAD:
            handle->rx_frame[handle->rx_frame_index++] = data;
            if (handle->rx_frame_index == handle->rx_frame_len + 1)
            {
                handle->parser_state = IOC_STATE_CCK_A;
            }
            break;
        case IOC_STATE_CCK_A:
            handle->rx_frame[handle->rx_frame_index++] = data;
            handle->parser_state = IOC_STATE_CCK_B;
            break;
        case IOC_STATE_CCK_B:
            handle->rx_frame[handle->rx_frame_index++] = data;
            crc = (uint16_t)(handle->rx_frame[handle->rx_frame_len + 2] << 8 | handle->rx_frame[handle->rx_frame_len + 1]);
            /*计算校验和*/
            if (crc_calculate(handle->rx_frame, handle->rx_frame_len + 1) == crc)
            {
                // IOC_Decode(handle, &handle->rx_frame[1]);
                return true;/*接收到一阵完整数据返回处理*/
            }
            else
            {
                IOC_DEBUG("CRC error\r\n");
            }
            ioc_payload_reset(handle); /*调试需要*/
            handle->parser_state = IOC_STATE_SYNC1;
            break;
        default:
            handle->parser_state = IOC_STATE_SYNC1;
            IOC_DEBUG("IOC_STATE_DEFAULT error\r\n");
            ioc_payload_reset(handle);
            break;
        }
    }
    return false;
}

/**
 * @brief 获取接收到的消息类型
 * 
 * @param handle - 协议句柄指针
 * @retval ioc_msg_type_t - 消息类型
 */
IOC_msg_type_t ioc_decode_msg_type(IOC_handle_t *handle)
{
    return (IOC_msg_type_t)handle->rx_frame[1];
}

/**
 * @brief 获取的心跳包
 * 
 */
void ioc_decode_heartbeat(IOC_handle_t *handle, bool *heartbeat_switch)
{
    /*检查数据长度是否正确*/
    if (handle->rx_frame_len != IOC_PAYLOAD_HEARTBEAT_LENGTH)
    {
        IOC_DEBUG("IOC_PAYLOAD_HEARTBEAT_LENGTH error\r\n");
        return;
    }
    /*下面的payload数据就是一个变字长数据，如果传入的rx_frame_len=0，那payload会是一个相当大的数组，造成栈溢出*/
    // uint8_t payload[handle->rx_frame_len - 1];
    // /*获取心跳包数据*/
    // memcpy(payload, handle->rx_frame + 2, handle->rx_frame_len - 1);
    const uint8_t *payload = &handle->rx_frame[2]; 
    /*获取安全开关状态*/
    *heartbeat_switch = payload[0];
    /*设置标志位*/
    handle->heartbeat_decode_done = SET;
}



/**
 * @brief 获取接收到的RC数据
 * 
 * @param handle - 协议句柄指针
 * @param rc_raw - 接收的RC数据指针
 */
void ioc_decode_rc(IOC_handle_t *handle, rc_raw_data_t *rc_raw)
{
    /*检查数据长度是否正确*/
    if (handle->rx_frame_len != IOC_PAYLOAD_RC_LENGTH)
    {
        IOC_DEBUG("IOC_PAYLOAD_RC_LENGTH error\r\n");
        return;
    }
    // uint8_t payload[handle->rx_frame_len - 1];
    // /*rx_frame第一个字节为消息长度,第二个字节为消息类型，需要跳过*/
    // memcpy(payload, handle->rx_frame + 2, handle->rx_frame_len - 1);
    const uint8_t *payload = &handle->rx_frame[2]; 
    /*获取rc数据*/
    for (uint8_t i = 0; i < RC_CHANNEL_NUM; i++)
    {
        rc_raw->channel[i] = (uint16_t)(payload[i * 2] | payload[i * 2 + 1] << 8);/*小端存储*/
    }
    /*设置标志位*/
    handle->rc_decode_done = SET;
}




// /**
//  * @brief  打包电机控制值，输出到协处理器
//  * @note   这里只是通过数据发送
//  * @param  无
//  * @retval 无
//  */
// static void IOC_Send_Motor_Value(void)
// {
//     /*打包电机控制值*/
//     uint8_t motor_packet[IOC_MSG_PWM_LENGTH];
//     IOC_PUT_u8(motor_packet, 0, IOC_SYNC1);
//     IOC_PUT_u8(motor_packet, 1, IOC_SYNC2);
//     IOC_PUT_u8(motor_packet, 2, IOC_PAYLOAD_PWM_LENGTH);
//     IOC_PUT_u8(motor_packet, 3, IOC_MSG_TYPE_PWM_DATA);
//     IOC_PUT_u16(motor_packet, 4, motor_value[0]);
//     IOC_PUT_u16(motor_packet, 6, motor_value[1]);
//     IOC_PUT_u16(motor_packet, 8, motor_value[2]);
//     IOC_PUT_u16(motor_packet, 10, motor_value[3]);
//     /*计算校验和*/
//     uint16_t crc = crc_calculate(&motor_packet[2], IOC_PAYLOAD_PWM_LENGTH + 1);
//     IOC_PUT_u16(motor_packet, 12, crc);
//     /*发送电机数据包*/
//     IOC_Send_Packet(motor_packet, IOC_MSG_PWM_LENGTH);
// }

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



// /**
//  * @brief  双机通信任务
//  * @note
//  * @param  无
//  * @retval 无
//  */
// void IC_Comm_Task(void)
// {
//     IC_Comm_Parser();
// }

// /**
//  * @brief  测试双机通信
//  * @note
//  * @param  无
//  * @retval 无
//  */
// uint8_t test_data1;
// void IOC_Comm_Test(void)
// {
//     static uint32_t heartbeat_last_time = 0;
//     //		while (IOC_RB_HasNew())
//     //    {
//     //        /*读取缓冲区*/
//     //        uint8_t data;
//     //        data = IOC_RB_Pop();
//     //				IOC_Send_Packet(&data, 1);
//     //				printf("%d ", data);
//     //		}
//     // uint8_t arr[10] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0xAA, 0x55, 0x01, 0x02, 0x04};
//     // while (1)
//     // {

//     //     IOC_Send_Packet(arr, 10);
//     //     delay_ms(50);
//     // }
//     /*发送*/
//     /*发送心跳包*/
//     /*发送电机数据包*/
//     IC_Comm_Send_Motor_Value();
//     /*接收*/
//     IC_Comm_Parser();
//     if (Heartbeat_Decode_Done == SET)
//     {
//         Heartbeat_Decode_Done = RESET;
//         heartbeat_last_time = GetTick();
//     }
//     if (Heartbeat_Decode_Done == RESET)
//     {
//         if (GetTick() - heartbeat_last_time > 500)
//         {
//             //    IOC_DEBUG("F1信号丢失\r\n");
//         }
//     }
// }

/**
 * @}
 */
