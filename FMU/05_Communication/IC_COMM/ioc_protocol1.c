#include "ioc_protocol1.h"
#include <string.h>

/* 内部函数声明 */
static uint8_t IOC_CalCRC8(uint8_t *p_data, uint16_t len);
static void IOC_RingBuff_Push(IOC_RingBuff_t *fifo, uint8_t data);
static bool IOC_RingBuff_Pop(IOC_RingBuff_t *fifo, uint8_t *data);
static bool IOC_RingBuff_IsEmpty(IOC_RingBuff_t *fifo);

/**
 * @brief 初始化协议栈
 * @param protocol 协议句柄指针
 */
void IOC_Protocol_Init(IOC_Protocol_t *protocol)
{
    if (protocol == NULL)
        return;

    /* 初始化FIFO */
    protocol->rx_fifo.head = 0;
    protocol->rx_fifo.tail = 0;
    protocol->rx_fifo.count = 0;
    memset(protocol->rx_fifo.buffer, 0, IOC_RX_BUFF_SIZE);

    /* 初始化状态机 */
    protocol->state = IOC_STATE_WAIT_HEADER0;
    protocol->rx_index = 0;

    /* 初始化统计 */
    protocol->rx_packets = 0;
    protocol->rx_errors = 0;
}

/**
 * @brief 将接收到的字节压入FIFO
 * @param protocol 协议句柄指针
 * @param data 接收到的字节
 */
void IOC_Protocol_PushByte(IOC_Protocol_t *protocol, uint8_t data)
{
    if (protocol == NULL)
        return;
    IOC_RingBuff_Push(&protocol->rx_fifo, data);
}

/**
 * @brief 解析协议数据
 * @param protocol 协议句柄指针
 * @return true: 解析到一帧完整数据, false: 未解析到
 */
bool IOC_Protocol_Parse(IOC_Protocol_t *protocol)
{
    uint8_t byte = 0;

    if (protocol == NULL)
        return false;

    /* 处理FIFO中的所有数据 */
    while (!IOC_RingBuff_IsEmpty(&protocol->rx_fifo))
    {
        /* 从FIFO取出一个字节 */
        if (!IOC_RingBuff_Pop(&protocol->rx_fifo, &byte))
            break;

        switch (protocol->state)
        {
        case IOC_STATE_WAIT_HEADER0:
            if (byte == IOC_FRAME_HEADER_0)
            {
                protocol->state = IOC_STATE_WAIT_HEADER1;
            }
            break;

        case IOC_STATE_WAIT_HEADER1:
            if (byte == IOC_FRAME_HEADER_1)
            {
                protocol->state = IOC_STATE_WAIT_LEN;
            }
            else if (byte == IOC_FRAME_HEADER_0)
            {
                /* 可能是重复的Header0 */
                protocol->state = IOC_STATE_WAIT_HEADER1;
            }
            else
            {
                protocol->state = IOC_STATE_WAIT_HEADER0;
            }
            break;

        case IOC_STATE_WAIT_LEN:
            if (byte <= IOC_MAX_PAYLOAD_LEN)
            {
                protocol->rx_frame.len = byte;
                protocol->state = IOC_STATE_WAIT_ID;
            }
            else
            {
                /* 长度非法，重置 */
                protocol->rx_errors++;
                protocol->state = IOC_STATE_WAIT_HEADER0;
            }
            break;

        case IOC_STATE_WAIT_ID:
            protocol->rx_frame.cmd_id = byte;
            protocol->rx_index = 0;
            if (protocol->rx_frame.len > 0)
            {
                protocol->state = IOC_STATE_WAIT_PAYLOAD;
            }
            else
            {
                /* 无载荷，直接校验 */
                protocol->state = IOC_STATE_WAIT_CRC;
            }
            break;

        case IOC_STATE_WAIT_PAYLOAD:
            protocol->rx_frame.payload[protocol->rx_index++] = byte;
            if (protocol->rx_index >= protocol->rx_frame.len)
            {
                protocol->state = IOC_STATE_WAIT_CRC;
            }
            break;

        case IOC_STATE_WAIT_CRC:
        {
            /* 计算校验和: Len + ID + Payload */
            uint8_t cal_crc = 0;
            uint8_t check_buff[IOC_MAX_PAYLOAD_LEN + 2];

            check_buff[0] = protocol->rx_frame.len;
            check_buff[1] = protocol->rx_frame.cmd_id;
            if (protocol->rx_frame.len > 0)
            {
                memcpy(&check_buff[2], protocol->rx_frame.payload, protocol->rx_frame.len);
            }

            cal_crc = IOC_CalCRC8(check_buff, protocol->rx_frame.len + 2);

            if (cal_crc == byte)
            {
                /* 校验通过 */
                protocol->rx_packets++;
                protocol->state = IOC_STATE_WAIT_HEADER0;
                return true; /* 返回成功，调用者可以通过GetFrame获取数据 */
            }
            else
            {
                /* 校验失败 */
                protocol->rx_errors++;
                protocol->state = IOC_STATE_WAIT_HEADER0;
            }
        }
        break;

        default:
            protocol->state = IOC_STATE_WAIT_HEADER0;
            break;
        }
    }

    return false;
}

/**
 * @brief 打包数据帧
 * @param cmd_id 命令ID
 * @param p_data 数据指针
 * @param len 数据长度
 * @param tx_buff 发送缓冲区
 * @return 打包后的总长度
 */
uint16_t IOC_Protocol_Pack(uint8_t cmd_id, uint8_t *p_data, uint8_t len, uint8_t *tx_buff)
{
    uint16_t index = 0;
    uint8_t crc = 0;

    if (tx_buff == NULL)
        return 0;
    if (len > IOC_MAX_PAYLOAD_LEN)
        return 0;

    /* 帧头 */
    tx_buff[index++] = IOC_FRAME_HEADER_0;
    tx_buff[index++] = IOC_FRAME_HEADER_1;

    /* 长度 */
    tx_buff[index++] = len;

    /* 命令ID */
    tx_buff[index++] = cmd_id;

    /* 载荷 */
    if (len > 0 && p_data != NULL)
    {
        memcpy(&tx_buff[index], p_data, len);
        index += len;
    }

    /* 计算CRC (Len + ID + Payload) */
    /* 注意：tx_buff[2]是Len, tx_buff[3]是ID */
    crc = IOC_CalCRC8(&tx_buff[2], len + 2);
    tx_buff[index++] = crc;

    return index;
}

/**
 * @brief 获取解析到的帧数据
 * @param protocol 协议句柄指针
 * @return 帧数据指针
 */
IOC_Frame_t *IOC_Protocol_GetFrame(IOC_Protocol_t *protocol)
{
    if (protocol == NULL)
        return NULL;
    return &protocol->rx_frame;
}

/* ================= 内部辅助函数 ================= */

/**
 * @brief 简单的CRC8计算
 * @param p_data 数据指针
 * @param len 长度
 * @return CRC值
 */
static uint8_t IOC_CalCRC8(uint8_t *p_data, uint16_t len)
{
    uint8_t crc = 0;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        crc ^= p_data[i];
    }
    return crc;
}

/* ================= 环形缓冲区实现 ================= */

static void IOC_RingBuff_Push(IOC_RingBuff_t *fifo, uint8_t data)
{
    uint16_t next = (fifo->head + 1) % IOC_RX_BUFF_SIZE;

    if (next != fifo->tail)
    {
        fifo->buffer[fifo->head] = data;
        fifo->head = next;
        fifo->count++;
    }
    else
    {
        /* 缓冲区满，覆盖旧数据或丢弃，这里选择丢弃 */
    }
}

static bool IOC_RingBuff_Pop(IOC_RingBuff_t *fifo, uint8_t *data)
{
    if (fifo->head == fifo->tail)
    {
        return false; /* 空 */
    }

    *data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % IOC_RX_BUFF_SIZE;
    fifo->count--;
    return true;
}

static bool IOC_RingBuff_IsEmpty(IOC_RingBuff_t *fifo)
{
    return (fifo->head == fifo->tail);
}
