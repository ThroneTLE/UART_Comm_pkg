/**
  ******************************************************************************
  * @file    data_communication_pkg.c
  * @brief   通用数据通信协议库实现
  * @version V2.1.0
  * @date    2025-07-21
  ******************************************************************************
  */

#include "data_communication_pkg.h"
#include <string.h>
#include "main.h"
#include "usart.h"
/* ========================= 私有类型定义 ========================= */
/**
  * @brief  解析状态机状态定义
  */
typedef enum {
    STATE_WAIT_HEADER1,       // 等待帧头第1字节
    STATE_WAIT_HEADER2,       // 等待帧头第2字节
    STATE_WAIT_LENGTH_HIGH,   // 等待长度高字节
    STATE_WAIT_LENGTH_LOW,    // 等待长度低字节
    STATE_WAIT_CMD,           // 等待命令字节
    STATE_READ_DATA,          // 读取数据载荷
    STATE_WAIT_CRC1,          // 等待CRC高字节
    STATE_WAIT_CRC2,          // 等待CRC低字节
    STATE_WAIT_END1,          // 等待帧尾第1字节
    STATE_WAIT_END2           // 等待帧尾第2字节
} ParseState;

/**
  * @brief  解析上下文结构体
  */
typedef struct {
    ParseState state;                    // 当前解析状态
    uint16_t data_index;                 // 数据索引
    uint16_t pkg_length;                 // 数据包长度（CMD+DATA）
    uint8_t cmd;                         // 命令字节
    uint8_t data[MAX_DATA_LENGTH];       // 数据缓冲区
    uint16_t recv_crc;                   // 接收到的CRC
    uint8_t crc_buffer[MAX_DATA_LENGTH + 3]; // CRC计算缓冲区
    uint16_t crc_index;                  // CRC缓冲区索引
} ParseContext;

/* ========================= 私有变量 ========================= */
static ParseContext g_ctx;

/* ========================= 私有函数 ========================= */
/**
  * @brief  CRC16-CCITT计算
  * @param  data : 数据缓冲区
  * @param  len  : 数据长度
  * @retval CRC16值
  * @note   多项式: 0x1021, 初始值: 0xFFFF
  */
static uint16_t crc16_ccitt(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ========================= API函数实现 ========================= */
/**
  * @brief  初始化通信协议模块
  * @param  无
  * @retval 无
  */
void data_comm_init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.state = STATE_WAIT_HEADER1;
}

/**
  * @brief  打包并发送数据
  * @param  cmd  : 命令字节（1字节）
  * @param  data : 数据载荷指针
  * @param  len  : 数据载荷长度（0~MAX_DATA_LENGTH）
  * @retval 实际发送的字节数
  */
uint16_t data_comm_send(uint8_t cmd, uint8_t *data, uint16_t len)
{
    static uint8_t buffer[MAX_DATA_LENGTH + 10]; // 最大帧长度
    uint16_t index = 0;
    uint16_t total_len;
    
    /* 参数检查 */
    if (len > MAX_DATA_LENGTH) {
        return 0;
    }
    
    /* 1. 帧头（2字节） */
    buffer[index++] = (FRAME_HEADER >> 8) & 0xFF;
    buffer[index++] = FRAME_HEADER & 0xFF;
    
    /* 2. 长度字段（2字节，表示CMD+DATA的总长度） */
    total_len = len + 1;  // +1 for CMD
    buffer[index++] = (total_len >> 8) & 0xFF;
    buffer[index++] = total_len & 0xFF;
    
    /* 3. 命令字节（1字节） */
    buffer[index++] = cmd;
    
    /* 4. 数据载荷（len字节） */
    if (data && len > 0) {
        memcpy(&buffer[index], data, len);
        index += len;
    }
    
#if USE_CRC16
    /* 5. CRC16校验（2字节）- 从长度字段开始计算 */
    uint16_t crc = crc16_ccitt(&buffer[2], index - 2);
    buffer[index++] = (crc >> 8) & 0xFF;
    buffer[index++] = crc & 0xFF;
#endif
    
    /* 6. 帧尾（2字节） */
    buffer[index++] = (FRAME_END >> 8) & 0xFF;
    buffer[index++] = FRAME_END & 0xFF;
    
    /* 调用用户发送函数 */
    user_transmit(buffer, index);
    
    return index;
}

/**
  * @brief  解析接收到的单个字节
  * @param  byte : 接收到的字节
  * @retval 无
  * @note   将接收到的每个字节传入此函数，内部自动进行协议解析
  */
void data_comm_parse_byte(uint8_t byte)
{
    switch (g_ctx.state) {
        case STATE_WAIT_HEADER1:
            if (byte == ((FRAME_HEADER >> 8) & 0xFF)) {
                g_ctx.state = STATE_WAIT_HEADER2;
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == (FRAME_HEADER & 0xFF)) {
                g_ctx.state = STATE_WAIT_LENGTH_HIGH;
                g_ctx.crc_index = 0;
            } else {
                g_ctx.state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_LENGTH_HIGH:
            g_ctx.pkg_length = byte << 8;
            g_ctx.crc_buffer[g_ctx.crc_index++] = byte;
            g_ctx.state = STATE_WAIT_LENGTH_LOW;
            break;
            
        case STATE_WAIT_LENGTH_LOW:
            g_ctx.pkg_length |= byte;
            g_ctx.crc_buffer[g_ctx.crc_index++] = byte;
            
            /* 长度检查 */
            if (g_ctx.pkg_length == 0 || g_ctx.pkg_length > (MAX_DATA_LENGTH + 1)) {
                g_ctx.state = STATE_WAIT_HEADER1;
                return;
            }
            g_ctx.state = STATE_WAIT_CMD;
            break;
            
        case STATE_WAIT_CMD:
            g_ctx.cmd = byte;
            g_ctx.crc_buffer[g_ctx.crc_index++] = byte;
            g_ctx.data_index = 0;
            
            /* 如果只有命令字节，没有数据 */
            if (g_ctx.pkg_length == 1) {
#if USE_CRC16
                g_ctx.state = STATE_WAIT_CRC1;
#else
                g_ctx.state = STATE_WAIT_END1;
#endif
            } else {
                g_ctx.state = STATE_READ_DATA;
            }
            break;
            
        case STATE_READ_DATA:
            g_ctx.data[g_ctx.data_index] = byte;
            g_ctx.crc_buffer[g_ctx.crc_index++] = byte;
            g_ctx.data_index++;
            
            /* 数据接收完成 */
            if (g_ctx.data_index >= (g_ctx.pkg_length - 1)) {
#if USE_CRC16
                g_ctx.state = STATE_WAIT_CRC1;
#else
                g_ctx.state = STATE_WAIT_END1;
#endif
            }
            break;
            
#if USE_CRC16
        case STATE_WAIT_CRC1:
            g_ctx.recv_crc = byte << 8;
            g_ctx.state = STATE_WAIT_CRC2;
            break;
            
        case STATE_WAIT_CRC2:
            g_ctx.recv_crc |= byte;
            
            /* CRC校验 */
            uint16_t calc_crc = crc16_ccitt(g_ctx.crc_buffer, g_ctx.crc_index);
            if (calc_crc != g_ctx.recv_crc) {
                /* CRC错误，重新开始 */
                g_ctx.state = STATE_WAIT_HEADER1;
                return;
            }
            g_ctx.state = STATE_WAIT_END1;
            break;
#endif
            
        case STATE_WAIT_END1:
            if (byte == ((FRAME_END >> 8) & 0xFF)) {
                g_ctx.state = STATE_WAIT_END2;
            } else {
                g_ctx.state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_END2:
            if (byte == (FRAME_END & 0xFF)) {
                /* 完整数据包接收成功，调用用户处理函数 */
                user_packet_handler(g_ctx.cmd, g_ctx.data, g_ctx.data_index);
            }
            g_ctx.state = STATE_WAIT_HEADER1;
            break;
            
        default:
            g_ctx.state = STATE_WAIT_HEADER1;
            break;
    }
}

/* ========================= 用户需要实现的函数 ========================= */
/**
  * @brief  数据发送函数（用户必须实现）
  * @param  data : 待发送数据缓冲区
  * @param  len  : 数据长度
  * @retval 无
  * @note   用户需根据实际硬件实现此函数
  *         示例：HAL_UART_Transmit(&huart1, data, len, 100);
  */
void user_transmit(uint8_t *data, uint16_t len)
{
    /* 此函数需要用户根据实际硬件实现 */
    HAL_UART_Transmit(&huart2, data, len, 100);
}

/**
  * @brief  数据包接收回调函数（用户必须实现）
  * @param  cmd  : 接收到的命令字节
  * @param  data : 数据载荷缓冲区
  * @param  len  : 数据载荷长度
  * @retval 无
  * @note   当接收到完整数据包时被调用
  */
void user_packet_handler(uint8_t cmd, uint8_t *data, uint16_t len)
{
    /* 此函数需要用户实现数据包处理逻辑 */
    switch(cmd) {
        case 0x01:  // 传感器数据

            break;
        case 0x02:  // LED控制
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, data[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
    }
}
