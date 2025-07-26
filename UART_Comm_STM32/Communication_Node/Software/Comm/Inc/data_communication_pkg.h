/**
  ******************************************************************************
  * @file    data_communication_pkg.h
  * @brief   通用数据通信协议库
  * @version V2.1.0
  * @date    2025-07-21
  ******************************************************************************
  */

#ifndef __DATA_COMMUNICATION_PKG_H
#define __DATA_COMMUNICATION_PKG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 协议参数配置 */
#define FRAME_HEADER      0xAA55      // 帧头
#define FRAME_END         0x55AA      // 帧尾
#define MAX_DATA_LENGTH   256         // 最大数据长度
#define USE_CRC16         1           // 启用CRC16校验

/* 数据包状态枚举 */
typedef enum {
    PKG_OK = 0,
    PKG_HEADER_ERR,
    PKG_LENGTH_ERR,
    PKG_CRC_ERR,
    PKG_END_ERR
} PkgStatus;

/* API函数接口 */
void data_comm_init(void);
uint16_t data_comm_send(uint8_t cmd, uint8_t *data, uint16_t len);
void data_comm_parse_byte(uint8_t byte);

/* 用户实现接口 */
void user_transmit(uint8_t *data, uint16_t len);
void user_packet_handler(uint8_t cmd, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_COMMUNICATION_PKG_H */
