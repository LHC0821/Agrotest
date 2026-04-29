#ifndef _CAN_H
#define _CAN_H

#include "main.h"

/**
 * @brief CAN 接收回调函数指针类型
 * @param hfdcan CAN 句柄
 * @param header 接收消息的 CAN 帧头
 * @param data 接收到的 8 字节数据
 * @return 无
 */
typedef void (*can_rx_callback_t)(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef* header, uint8_t* data);

/**
 * @brief 根据数据长度获取 FDCAN 数据长度码 (DLC)
 * @param size 数据字节数
 * @return FDCAN DLC 值
 */
uint32_t get_fdcan_dlc(uint16_t size);
/**
 * @brief 根据 FDCAN 数据长度码 (DLC) 获取数据长度 (字节)
 * @param dlc FDCAN DLC 值
 * @return 数据字节数
 */
uint16_t get_fdcan_data_size(uint32_t dlc);
/**
 * @brief 初始化 CAN 过滤器
 * @param fdcanHandle CAN 句柄
 * @return 无
 */
void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle);
/**
 * @brief 发送 CAN 消息
 * @param hfdcanx CAN 句柄
 * @param id CAN 消息 ID
 * @param data 要发送的数据缓冲区
 * @param len 数据长度 (字节)
 * @return HAL 状态码
 */
HAL_StatusTypeDef can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);

/**
 * @brief 注册 CAN 接收回调函数
 * @param hfdcan CAN 句柄
 * @param callback 回调函数指针
 * @return 无
 */
void can_rx_callback_register(FDCAN_HandleTypeDef* hfdcan, can_rx_callback_t callback);

#endif
