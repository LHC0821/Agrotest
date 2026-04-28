#ifndef _CAN_H
#define _CAN_H

#include "main.h"

typedef void (*can_rx_callback_t)(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef* header, uint8_t* data);

uint32_t get_fdcan_dlc(uint16_t size);
uint16_t get_fdcan_data_size(uint32_t dlc);
void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle);
void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);

// 新增：注册回调函数接口
void can_rx_callback_register(can_rx_callback_t callback);

#endif