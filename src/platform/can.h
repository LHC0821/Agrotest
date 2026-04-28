#ifndef _CAN_H
#define _CAN_H


#include "main.h"

uint32_t get_fdcan_dlc(uint16_t size);
uint16_t get_fdcan_data_size(uint32_t dlc);
void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle);
void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);
void register_can_rx_callback(FDCAN_HandleTypeDef* hfdcanx, void (*callback)(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifox_its));


#endif
