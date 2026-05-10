#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "fdcan.h"

#ifndef CAN_BUS_TX_TIMEOUT_MS
#define CAN_BUS_TX_TIMEOUT_MS 10U
#endif

#define CAN_BUS_MAX_CLASSIC_DATA_LEN 8U

typedef enum {
    CAN_BUS_OK = 0,
    CAN_BUS_ERROR,
    CAN_BUS_BUSY,
    CAN_BUS_TIMEOUT,
    CAN_BUS_INVALID_ARG,
} can_bus_status_t;

typedef enum {
    CAN_BUS_ID_STD = 0,
    CAN_BUS_ID_EXT = 1,
} can_bus_id_type_t;

typedef struct {
    uint32_t id;
    can_bus_id_type_t id_type;
    uint8_t len;
    uint8_t data[CAN_BUS_MAX_CLASSIC_DATA_LEN];
} can_bus_frame_t;

uint32_t can_bus_len_to_dlc(uint8_t len);
uint8_t  can_bus_dlc_to_len(uint32_t dlc);

can_bus_status_t can_bus_init(FDCAN_HandleTypeDef* hfdcan);
can_bus_status_t can_bus_send(FDCAN_HandleTypeDef* hfdcan, const can_bus_frame_t* frame, uint32_t timeout_ms);
can_bus_status_t can_bus_send_std(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);
can_bus_status_t can_bus_send_ext(FDCAN_HandleTypeDef* hfdcan, uint32_t ext_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);
can_bus_status_t can_bus_receive_fifo0(FDCAN_HandleTypeDef* hfdcan, can_bus_frame_t* frame);

void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle);
void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);

#endif
