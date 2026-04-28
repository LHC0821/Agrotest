#include "can.h"

static void(*can_rx_callback[2])(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifox_its) = { NULL, NULL };

FDCAN_TxHeaderTypeDef TxHeader =
{
    .TxFrameType = FDCAN_DATA_FRAME,            // 数据帧
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,    // 错误指示状态
    .BitRateSwitch = FDCAN_BRS_OFF,             // 比特率切换关闭
    .FDFormat = FDCAN_CLASSIC_CAN,              // 经典 CAN 格式
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,   // 不使用发送事件 FIFO
    .MessageMarker = 0,                         // 消息标记
};

#define MAX_CAN_CALLBACKS 10
static can_rx_callback_t g_can_rx_callbacks[MAX_CAN_CALLBACKS];
static uint8_t g_callback_count = 0;

/**
 * @brief 注册 CAN 接收回调函数 (所有 FDCAN 共用分发)
 */
void can_rx_callback_register(can_rx_callback_t callback) {
    if (g_callback_count < MAX_CAN_CALLBACKS && callback != NULL) {
        g_can_rx_callbacks[g_callback_count++] = callback;
    }
}

/**
 * @brief 统一的分发逻辑 (内部函数)
 */
static void can_dispatch_callbacks(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifoITs, uint32_t fifo) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 获取消息
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) == HAL_OK) {
        for (uint8_t i = 0; i < g_callback_count; i++) {
            if (g_can_rx_callbacks[i] != NULL) {
                g_can_rx_callbacks[i](hfdcan, &rx_header, rx_data);
            }
        }
    }
}

uint32_t get_fdcan_dlc(uint16_t size) {
    uint32_t fdcan_dlc = 0;

    if(size == 0) {
        fdcan_dlc = FDCAN_DLC_BYTES_0;
    }
    else if(size <= 1) {
        fdcan_dlc = FDCAN_DLC_BYTES_1;
    }
    else if(size <= 2) {
        fdcan_dlc = FDCAN_DLC_BYTES_2;
    }
    else if(size <= 3) {
        fdcan_dlc = FDCAN_DLC_BYTES_3;
    }
    else if(size <= 4) {
        fdcan_dlc = FDCAN_DLC_BYTES_4;
    }
    else if(size <= 5) {
        fdcan_dlc = FDCAN_DLC_BYTES_5;
    }
    else if(size <= 6) {
        fdcan_dlc = FDCAN_DLC_BYTES_6;
    }
    else if(size <= 7) {
        fdcan_dlc = FDCAN_DLC_BYTES_7;
    }
    else if(size <= 8) {
        fdcan_dlc = FDCAN_DLC_BYTES_8;
    }
    else if(size <= 12) {
        fdcan_dlc = FDCAN_DLC_BYTES_12;
    }
    else if(size <= 16) {
        fdcan_dlc = FDCAN_DLC_BYTES_16;
    }
    else if(size <= 20) {
        fdcan_dlc = FDCAN_DLC_BYTES_20;
    }
    else if(size <= 24) {
        fdcan_dlc = FDCAN_DLC_BYTES_24;
    }
    else if(size <= 32) {
        fdcan_dlc = FDCAN_DLC_BYTES_32;
    }
    else if(size <= 48) {
        fdcan_dlc = FDCAN_DLC_BYTES_48;
    }
    else if(size <= 64) {
        fdcan_dlc = FDCAN_DLC_BYTES_64;
    }
    return fdcan_dlc;
}


uint16_t get_fdcan_data_size(uint32_t dlc) {
    uint16_t size = 0;

    switch(dlc) {
        case FDCAN_DLC_BYTES_0:
            size = 0;
            break;
        case FDCAN_DLC_BYTES_1:
            size = 1;
            break;
        case FDCAN_DLC_BYTES_2:
            size = 2;
            break;
        case FDCAN_DLC_BYTES_3:
            size = 3;
            break;
        case FDCAN_DLC_BYTES_4:
            size = 4;
            break;
        case FDCAN_DLC_BYTES_5:
            size = 5;
            break;
        case FDCAN_DLC_BYTES_6:
            size = 6;
            break;
        case FDCAN_DLC_BYTES_7:
            size = 7;
            break;
        case FDCAN_DLC_BYTES_8:
            size = 8;
            break;
        case FDCAN_DLC_BYTES_12:
            size = 12;
            break;
        case FDCAN_DLC_BYTES_16:
            size = 16;
            break;
        case FDCAN_DLC_BYTES_20:
            size = 20;
            break;
        case FDCAN_DLC_BYTES_24:
            size = 24;
            break;
        case FDCAN_DLC_BYTES_32:
            size = 32;
            break;
        case FDCAN_DLC_BYTES_48:
            size = 48;
            break;
        case FDCAN_DLC_BYTES_64:
            size = 64;
            break;
        default:
            break;
    }

    return size;
}


void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle) {
    if(HAL_FDCAN_ConfigGlobalFilter(fdcanHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    // 核心修复：同时使能 FIFO0 和 FIFO1 的新消息通知
    if(HAL_FDCAN_ActivateNotification(fdcanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK) {
        Error_Handler();
    }
    
    HAL_FDCAN_ConfigTxDelayCompensation(fdcanHandle, fdcanHandle->Init.DataPrescaler * fdcanHandle->Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(fdcanHandle);

    if(HAL_FDCAN_Start(fdcanHandle) != HAL_OK) {
        Error_Handler();
    }
}


// void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len) {
//     TxHeader.Identifier = id;


//     if(id > 0x7ff) {
//         TxHeader.IdType = FDCAN_EXTENDED_ID;
//     }
//     else {

//         TxHeader.IdType = FDCAN_STANDARD_ID;
//     }

//     TxHeader.DataLength = get_fdcan_dlc(len);
//     HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &TxHeader, data);
// }

void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len) {
    // 每次发送时在栈上创建独立的 Header
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN; // RS06 使用经典模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    tx_header.Identifier = id;

    // 2. 严谨判断 ID 类型
    if(id > 0x7ff) {
        tx_header.IdType = FDCAN_EXTENDED_ID;
    }
    else {
        tx_header.IdType = FDCAN_STANDARD_ID;
    }

    tx_header.DataLength = get_fdcan_dlc(len);

    // 3. 检查 FIFO 状态，防止由于发送过快导致的丢失
    uint32_t fill_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcanx);
    if(fill_level > 0) {
        if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &tx_header, data) != HAL_OK) {
            // 发送失败处理，可以在此处断点调试
        }
    }
}

void register_can_rx_callback(FDCAN_HandleTypeDef* hfdcanx, void (*callback)(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifox_its)) {
    if(hfdcanx->Instance == FDCAN1) can_rx_callback[0] = callback;
    else if(hfdcanx->Instance == FDCAN2) can_rx_callback[1] = callback;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if(hfdcan == NULL) return;

    if(hfdcan->Instance == FDCAN1 && can_rx_callback[0] != NULL) {
        can_rx_callback[0](hfdcan, RxFifo0ITs);
    }
    else if(hfdcan->Instance == FDCAN2 && can_rx_callback[1] != NULL) {
        can_rx_callback[1](hfdcan, RxFifo0ITs);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
    if(hfdcan == NULL) return;

    if(hfdcan->Instance == FDCAN1 && can_rx_callback[0] != NULL) {
        can_rx_callback[0](hfdcan, RxFifo1ITs);
    }
    else if(hfdcan->Instance == FDCAN2 && can_rx_callback[1] != NULL) {
        can_rx_callback[1](hfdcan, RxFifo1ITs);
    }
}
