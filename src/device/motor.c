#include "motor.h"
#include "can.h"
#include "fdcan.h"
#include "main.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define MOTOR_CMD_SPEED_1_TO_4 0x032U
#define MOTOR_CMD_SPEED_5_TO_8 0x033U

#define MOTOR_CMD_CALIBRATION 0x104U
#define MOTOR_CMD_SET_MODE 0x105U
#define MOTOR_CMD_SET_FEEDBACK 0x106U
#define MOTOR_CMD_REQUEST_REPORT 0x107U
#define MOTOR_CMD_SET_ID 0x108U

#define MOTOR_SPEED_SCALE 100
#define MOTOR_PROTOCOL_SPEED_MAX 32767
#define MOTOR_PROTOCOL_SPEED_MIN -32767

#define PI 3.1415926535f

#define MOTOR_REPORT_COUNT ((uint8_t)(MOTOR_ID_MAX - MOTOR_ID_MIN + 1U))

#define SX(name, value) .name = MOTOR_STATUS_##name,
const struct MotorInterface dm_motor_instance = {
    {
        MOTOR_STATUS_TABLE
    },
    .status_str = dm_motor_status_str,
    .init = dm_motor_init,
    .set_feedback = dm_motor_set_feedback,
    .calibration = dm_motor_calibration,
    .set_id = dm_motor_set_id,
    .set_mode_raw = dm_motor_set_mode_raw,
    .set_speed = dm_motor_set_speed,
    .request_report = dm_motor_request_report,
    .update = dm_motor_update,
    .get_pos = dm_motor_get_pos,
    .get_spd = dm_motor_get_spd,
    .get_tor = dm_motor_get_tor,
    .latest_report = dm_motor_latest_report,
    .set_speed_rps = dm_motor_set_speed_rps,
    .set_speed_rads = dm_motor_set_speed_rads, // 新增弧度驱动接口
    .get_pos_rad = dm_motor_get_pos_rad,
    .get_spd_rads = dm_motor_get_spd_rads
};
#undef SX

const struct MotorInterface* motor_instance = &dm_motor_instance;

static uint8_t g_motor_tx_0x32[8] = { 0 };
static uint8_t g_motor_tx_0x33[8] = { 0 };

static MotorReport g_motor_report_cache[MOTOR_REPORT_COUNT];
static uint8_t g_motor_report_valid[MOTOR_REPORT_COUNT] = { 0 };

static MotorFeedback g_motor_last_feedback;
static uint8_t g_motor_has_new_feedback = 0U;
static uint8_t g_motor_last_query_id = MOTOR_ID_MIN;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint8_t motor_id_is_valid(uint8_t id);
static int16_t motor_clamp_rpm(int16_t rpm);
static int16_t motor_rpm_to_protocol_speed(int16_t rpm);
static MotorStatus motor_start_fdcan(FDCAN_HandleTypeDef* hfdcan);
static void motor_send_speed_scaled(uint8_t id, int16_t speed_scaled);
static void motor_cache_feedback(uint8_t id, const uint8_t rx_data[8], uint16_t rx_len);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void motor_set_instance(const struct MotorInterface* instance) {
    if(instance != NULL) {
        motor_instance = instance;
    }
}

#define SX(name, value) case MOTOR_STATUS_##name: return #name;
const char* dm_motor_status_str(MotorStatus status) {
    switch(status) {
        MOTOR_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef SX

MotorStatus dm_motor_init(void) {
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

    if(motor_start_fdcan(&hfdcan1) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(motor_start_fdcan(&hfdcan2) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    memset(g_motor_tx_0x32, 0, sizeof(g_motor_tx_0x32));
    memset(g_motor_tx_0x33, 0, sizeof(g_motor_tx_0x33));
    memset(g_motor_report_cache, 0, sizeof(g_motor_report_cache));
    memset(g_motor_report_valid, 0, sizeof(g_motor_report_valid));
    memset(&g_motor_last_feedback, 0, sizeof(g_motor_last_feedback));

    g_motor_has_new_feedback = 0U;
    g_motor_last_query_id = MOTOR_ID_MIN;

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_set_feedback(uint8_t feedback_cmd, uint8_t id) {
    uint8_t tx_data[8] = { 0 };

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    tx_data[id - 1U] = feedback_cmd;
    can_send(&hfdcan1, MOTOR_CMD_SET_FEEDBACK, tx_data, 8);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_calibration(void) {
    uint8_t tx_data[8] = { 0 };
    can_send(&hfdcan1, MOTOR_CMD_CALIBRATION, tx_data, 8);
    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_set_id(uint8_t id) {
    uint8_t tx_data[8] = { 0 };

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    tx_data[0] = id;
    can_send(&hfdcan1, MOTOR_CMD_SET_ID, tx_data, 8);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_set_mode_raw(uint8_t mode) {
    uint8_t tx_data[8] = { 0 };
    tx_data[0] = mode;

    can_send(&hfdcan1, MOTOR_CMD_SET_MODE, tx_data, 8);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_set_speed(uint8_t id, int16_t rpm) {
    int16_t rpm_clamped;
    int16_t speed_scaled;

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    rpm_clamped = motor_clamp_rpm(rpm);
    speed_scaled = motor_rpm_to_protocol_speed(rpm_clamped);

    motor_send_speed_scaled(id, speed_scaled);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_request_report(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3) {
    uint8_t tx_data[8] = { 0 };

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    tx_data[0] = id;
    tx_data[1] = check1;
    tx_data[2] = check2;
    tx_data[3] = check3;
    tx_data[4] = 0xAA;

    g_motor_last_query_id = id;
    can_send(&hfdcan1, MOTOR_CMD_REQUEST_REPORT, tx_data, 8);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_update(MotorFeedback* feedback) {
    if(feedback == NULL) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    if(g_motor_has_new_feedback == 0U) {
        return MOTOR_STATUS_NO_DATA;
    }

    *feedback = g_motor_last_feedback;

    g_motor_has_new_feedback = 0U;

    return MOTOR_STATUS_OK;
}

float dm_motor_get_pos(uint8_t id) {
    return (float)g_motor_last_feedback.pos;
}

float dm_motor_get_spd(uint8_t id) {
    return (float)g_motor_last_feedback.spd;
}

float dm_motor_get_tor(uint8_t id) {
    return (float)g_motor_last_feedback.torque;
}

float dm_motor_get_pos_rad(uint8_t id) {
    return g_motor_last_feedback.pos * (PI / 180.0f);
}

float dm_motor_get_spd_rads(uint8_t id) {
    return g_motor_last_feedback.spd * (PI / 30.0f);
}

MotorStatus dm_motor_latest_report(uint8_t id, MotorReport* report) {
    uint8_t index;

    if(!motor_id_is_valid(id) || report == NULL) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    index = (uint8_t)(id - MOTOR_ID_MIN);
    if(g_motor_report_valid[index] == 0U) {
        return MOTOR_STATUS_NO_DATA;
    }

    *report = g_motor_report_cache[index];

    return MOTOR_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint8_t motor_id_is_valid(uint8_t id) {
    return (uint8_t)((id >= MOTOR_ID_MIN) && (id <= MOTOR_ID_MAX));
}

static int16_t motor_clamp_rpm(int16_t rpm) {
    if(rpm > MOTOR_RPM_MAX) {
        return MOTOR_RPM_MAX;
    }
    if(rpm < MOTOR_RPM_MIN) {
        return MOTOR_RPM_MIN;
    }
    return rpm;
}

static int16_t motor_rpm_to_protocol_speed(int16_t rpm) {
    int32_t scaled = (int32_t)rpm * MOTOR_SPEED_SCALE;

    if(scaled > MOTOR_PROTOCOL_SPEED_MAX) {
        scaled = MOTOR_PROTOCOL_SPEED_MAX;
    }
    if(scaled < MOTOR_PROTOCOL_SPEED_MIN) {
        scaled = MOTOR_PROTOCOL_SPEED_MIN;
    }

    return (int16_t)scaled;
}

/**
 * @brief 设置电机目标角速度 (输入单位: rad/s 弧度每秒)
 * @param id 电机ID (1-8)
 * @param rads 目标角速度 (Radians Per Second)
 * @return MotorStatus 状态码
 */
MotorStatus dm_motor_set_speed_rads(uint8_t id, float rads) {
    float rpm_float;
    int16_t rpm_target;
    int16_t rpm_clamped;
    int16_t speed_scaled;

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    // 1. 单位转换: rad/s -> RPM 
    // n = omega * 60 / (2 * PI) = omega * 30 / PI
    rpm_float = rads * (30.0f / PI);

    // 2. 四舍五入转为整数
    if (rpm_float >= 0.0f) {
        rpm_target = (int16_t)(rpm_float + 0.5f);
    } else {
        rpm_target = (int16_t)(rpm_float - 0.5f);
    }

    // 3. 限幅处理 (最大 MOTOR_RPM_MAX)
    rpm_clamped = motor_clamp_rpm(rpm_target);

    // 4. 协议缩放 (RPM * 100)
    speed_scaled = motor_rpm_to_protocol_speed(rpm_clamped);

    // 5. 执行发送
    motor_send_speed_scaled(id, speed_scaled);

    return MOTOR_STATUS_OK;
}

MotorStatus dm_motor_set_speed_rps(uint8_t id, float rps) {
    int16_t rpm_target;
    int16_t rpm_clamped;
    int16_t speed_scaled;

    if(!motor_id_is_valid(id)) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    if (rps >= 0.0f) {
        rpm_target = (int16_t)(rps * 60.0f + 0.5f);
    } else {
        rpm_target = (int16_t)(rps * 60.0f - 0.5f);
    }

    rpm_clamped = motor_clamp_rpm(rpm_target);
    speed_scaled = motor_rpm_to_protocol_speed(rpm_clamped);

    motor_send_speed_scaled(id, speed_scaled);

    return MOTOR_STATUS_OK;
}

static MotorStatus motor_start_fdcan(FDCAN_HandleTypeDef* hfdcan) {
    if(hfdcan == NULL) {
        return MOTOR_STATUS_PARAM_INVALID;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK) {
        return MOTOR_STATUS_ERROR;
    }

    if(HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return MOTOR_STATUS_ERROR;
    }

    if(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return MOTOR_STATUS_ERROR;
    }

    return MOTOR_STATUS_OK;
}

static void motor_send_speed_scaled(uint8_t id, int16_t speed_scaled) {
    uint8_t idx_high;
    uint8_t idx_low;

    if(id <= 4U) {
        idx_high = (uint8_t)((id - 1U) * 2U);
        idx_low = (uint8_t)(idx_high + 1U);

        g_motor_tx_0x32[idx_high] = (uint8_t)((uint16_t)speed_scaled >> 8);
        g_motor_tx_0x32[idx_low] = (uint8_t)((uint16_t)speed_scaled & 0x00FFU);

        can_send(&hfdcan1, MOTOR_CMD_SPEED_1_TO_4, g_motor_tx_0x32, 8);
        return;
    }

    idx_high = (uint8_t)((id - 5U) * 2U);
    idx_low = (uint8_t)(idx_high + 1U);

    g_motor_tx_0x33[idx_high] = (uint8_t)((uint16_t)speed_scaled >> 8);
    g_motor_tx_0x33[idx_low] = (uint8_t)((uint16_t)speed_scaled & 0x00FFU);

    can_send(&hfdcan1, MOTOR_CMD_SPEED_5_TO_8, g_motor_tx_0x33, 8);
}

static void motor_cache_feedback(uint8_t id, const uint8_t rx_data[8], uint16_t rx_len) {
    uint8_t index = (uint8_t)(id - MOTOR_ID_MIN);
    MotorReport* report = &g_motor_report_cache[index];

    report->id = id;
    report->fb_speed = (int16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);

    if(rx_len >= 8U) {
        report->e_curru = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
        report->position = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
        report->err_code = rx_data[6];
        report->fb_mode = rx_data[7];
    }
    else {
        report->e_curru = 0;
        report->position = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
        report->err_code = rx_data[5];
        report->fb_mode = 0;
    }

    g_motor_report_valid[index] = 1U;

    g_motor_last_feedback.id = id;
    g_motor_last_feedback.spd = (float)report->fb_speed / 100.0f;
    g_motor_last_feedback.torque = (float)report->e_curru / 100.0f;
    g_motor_last_feedback.pos = (float)report->position / 100.0f;
    g_motor_last_feedback.err_code = report->err_code;

    g_motor_has_new_feedback = 1U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = { 0 };
    uint16_t rx_len;

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if(hfdcan == NULL || hfdcan->Instance != FDCAN1) {
        return;
    }

    if(!motor_id_is_valid(g_motor_last_query_id)) {
        return;
    }

    rx_len = get_fdcan_data_size(rx_header.DataLength);
    if(rx_len < 6U) {
        return;
    }

    motor_cache_feedback(g_motor_last_query_id, rx_data, rx_len);
}