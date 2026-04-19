#include "motor.h"
#include "can.h"

#include "delay.h"
#include "fdcan.h"
#include "tim.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define DSX(name, value) .name = DM_MOTOR_##name,
#define DMX(name, value) .name = DM_MOTOR_MODE_##name,
const struct MotorInterface motor_instance = {
    {
        DM_MOTOR_STATUS_TABLE
    },
    {
        DM_MOTOR_MODE_TABLE
    },
    .status_str = dm_status_str,
    .mode_str = dm_mode_str,
    .enable = dm_enable,
    .disable = dm_disable,
    .set_mit = dm_set_mit,
    .set_pos_spd = dm_set_pos_spd,
    .set_spd = dm_set_spd,
    .set_pos_spd_cur = dm_set_pos_spd_cur,
    .get_feedback = dm_get_feedback,
    .get_err_code = dm_get_err_code,
    .get_pos = dm_get_pos,
    .get_spd = dm_get_spd,
    .get_torque = dm_get_torque,
    .request_feedback = dm_request_feedback,
    .update = dm_update,
    .init = dm_init,
    .set_feedback = dm_set_feedback,
    .calibration = dm_calibration,
    .check = dm_check,
    .obtain_report = dm_obtain_report,
    .set_id = dm_set_id,
    .set_mode_raw = dm_set_mode_raw,
    .speed_control = dm_speed_control,
    .control_all = dm_control_all,
    .speed_control_smooth = dm_speed_control_smooth,
    .stop_immediately = dm_stop_immediately,
    .monitor_read = dm_monitor_read
};
#undef DSX
#undef DMX

MotorReport dm_reporter_cache[4];
uint8_t dm_query_id = 1;

static uint8_t g_dm_last_feedback[DM_MOTOR_CMD_LEN];

static uint8_t g_motor_tx_0x32[8] = { 0 };
static uint8_t g_motor_tx_0x33[8] = { 0 };

typedef struct {
    int16_t target_rpm;
    float current_rpm;
    float accel_step;
} MotorSmoothCtrl;

static MotorSmoothCtrl g_motor_states[4];

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static MotorStatus dm_can_send(uint16_t id, uint8_t data[8]);
static MotorStatus dm_can_rcvd(uint8_t buffer[8]);
static void dm_write_register(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
static void dm_switch_mode(uint16_t id, MotorMode mode);
static uint16_t dm_f32_to_u16(float val, float min, float max, uint8_t bits);
static float dm_u16_to_f32(uint16_t val, float min, float max, uint8_t bits);

static int16_t motor_clamp_rpm(int16_t rpm);
static void motor_drive_direct(int16_t input_rpm, uint8_t id);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define DSX(name, value) case DM_MOTOR_##name: return #name;
const char* dm_status_str(MotorStatus status) {
    switch(status) {
        DM_MOTOR_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef DSX

#define DMX(name, value) case DM_MOTOR_MODE_##name: return #name;
const char* dm_mode_str(MotorMode mode) {
    switch(mode) {
        DM_MOTOR_MODE_TABLE
        default: return "UNKNOWN";
    }
}
#undef DMX

MotorStatus dm_enable(uint16_t id) {
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    return dm_can_send(id, data);
}

MotorStatus dm_disable(uint16_t id) {
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    return dm_can_send(id, data);
}

MotorStatus dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque) {
    dm_switch_mode(id, DM_MOTOR_MODE_MIT);

    uint16_t pos_bits = dm_f32_to_u16(pos, -DM_MOTOR_POS_LIMIT, DM_MOTOR_POS_LIMIT, 16);
    uint16_t spd_bits = dm_f32_to_u16(spd, -DM_MOTOR_SPD_LIMIT, DM_MOTOR_SPD_LIMIT, 12);
    uint16_t kp_bits = dm_f32_to_u16(kp, 0.0f, DM_MOTOR_KP_LIMIT, 12);
    uint16_t kd_bits = dm_f32_to_u16(kd, 0.0f, DM_MOTOR_KD_LIMIT, 12);
    uint16_t torque_bits = dm_f32_to_u16(torque, -DM_MOTOR_TORQUE_LIMIT, DM_MOTOR_TORQUE_LIMIT, 12);

    uint8_t data[8];
    data[0] = (uint8_t)(pos_bits >> 8);
    data[1] = (uint8_t)(pos_bits & 0xFF);
    data[2] = (uint8_t)(spd_bits >> 4);
    data[3] = (uint8_t)(((spd_bits & 0x0F) << 4) | (kp_bits >> 8));
    data[4] = (uint8_t)(kp_bits & 0xFF);
    data[5] = (uint8_t)(kd_bits >> 4);
    data[6] = (uint8_t)(((kd_bits & 0x0F) << 4) | (torque_bits >> 8));
    data[7] = (uint8_t)(torque_bits & 0xFF);

    return dm_can_send(id, data);
}

MotorStatus dm_set_pos_spd(uint16_t id, float pos, float spd) {
    dm_switch_mode(id, DM_MOTOR_MODE_POS_SPD);

    uint16_t target_id = (uint16_t)(id + 0x100);
    uint8_t data[8];
    memcpy(&data[0], &pos, sizeof(float));
    memcpy(&data[4], &spd, sizeof(float));

    return dm_can_send(target_id, data);
}

MotorStatus dm_set_spd(uint16_t id, float spd) {
    dm_switch_mode(id, DM_MOTOR_MODE_SPD);

    uint16_t target_id = (uint16_t)(id + 0x200);
    uint8_t data[8] = { 0 };
    memcpy(&data[0], &spd, sizeof(float));

    return dm_can_send(target_id, data);
}

MotorStatus dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur) {
    dm_switch_mode(id, DM_MOTOR_MODE_POS_SPD_CUR);

    uint16_t target_id = (uint16_t)(id + 0x300);
    uint16_t spd_scaled = (uint16_t)(spd * 100.0f);
    uint16_t cur_scaled = (uint16_t)(cur * 10000.0f);
    uint8_t data[8];

    memcpy(&data[0], &pos, sizeof(float));
    data[4] = (uint8_t)(spd_scaled & 0xFF);
    data[5] = (uint8_t)(spd_scaled >> 8);
    data[6] = (uint8_t)(cur_scaled & 0xFF);
    data[7] = (uint8_t)(cur_scaled >> 8);

    return dm_can_send(target_id, data);
}

MotorStatus dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms) {
    if(feedback == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t req_data[8] = { can_id_l, can_id_h, 0xCC, 0x00, 0, 0, 0, 0 };

    if(dm_can_send(0x7FF, req_data) != DM_MOTOR_OK) {
        return DM_MOTOR_ERROR;
    }

    ms_t start = HAL_GetTick();
    while((HAL_GetTick() - start) < timeout_ms) {
        if(dm_can_rcvd(feedback) == DM_MOTOR_OK) {
            uint8_t expected_id = (uint8_t)(can_id_l & 0x0F);
            uint8_t received_id = (uint8_t)(feedback[0] & 0x0F);
            return (expected_id == received_id) ? DM_MOTOR_OK : DM_MOTOR_ID_MISMATCH;
        }
    }

    return DM_MOTOR_TIMEOUT;
}

MotorStatus dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms) {
    if(err_code == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    MotorStatus result = dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_OK) {
        return result;
    }

    *err_code = (uint8_t)(feedback[0] >> 4);
    return DM_MOTOR_OK;
}

MotorStatus dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms) {
    if(pos == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    MotorStatus result = dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_OK) {
        return result;
    }

    uint16_t pos_bits = (uint16_t)(((uint16_t)feedback[1] << 8) | feedback[2]);
    *pos = dm_u16_to_f32(pos_bits, -DM_MOTOR_POS_LIMIT, DM_MOTOR_POS_LIMIT, 16);
    return DM_MOTOR_OK;
}

MotorStatus dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms) {
    if(spd == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    MotorStatus result = dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_OK) {
        return result;
    }

    uint16_t spd_bits = (uint16_t)(((uint16_t)feedback[3] << 4) | ((uint16_t)(feedback[4] & 0xF0) >> 4));
    *spd = dm_u16_to_f32(spd_bits, -DM_MOTOR_SPD_LIMIT, DM_MOTOR_SPD_LIMIT, 12);
    return DM_MOTOR_OK;
}

MotorStatus dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms) {
    if(torque == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    MotorStatus result = dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_OK) {
        return result;
    }

    uint16_t torque_bits = (uint16_t)((((uint16_t)feedback[4] & 0x0F) << 8) | feedback[5]);
    *torque = dm_u16_to_f32(torque_bits, -DM_MOTOR_TORQUE_LIMIT, DM_MOTOR_TORQUE_LIMIT, 12);
    return DM_MOTOR_OK;
}

MotorStatus dm_request_feedback(uint16_t id) {
    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t req_data[8] = { can_id_l, can_id_h, 0xCC, 0x00, 0, 0, 0, 0 };

    return dm_can_send(0x7FF, req_data);
}

MotorStatus dm_update(MotorFeedback* feedback) {
    if(feedback == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t raw_feedback[8];
    if(dm_can_rcvd(raw_feedback) != DM_MOTOR_OK) {
        return DM_MOTOR_TIMEOUT;
    }

    uint8_t id = (uint8_t)(raw_feedback[0] & 0x0F);
    uint8_t err_code = (uint8_t)(raw_feedback[0] >> 4);
    uint16_t pos_bits = (uint16_t)(((uint16_t)raw_feedback[1] << 8) | raw_feedback[2]);
    float pos = dm_u16_to_f32(pos_bits, -DM_MOTOR_POS_LIMIT, DM_MOTOR_POS_LIMIT, 16);
    uint16_t spd_bits = (uint16_t)(((uint16_t)raw_feedback[3] << 4) | ((uint16_t)(raw_feedback[4] & 0xF0) >> 4));
    float spd = dm_u16_to_f32(spd_bits, -DM_MOTOR_SPD_LIMIT, DM_MOTOR_SPD_LIMIT, 12);
    uint16_t torque_bits = (uint16_t)((((uint16_t)raw_feedback[4] & 0x0F) << 8) | raw_feedback[5]);
    float torque = dm_u16_to_f32(torque_bits, -DM_MOTOR_TORQUE_LIMIT, DM_MOTOR_TORQUE_LIMIT, 12);

    feedback->id = id;
    feedback->err_code = err_code;
    feedback->pos = pos;
    feedback->spd = spd;
    feedback->torque = torque;

    memcpy(g_dm_last_feedback, raw_feedback, DM_MOTOR_CMD_LEN);

    return DM_MOTOR_OK;
}

void dm_init(void) {
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

    can_filter_init(&hfdcan1);
    can_filter_init(&hfdcan2);

    for(uint8_t i = 0; i < 4; i++) {
        g_motor_states[i].target_rpm = 0;
        g_motor_states[i].current_rpm = 0.0f;
        g_motor_states[i].accel_step = 0.4f;
        memset(&dm_reporter_cache[i], 0, sizeof(dm_reporter_cache[i]));
    }

    HAL_TIM_Base_Start_IT(&htim6);
}

MotorStatus dm_set_feedback(uint8_t feedback_cmd, uint8_t id) {
    uint8_t data[8] = { 0 };
    if(id > 0 && id <= 8) {
        data[id - 1] = feedback_cmd;
    }
    return dm_can_send(0x106, data);
}

void dm_calibration(void) {
    uint8_t data[8] = { 0 };
    dm_can_send(0x104, data);
}

void dm_check(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3, MotorReport* report) {
    uint8_t tx_buf[8] = { id, check1, check2, check3, 0xAA, 0, 0, 0 };
    dm_can_send(0x107, tx_buf);

    if(report != NULL) {
        dm_obtain_report(report);
    }
}

void dm_obtain_report(MotorReport* report) {
    if(report == NULL) {
        return;
    }

    report->fb_speed = (int16_t)((g_dm_last_feedback[0] << 8) | g_dm_last_feedback[1]);
    report->e_curru = (int16_t)((g_dm_last_feedback[2] << 8) | g_dm_last_feedback[3]);
    report->position = (int16_t)((g_dm_last_feedback[4] << 8) | g_dm_last_feedback[5]);
    report->err_code = g_dm_last_feedback[6];
    report->fb_mode = g_dm_last_feedback[7];
}

MotorStatus dm_set_id(uint8_t id) {
    uint8_t data[8] = { 0 };
    data[0] = id;
    return dm_can_send(0x108, data);
}

MotorStatus dm_set_mode_raw(uint8_t mode) {
    uint8_t data[8] = { 0 };
    data[0] = mode;
    return dm_can_send(0x105, data);
}

void dm_speed_control(int16_t input_rpm, uint8_t id) {
    motor_drive_direct(input_rpm, id);
}

void dm_control_all(int16_t rpms[4]) {
    if(rpms == NULL) {
        return;
    }

    for(uint8_t i = 0; i < 4; i++) {
        motor_drive_direct(rpms[i], (uint8_t)(i + 1));
    }
}

void dm_speed_control_smooth(int16_t input_rpm, uint8_t id) {
    if(id < 1 || id > 4) {
        return;
    }

    g_motor_states[id - 1].target_rpm = motor_clamp_rpm(input_rpm);
}

void dm_stop_immediately(uint8_t id) {
    if(id < 1 || id > 4) {
        return;
    }

    g_motor_states[id - 1].target_rpm = 0;
    g_motor_states[id - 1].current_rpm = 0.0f;
    motor_drive_direct(0, id);
}

void dm_monitor_read(void) {
    MotorFeedback fb;
    if(dm_update(&fb) != DM_MOTOR_OK) {
        return;
    }

    if(fb.id >= 1 && fb.id <= 4) {
        MotorReport* cache = &dm_reporter_cache[fb.id - 1];
        cache->fb_speed = (int16_t)(fb.spd * 100.0f);
        cache->e_curru = (int16_t)(fb.torque * 100.0f);
        cache->position = (int16_t)(fb.pos * 100.0f);
        cache->err_code = fb.err_code;
        cache->fb_mode = 0;
    }

    if(++dm_query_id > 4) {
        dm_query_id = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if(htim->Instance == TIM6) {
        for(uint8_t i = 0; i < 4; i++) {
            float diff = (float)g_motor_states[i].target_rpm - g_motor_states[i].current_rpm;

            if(diff > g_motor_states[i].accel_step) {
                g_motor_states[i].current_rpm += g_motor_states[i].accel_step;
            }
            else if(diff < -g_motor_states[i].accel_step) {
                g_motor_states[i].current_rpm -= g_motor_states[i].accel_step;
            }
            else {
                g_motor_states[i].current_rpm = (float)g_motor_states[i].target_rpm;
            }

            int16_t send_val = (int16_t)(g_motor_states[i].current_rpm * 100.0f);
            g_motor_tx_0x32[i * 2] = (uint8_t)(send_val >> 8);
            g_motor_tx_0x32[i * 2 + 1] = (uint8_t)(send_val & 0xFF);
        }

        can_send(&hfdcan1, 0x032, g_motor_tx_0x32, 8);
    }
    if(htim->Instance == TIM15) {
        dm_monitor_read();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, g_dm_last_feedback) != HAL_OK) {
        return;
    }

    if(dm_query_id >= 1 && dm_query_id <= 4) {
        MotorReport* cache = &dm_reporter_cache[dm_query_id - 1];
        cache->fb_speed = (int16_t)((g_dm_last_feedback[0] << 8) | g_dm_last_feedback[1]);
        cache->position = (int16_t)((g_dm_last_feedback[2] << 8) | g_dm_last_feedback[3]);
        cache->err_code = g_dm_last_feedback[4];
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static MotorStatus dm_can_send(uint16_t id, uint8_t data[8]) {
    if(data == NULL) {
        return DM_MOTOR_ERROR;
    }

    can_send(&hfdcan1, id, data, DM_MOTOR_CMD_LEN);
    delay_ms(1);
    return DM_MOTOR_OK;
}

static MotorStatus dm_can_rcvd(uint8_t buffer[8]) {
    if(buffer == NULL) {
        return DM_MOTOR_ERROR;
    }

    if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
        return DM_MOTOR_TIMEOUT;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, buffer) != HAL_OK) {
        return DM_MOTOR_TIMEOUT;
    }

    uint16_t data_len = get_fdcan_data_size(rx_header.DataLength);
    if(data_len > DM_MOTOR_CMD_LEN) {
        data_len = DM_MOTOR_CMD_LEN;
    }
    if(data_len < DM_MOTOR_CMD_LEN) {
        memset(&buffer[data_len], 0, DM_MOTOR_CMD_LEN - data_len);
    }

    memcpy(g_dm_last_feedback, buffer, DM_MOTOR_CMD_LEN);
    return DM_MOTOR_OK;
}

static void dm_write_register(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t data[8] = { can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3 };

    dm_can_send(0x7FF, data);
}

static void dm_switch_mode(uint16_t id, MotorMode mode) {
    dm_write_register(id, 10, (uint8_t)mode, 0, 0, 0);
    delay_ms(1);
}

static uint16_t dm_f32_to_u16(float val, float min, float max, uint8_t bits) {
    if(bits == 0 || max <= min) {
        return 0;
    }

    if(val < min) {
        val = min;
    }
    if(val > max) {
        val = max;
    }

    float span = max - min;
    float normalized = (val - min) / span;
    uint32_t max_bits_val = (1UL << bits) - 1UL;

    return (uint16_t)(normalized * (float)max_bits_val);
}

static float dm_u16_to_f32(uint16_t val, float min, float max, uint8_t bits) {
    if(bits == 0 || max <= min) {
        return 0.0f;
    }

    float span = max - min;
    uint32_t max_bits_val = (1UL << bits) - 1UL;

    return ((float)val) * span / (float)max_bits_val + min;
}

static int16_t motor_clamp_rpm(int16_t rpm) {
    if(rpm > 210) {
        return 210;
    }
    if(rpm < -210) {
        return -210;
    }
    return rpm;
}

static void motor_drive_direct(int16_t input_rpm, uint8_t id) {
    int16_t rpm = motor_clamp_rpm(input_rpm);
    int32_t target_val = (int32_t)rpm * 100;

    if(target_val > 32767) {
        target_val = 32767;
    }
    if(target_val < -32767) {
        target_val = -32767;
    }

    int16_t scaled_speed = (int16_t)target_val;

    if(id >= 1 && id <= 4) {
        uint8_t idx_high = (uint8_t)((id - 1) * 2);
        uint8_t idx_low = (uint8_t)(idx_high + 1);
        g_motor_tx_0x32[idx_high] = (uint8_t)(scaled_speed >> 8);
        g_motor_tx_0x32[idx_low] = (uint8_t)(scaled_speed & 0xFF);
        can_send(&hfdcan1, 0x032, g_motor_tx_0x32, 8);
    }
    else if(id >= 5 && id <= 8) {
        uint8_t idx_high = (uint8_t)((id - 5) * 2);
        uint8_t idx_low = (uint8_t)(idx_high + 1);
        g_motor_tx_0x33[idx_high] = (uint8_t)(scaled_speed >> 8);
        g_motor_tx_0x33[idx_low] = (uint8_t)(scaled_speed & 0xFF);
        can_send(&hfdcan1, 0x033, g_motor_tx_0x33, 8);
    }
}
