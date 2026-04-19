#include "servo.h"
#include "can.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define RSX(name, value) .name = RS06_SERVO_STATUS_##name,
#define RMX(name, value) .name = RS06_SERVO_MODE_##name,
#define RTX(name, value) .name = RS06_SERVO_TYPE_##name,
const struct Rs06ServoInterface rs06_servo_instance = {
    {
        RS06_SERVO_STATUS_TABLE
    },
    {
        RS06_SERVO_MODE_TABLE
    },
    {
        RS06_SERVO_TYPE_TABLE
    },
    .status_str = rs06_status_str,
    .mode_str = rs06_mode_str,
    .enable = rs06_enable,
    .stop = rs06_stop,
    .change_id = rs06_change_id,
    .set_mode = rs06_set_mode,
    .set_position_target = rs06_set_position_target,
    .set_position = rs06_set_position,
    .turn = rs06_turn
};
#undef RSX
#undef RMX
#undef RTX

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint16_t rs06_float_to_u16(float val, float min, float max, uint8_t bits);
static Rs06ServoStatus rs06_send(FDCAN_HandleTypeDef* hfdcan, uint32_t id, uint8_t data[8]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define RSX(name, value) case RS06_SERVO_STATUS_##name: return #name;
const char* rs06_status_str(Rs06ServoStatus status) {
    switch(status) {
        RS06_SERVO_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef RSX

#define RMX(name, value) case RS06_SERVO_MODE_##name: return #name;
const char* rs06_mode_str(Rs06ServoMode mode) {
    switch(mode) {
        RS06_SERVO_MODE_TABLE
        default: return "UNKNOWN";
    }
}
#undef RMX

Rs06ServoStatus rs06_enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_ENABLE, RS06_HOST_ID, motor_id);
    uint8_t data[8] = { 0 };
    return rs06_send(hfdcan, id, data);
}

Rs06ServoStatus rs06_stop(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_STOP, RS06_HOST_ID, motor_id);
    uint8_t data[8] = { 0 };
    return rs06_send(hfdcan, id, data);
}

Rs06ServoStatus rs06_change_id(FDCAN_HandleTypeDef* hfdcan, uint8_t old_id, uint8_t new_id) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_SET_ID, new_id, old_id);
    uint8_t data[8] = { 0 };
    return rs06_send(hfdcan, id, data);
}

Rs06ServoStatus rs06_set_mode(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, Rs06ServoMode mode) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_WR_PARAM, RS06_HOST_ID, motor_id);
    uint8_t data[8] = { 0 };

    data[0] = RS06_PARAM_RUN_MODE & 0xFF;
    data[1] = (RS06_PARAM_RUN_MODE >> 8) & 0xFF;
    data[4] = (uint8_t)mode;

    return rs06_send(hfdcan, id, data);
}

Rs06ServoStatus rs06_set_position_target(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_WR_PARAM, RS06_HOST_ID, motor_id);
    uint8_t data[8] = { 0 };

    data[0] = RS06_PARAM_LOC_REF & 0xFF;
    data[1] = (RS06_PARAM_LOC_REF >> 8) & 0xFF;
    memcpy(&data[4], &angle_rad, sizeof(float));

    return rs06_send(hfdcan, id, data);
}

Rs06ServoStatus rs06_set_position(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff) {
    uint32_t id = RS06_EXT_ID(RS06_SERVO_TYPE_RUN, RS06_HOST_ID, motor_id);

    uint16_t p_int = rs06_float_to_u16(angle_rad, RS06_P_MIN, RS06_P_MAX, 16);
    uint16_t v_int = rs06_float_to_u16(speed_rad_s, RS06_V_MIN, RS06_V_MAX, 12);
    uint16_t kp_int = rs06_float_to_u16(kp, RS06_KP_MIN, RS06_KP_MAX, 12);
    uint16_t kd_int = rs06_float_to_u16(kd, RS06_KD_MIN, RS06_KD_MAX, 12);
    uint16_t t_int = rs06_float_to_u16(t_ff, -36.0f, 36.0f, 12);

    uint8_t data[8];
    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)(v_int >> 4);
    data[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = (uint8_t)(t_int & 0xFF);

    return rs06_send(hfdcan, id, data);
}

void rs06_turn(void) {
    // 预留测试序列，按需启用。
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint16_t rs06_float_to_u16(float val, float min, float max, uint8_t bits) {
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

static Rs06ServoStatus rs06_send(FDCAN_HandleTypeDef* hfdcan, uint32_t id, uint8_t data[8]) {
    if(hfdcan == NULL || data == NULL) {
        return RS06_SERVO_STATUS_PARAM_INVALID;
    }

    can_send(hfdcan, id, data, 8);
    return RS06_SERVO_STATUS_OK;
}
