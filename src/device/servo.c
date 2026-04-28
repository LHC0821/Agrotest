#include "servo.h"
#include "can.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define SX(name, value) .name = SERVO_STATUS_##name,
#define MX(name, value) .name = SERVO_MODE_##name,
#define TX(name, value) .name = SERVO_TYPE_##name,
const struct ServoInterface rs06_servo_instance = {
    {
        SERVO_STATUS_TABLE
    },
    {
        SERVO_MODE_TABLE
    },
    {
        SERVO_TYPE_TABLE
    },
    .status_str = rs06_status_str,
    .mode_str = rs06_mode_str,
    .enable = rs06_enable,
    .stop = rs06_stop,
    .change_id = rs06_change_id,
    .set_mode = rs06_set_mode,
    .set_position = rs06_set_position,
    .set_mit = rs06_set_mit,
    .reset = rs06_reset,
    .config_reporting = rs06_config_reporting,
    .parse_feedback = rs06_parse_feedback,
    .get_pos = rs06_get_pos,
    .get_spd = rs06_get_spd,
    .get_tor = rs06_get_tor,
};
#undef SX
#undef MX
#undef TX

const struct ServoInterface* servo_instance = &rs06_servo_instance;

static FDCAN_HandleTypeDef* servo_can;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint16_t rs06_float_to_u16(float val, float min, float max, uint8_t bits);
static float rs06_u16_to_float(uint16_t val, float min, float max, uint8_t bits); // 新增：缺失函数的声明
static ServoStatus rs06_send(uint32_t id, uint8_t data[8]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void servo_set_instance(const struct ServoInterface* instance) {
    if(instance != NULL) {
        servo_instance = instance;
    }
}

ServoStatus rs06_init(FDCAN_HandleTypeDef* hfdcan) {
    if(hfdcan == NULL) return servo.PARAM_INVALID;

    servo_can = hfdcan;
    return servo.OK;
}

#define SX(name, value) case SERVO_STATUS_##name: return #name;
const char* rs06_status_str(ServoStatus status) {
    switch(status) {
        SERVO_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef SX

#define MX(name, value) case SERVO_MODE_##name: return #name;
const char* rs06_mode_str(ServoMode mode) {
    switch(mode) {
        SERVO_MODE_TABLE
        default: return "UNKNOWN";
    }
}
#undef MX

ServoStatus rs06_enable(uint8_t motor_id) {
    uint32_t id = EXT_ID(SERVO_TYPE_ENABLE, HOST_ID, motor_id);
    uint8_t data[8] = { 0 };
    return rs06_send(id, data);
}

ServoStatus rs06_stop(uint8_t motor_id) {
    uint32_t id = EXT_ID(SERVO_TYPE_STOP, HOST_ID, motor_id);
    uint8_t data[8] = { 0 };
    return rs06_send(id, data);
}

ServoStatus rs06_change_id(uint8_t old_id, uint8_t new_id) {
    uint32_t id = EXT_ID(SERVO_TYPE_SET_ID, new_id, old_id);
    uint8_t data[8] = { 0 };
    return rs06_send(id, data);
}

ServoStatus rs06_set_mode(uint8_t motor_id, ServoMode mode) {
    uint32_t id = EXT_ID(SERVO_TYPE_WR_PARAM, HOST_ID, motor_id);
    uint8_t data[8] = { 0 };

    data[0] = PARAM_RUN_MODE & 0xFF;
    data[1] = (PARAM_RUN_MODE >> 8) & 0xFF;
    data[4] = (uint8_t)mode;

    return rs06_send(id, data);
}

ServoStatus rs06_set_position(uint8_t motor_id, float angle_rad) {
    uint32_t id = EXT_ID(SERVO_TYPE_WR_PARAM, HOST_ID, motor_id);
    uint8_t data[8] = { 0 };

    data[0] = PARAM_LOC_REF & 0xFF;
    data[1] = (PARAM_LOC_REF >> 8) & 0xFF;
    memcpy(&data[4], &angle_rad, sizeof(float));

    return rs06_send(id, data);
}

ServoStatus rs06_set_mit(uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff) {
    uint32_t id = EXT_ID(SERVO_TYPE_RUN, HOST_ID, motor_id);

    uint16_t p_int = rs06_float_to_u16(angle_rad, P_MIN, P_MAX, 16);
    uint16_t v_int = rs06_float_to_u16(speed_rad_s, V_MIN, V_MAX, 12);
    uint16_t kp_int = rs06_float_to_u16(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = rs06_float_to_u16(kd, KD_MIN, KD_MAX, 12);
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

    return rs06_send(id, data);
}

void rs06_reset(void) {
    // 回归零位
}

/**
 * @brief 设置主动上报周期 (新增)
 */
ServoStatus rs06_config_reporting(uint8_t motor_id, uint16_t interval_ms) {
    uint32_t id = EXT_ID(SERVO_TYPE_WR_PARAM, HOST_ID, motor_id);
    uint8_t data[8] = { 0 };
    data[0] = PARAM_FEEDBACK_MS & 0xFF;
    data[1] = (PARAM_FEEDBACK_MS >> 8) & 0xFF;
    uint32_t val = (uint32_t)interval_ms;
    memcpy(&data[4], &val, sizeof(uint32_t));
    return rs06_send(id, data);
}

/**
 * @brief 解析主动上报帧 (已针对协议修正)
 * 修改点：
 * 1. res->motor_id 的提取改为从 bits 8-15 提取（发送源 ID）。
 * 2. msg_type 的判断改为提取 bits 24-28（指令模式位）。
 */
ServoStatus rs06_parse_feedback(uint32_t id, uint8_t data[8], ServoFeedback* res) {
    if(res == NULL || data == NULL) return SERVO_STATUS_PARAM_INVALID;

    // 修改部分 1：指令类型判断 (CMD位在 28-24 位）
    uint8_t cmd_type = (uint8_t)((id >> 24) & 0x1F);
    if(cmd_type != SERVO_TYPE_FEEDBACK) return SERVO_STATUS_ERROR;

    // 修改部分 2：电机 ID 提取
    // 在电机主动回传时，电机 ID 位于 bits 8-15 (Source ID 区域)
    res->motor_id = (uint8_t)((id >> 8) & 0xFF);

    // 以下保持您的原始数据解析逻辑不变
    uint16_t p_raw = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t v_raw = (uint16_t)((data[2] << 8) | data[3]);
    uint16_t t_raw = (uint16_t)((data[4] << 8) | data[5]);

    res->position = rs06_u16_to_float(p_raw, P_MIN, P_MAX, 16);
    res->velocity = rs06_u16_to_float(v_raw, V_MIN, V_MAX, 16);
    res->torque = rs06_u16_to_float(t_raw, T_MIN, T_MAX, 16);
    res->temperature = data[6];
    res->error_code = data[7];

    return SERVO_STATUS_OK;
}

extern ServoFeedback g_latest_servo_data;

ServoStatus rs06_get_feedback(ServoFeedback* out_data) {
    if(out_data == NULL) return SERVO_STATUS_PARAM_INVALID;

    // 为了防止读取时被中断修改导致数据错位，可以使用简单的关中断保护
    __disable_irq();
    *out_data = g_latest_servo_data;
    __enable_irq();

    return SERVO_STATUS_OK;
}

float rs06_get_pos(uint8_t motor_id) {
    return g_latest_servo_data.position;
}

float rs06_get_spd(uint8_t motor_id) {
    return g_latest_servo_data.velocity;
}

float rs06_get_tor(uint8_t motor_id) {
    return g_latest_servo_data.torque;
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

static ServoStatus rs06_send(uint32_t id, uint8_t data[8]) {
    if(data == NULL) {
        return SERVO_STATUS_PARAM_INVALID;
    }

    can_send(servo_can, id, data, 8);
    return SERVO_STATUS_OK;
}


/**
 * @brief 将无符号整数还原为浮点数 (新增实现)
 */
static float rs06_u16_to_float(uint16_t val, float min, float max, uint8_t bits) {
    if(bits == 0 || max <= min) {
        return 0.0f;
    }

    uint32_t max_bits_val = (1UL << bits) - 1UL;
    float span = max - min;
    float normalized = (float)val / (float)max_bits_val;

    return min + (normalized * span);
}