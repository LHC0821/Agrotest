#include "servo.h"
#include "can.h"
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

static ServoFeedback g_latest_servo_data;
static ServoFeedback g_servo_feedback_cache[SERVO_ID_MAX - SERVO_ID_MIN + 1U];
static uint8_t g_servo_feedback_valid[SERVO_ID_MAX - SERVO_ID_MIN + 1U];

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
static ServoStatus rs06_start_fdcan(FDCAN_HandleTypeDef* hfdcan);
static uint8_t rs06_motor_id_is_valid(uint8_t motor_id);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void servo_set_instance(const struct ServoInterface* instance) {
    if(instance != NULL) {
        servo_instance = instance;
    }
}

static void servo_can_rx_handler(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef* header, uint8_t* data);

ServoStatus rs06_init(FDCAN_HandleTypeDef* hfdcan) {
    if(hfdcan == NULL) return servo.PARAM_INVALID;

    servo_can = hfdcan;

    if(servo_can->Instance == FDCAN1) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
    else if(servo_can->Instance == FDCAN2) {
        HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);
    }

    memset(&g_latest_servo_data, 0, sizeof(g_latest_servo_data));
    memset(g_servo_feedback_cache, 0, sizeof(g_servo_feedback_cache));
    memset(g_servo_feedback_valid, 0, sizeof(g_servo_feedback_valid));

    can_rx_callback_register(servo_can, servo_can_rx_handler);

    if(rs06_start_fdcan(servo_can) != SERVO_STATUS_OK) {
        return SERVO_STATUS_ERROR;
    }

    return servo.OK;
}

/**
 * @brief 舵机专用的 CAN 接收处理逻辑
 */
static void servo_can_rx_handler(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef* header, uint8_t* data) {
    ServoFeedback feedback;

    if(hfdcan == NULL || header == NULL || data == NULL || servo_can == NULL) {
        return;
    }

    if(hfdcan->Instance != servo_can->Instance) {
        return;
    }

    if(rs06_parse_feedback(header->Identifier, data, &feedback) != SERVO_STATUS_OK) {
        return;
    }

    if(!rs06_motor_id_is_valid(feedback.motor_id)) {
        return;
    }

    __disable_irq();
    g_latest_servo_data = feedback;
    g_servo_feedback_cache[feedback.motor_id - SERVO_ID_MIN] = feedback;
    g_servo_feedback_valid[feedback.motor_id - SERVO_ID_MIN] = 1U;
    __enable_irq();
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
 * @brief 解析主动上报帧 (已修复 8位与16位比较 Bug)
 */
ServoStatus rs06_parse_feedback(uint32_t id, uint8_t data[8], ServoFeedback* res) {
    if(res == NULL || data == NULL) return SERVO_STATUS_PARAM_INVALID;

    // 修改部分 1：指令类型判断 (CMD位在 28-24 位）
    uint8_t cmd_type = (uint8_t)((id >> 24) & 0x1F);
    if(cmd_type != (uint8_t)(SERVO_TYPE_FEEDBACK >> 8)) return SERVO_STATUS_ERROR;

    // 提取电机 ID (bits 8-15)
    res->motor_id = (uint8_t)((id >> 8) & 0xFF);

    // 数据解析逻辑保持不变，确保物理量转换正确
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

ServoStatus rs06_get_feedback(ServoFeedback* out_data) {
    if(out_data == NULL) return SERVO_STATUS_PARAM_INVALID;

    // 为了防止读取时被中断修改导致数据错位，可以使用简单的关中断保护
    __disable_irq();
    *out_data = g_latest_servo_data;
    __enable_irq();

    return SERVO_STATUS_OK;
}

float rs06_get_pos(uint8_t motor_id) {
    float position = 0.0f;

    if(!rs06_motor_id_is_valid(motor_id)) return 0.0f;

    __disable_irq();
    if(g_servo_feedback_valid[motor_id - SERVO_ID_MIN] != 0U) {
        position = g_servo_feedback_cache[motor_id - SERVO_ID_MIN].position;
    }
    __enable_irq();

    return position;
}

float rs06_get_spd(uint8_t motor_id) {
    float velocity = 0.0f;

    if(!rs06_motor_id_is_valid(motor_id)) return 0.0f;

    __disable_irq();
    if(g_servo_feedback_valid[motor_id - SERVO_ID_MIN] != 0U) {
        velocity = g_servo_feedback_cache[motor_id - SERVO_ID_MIN].velocity;
    }
    __enable_irq();

    return velocity;
}

float rs06_get_tor(uint8_t motor_id) {
    float torque = 0.0f;

    if(!rs06_motor_id_is_valid(motor_id)) return 0.0f;

    __disable_irq();
    if(g_servo_feedback_valid[motor_id - SERVO_ID_MIN] != 0U) {
        torque = g_servo_feedback_cache[motor_id - SERVO_ID_MIN].torque;
    }
    __enable_irq();

    return torque;
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

static uint8_t rs06_motor_id_is_valid(uint8_t motor_id) {
    return (uint8_t)((motor_id >= SERVO_ID_MIN) && (motor_id <= SERVO_ID_MAX));
}

static ServoStatus rs06_start_fdcan(FDCAN_HandleTypeDef* hfdcan) {
    if(hfdcan == NULL) {
        return SERVO_STATUS_PARAM_INVALID;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK) {
        return SERVO_STATUS_ERROR;
    }

    if(HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return SERVO_STATUS_ERROR;
    }

    if(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return SERVO_STATUS_ERROR;
    }

    return SERVO_STATUS_OK;
}

static ServoStatus rs06_send(uint32_t id, uint8_t data[8]) {
    if(data == NULL) {
        return SERVO_STATUS_PARAM_INVALID;
    }
    if(servo_can == NULL) {
        return SERVO_STATUS_ERROR;
    }

    return (can_send(servo_can, id, data, 8) == HAL_OK) ? SERVO_STATUS_OK : SERVO_STATUS_ERROR;
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

