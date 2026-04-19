#ifndef _servo_h_
#define _servo_h_

#include "main.h"
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 电机反馈数据结构体 (新增：用于存放解析后的物理量)
 */
typedef struct {
    uint8_t motor_id;      // 电机ID
    float position;        // 当前位置 (rad)
    float velocity;        // 当前速度 (rad/s)
    float torque;          // 当前扭矩 (N.m)
    uint8_t temperature;   // 电机温度 (℃)
    uint8_t error_code;    // 错误代码
} ServoFeedback;

#define servo (*servo_instance)

#define HOST_ID 0xFD
#define EXT_ID(type, host, motor) (((uint32_t)(type) << 16) | ((uint32_t)(host) << 8) | ((uint32_t)(motor)))

#define PARAM_RUN_MODE    0x7005
#define PARAM_LOC_REF     0x7016
#define PARAM_LIMIT_SPD   0x7017
#define PARAM_FEEDBACK_MS 0x7011  // 来源：手册参数索引表，控制主动上报频率

#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -33.0f             // RS06 额定/峰值扭矩转换范围最小值
#define T_MAX 33.0f              // RS06 额定/峰值扭矩转换范围最大值

#define SERVO_STATUS_TABLE \
    SX(OK, 0) \
    SX(ERROR, 1) \
    SX(TIMEOUT, 2) \
    SX(PARAM_INVALID, 3)

#define SERVO_MODE_TABLE \
    MX(MIT, 0) \
    MX(PP, 1) \
    MX(CSP, 5)

#define SERVO_TYPE_TABLE \
    TX(RUN, 0x0100) \
    TX(ENABLE, 0x0300) \
    TX(STOP, 0x0400) \
    TX(SET_ZERO, 0x0600) \
    TX(SET_ID, 0x0701) \
    TX(WR_PARAM, 0x1200) \
    TX(FEEDBACK, 0x1800)      

#define SX(name, value) SERVO_STATUS_##name = value,
typedef enum {
    SERVO_STATUS_TABLE
} ServoStatus;
#undef SX

#define MX(name, value) SERVO_MODE_##name = value,
typedef enum {
    SERVO_MODE_TABLE
} ServoMode;
#undef MX

#define TX(name, value) SERVO_TYPE_##name = value,
typedef enum {
    SERVO_TYPE_TABLE
} ServoType;
#undef TX

#define SX(name, value) const ServoStatus name;
#define MX(name, value) const ServoMode name;
#define TX(name, value) const ServoType name;
extern const struct ServoInterface {
    struct {
        SERVO_STATUS_TABLE
    };
    struct {
        SERVO_MODE_TABLE
    };
    struct {
        SERVO_TYPE_TABLE
    };

    const char* (*status_str)(ServoStatus status);
    const char* (*mode_str)(ServoMode mode);

    ServoStatus(*init)(FDCAN_HandleTypeDef* hfdcan);
    ServoStatus(*enable)(uint8_t motor_id);
    ServoStatus(*stop)(uint8_t motor_id);
    ServoStatus(*change_id)(uint8_t old_id, uint8_t new_id);
    ServoStatus(*set_mode)(uint8_t motor_id, ServoMode mode);
    ServoStatus(*set_position)(uint8_t motor_id, float angle_rad);
    ServoStatus(*set_mit)(uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
    void (*reset)(void);
    
    // ! ========================= 新 增 接口 ========================= ! //
    ServoStatus(*config_reporting)(uint8_t motor_id, uint16_t interval_ms);
    ServoStatus(*parse_feedback)(uint32_t id, uint8_t data[8], ServoFeedback* res);
}*servo_instance;
#undef SX
#undef MX
#undef TX

extern const struct ServoInterface rs06_servo_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void servo_set_instance(const struct ServoInterface* instance);
ServoStatus rs06_init(FDCAN_HandleTypeDef* hfdcan);
const char* rs06_status_str(ServoStatus status);
const char* rs06_mode_str(ServoMode mode);
ServoStatus rs06_enable(uint8_t motor_id);
ServoStatus rs06_stop(uint8_t motor_id);
ServoStatus rs06_change_id(uint8_t old_id, uint8_t new_id);
ServoStatus rs06_set_mode(uint8_t motor_id, ServoMode mode);
ServoStatus rs06_set_position(uint8_t motor_id, float angle_rad);
ServoStatus rs06_set_mit(uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
void rs06_reset(void);

// ! ========================= 新 增 函 数 声 明 ========================= ! //
ServoStatus rs06_config_reporting(uint8_t motor_id, uint16_t interval_ms);
ServoStatus rs06_parse_feedback(uint32_t id, uint8_t data[8], ServoFeedback* res);

#endif