#ifndef _servo_h_
#define _servo_h_

#include "main.h"

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 舵机单例，用户自定义名称
 */
#define servo (*servo_instance)

#define HOST_ID 0xFD
#define EXT_ID(type, host, motor) (((uint32_t)(type) << 16) | ((uint32_t)(host) << 8) | ((uint32_t)(motor)))

#define PARAM_RUN_MODE 0x7005
#define PARAM_LOC_REF 0x7016
#define PARAM_LIMIT_SPD 0x7017

#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f

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
	TX(WR_PARAM, 0x1200)

#define SX(name, value) SERVO_STATUS_##name = value,
typedef enum {
    SERVO_STATUS_TABLE
} Rs06ServoStatus;
#undef SX

#define MX(name, value) SERVO_MODE_##name = value,
typedef enum {
    SERVO_MODE_TABLE
} Rs06ServoMode;
#undef MX

#define TX(name, value) SERVO_TYPE_##name = value,
typedef enum {
    SERVO_TYPE_TABLE
} Rs06ServoType;
#undef TX

#define SX(name, value) const Rs06ServoStatus name;
#define MX(name, value) const Rs06ServoMode name;
#define TX(name, value) const Rs06ServoType name;
extern const struct Rs06ServoInterface {
    struct {
        SERVO_STATUS_TABLE
    };
    struct {
        SERVO_MODE_TABLE
    };
    struct {
        SERVO_TYPE_TABLE
    };

    const char* (*status_str)(Rs06ServoStatus status);
    const char* (*mode_str)(Rs06ServoMode mode);

    Rs06ServoStatus(*enable)(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
    Rs06ServoStatus(*stop)(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
    Rs06ServoStatus(*change_id)(FDCAN_HandleTypeDef* hfdcan, uint8_t old_id, uint8_t new_id);
    Rs06ServoStatus(*set_mode)(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, Rs06ServoMode mode);
    Rs06ServoStatus(*set_position_target)(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad);
    Rs06ServoStatus(*set_position)(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
    void (*turn)(void);
}*servo_instance;
#undef SX
#undef MX
#undef TX

extern const struct Rs06ServoInterface rs06_servo_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void servo_set_instance(const struct Rs06ServoInterface* instance);

const char* rs06_status_str(Rs06ServoStatus status);
const char* rs06_mode_str(Rs06ServoMode mode);

Rs06ServoStatus rs06_enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
Rs06ServoStatus rs06_stop(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
Rs06ServoStatus rs06_change_id(FDCAN_HandleTypeDef* hfdcan, uint8_t old_id, uint8_t new_id);
Rs06ServoStatus rs06_set_mode(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, Rs06ServoMode mode);
Rs06ServoStatus rs06_set_position_target(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad);
Rs06ServoStatus rs06_set_position(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
void rs06_turn(void);

#endif