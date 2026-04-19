#ifndef _servo_h_
#define _servo_h_

#include "main.h"

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 舵机单例，用户自定义名称
 */
#define servo rs06_servo_instance

#define RS06_HOST_ID 0xFD
#define RS06_EXT_ID(type, host, motor) (((uint32_t)(type) << 16) | ((uint32_t)(host) << 8) | ((uint32_t)(motor)))

#define RS06_PARAM_RUN_MODE 0x7005
#define RS06_PARAM_LOC_REF 0x7016
#define RS06_PARAM_LIMIT_SPD 0x7017

#define RS06_P_MIN -12.57f
#define RS06_P_MAX 12.57f
#define RS06_V_MIN -50.0f
#define RS06_V_MAX 50.0f
#define RS06_KP_MIN 0.0f
#define RS06_KP_MAX 5000.0f
#define RS06_KD_MIN 0.0f
#define RS06_KD_MAX 100.0f

#define RS06_SERVO_STATUS_TABLE \
	RSX(OK, 0) \
	RSX(ERROR, 1) \
	RSX(TIMEOUT, 2) \
	RSX(PARAM_INVALID, 3)

#define RS06_SERVO_MODE_TABLE \
	RMX(MIT, 0) \
	RMX(PP, 1) \
	RMX(CSP, 5)

#define RS06_SERVO_TYPE_TABLE \
	RTX(RUN, 0x0100) \
	RTX(ENABLE, 0x0300) \
	RTX(STOP, 0x0400) \
	RTX(SET_ZERO, 0x0600) \
	RTX(SET_ID, 0x0701) \
	RTX(WR_PARAM, 0x1200)

#define RSX(name, value) RS06_SERVO_STATUS_##name = value,
typedef enum {
    RS06_SERVO_STATUS_TABLE
} Rs06ServoStatus;
#undef RSX

#define RMX(name, value) RS06_SERVO_MODE_##name = value,
typedef enum {
    RS06_SERVO_MODE_TABLE
} Rs06ServoMode;
#undef RMX

#define RTX(name, value) RS06_SERVO_TYPE_##name = value,
typedef enum {
    RS06_SERVO_TYPE_TABLE
} Rs06ServoType;
#undef RTX

#define RSX(name, value) const Rs06ServoStatus name;
#define RMX(name, value) const Rs06ServoMode name;
#define RTX(name, value) const Rs06ServoType name;
extern const struct Rs06ServoInterface {
    struct {
        RS06_SERVO_STATUS_TABLE
    };
    struct {
        RS06_SERVO_MODE_TABLE
    };
    struct {
        RS06_SERVO_TYPE_TABLE
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
} rs06_servo_instance;
#undef RSX
#undef RMX
#undef RTX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

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