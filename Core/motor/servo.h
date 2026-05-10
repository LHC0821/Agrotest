#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

#define RS06_DEFAULT_HOST_ID 0xFDU

typedef enum {
    RS06_OK = 0,
    RS06_CAN_ERROR,
    RS06_INVALID_ARG,
} rs06_status_t;

typedef enum {
    RS06_MODE_MOTION = 0,   /* 运控模式 */
    RS06_MODE_PP = 1,       /* 插补位置模式 */
    RS06_MODE_SPEED = 2,
    RS06_MODE_CURRENT = 3,
    RS06_MODE_CSP = 5,
} rs06_mode_t;

typedef enum {
    RS06_PATTERN_RESET = 0,
    RS06_PATTERN_CALI = 1,
    RS06_PATTERN_MOTOR = 2,
} rs06_pattern_t;

typedef struct {
    FDCAN_HandleTypeDef* bus;
    uint8_t host_id;
    uint8_t motor_id;
    uint32_t tx_timeout_ms;
} rs06_t;

typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    uint8_t fault_bits;
    rs06_pattern_t pattern;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} rs06_feedback_t;

void rs06_init(rs06_t* motor,
    FDCAN_HandleTypeDef* bus,
    uint8_t motor_id,
    uint8_t host_id);

rs06_status_t rs06_enable(const rs06_t* motor);
rs06_status_t rs06_stop(const rs06_t* motor, bool clear_error);
rs06_status_t rs06_set_mode(const rs06_t* motor, rs06_mode_t mode);
rs06_status_t rs06_set_mechanical_zero(const rs06_t* motor);
rs06_status_t rs06_save_config(const rs06_t* motor);
rs06_status_t rs06_zeroing_and_save(const rs06_t* motor);

rs06_status_t rs06_read_param(const rs06_t* motor, uint16_t index);
rs06_status_t rs06_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value);
rs06_status_t rs06_write_param_float(const rs06_t* motor, uint16_t index, float value);

rs06_status_t rs06_set_zero_sta(const rs06_t* motor, uint8_t zero_sta, bool save);
rs06_status_t rs06_set_pp_config(const rs06_t* motor, float speed_rad_s, float accel_rad_s2);
rs06_status_t rs06_set_csp_config(const rs06_t* motor, float speed_rad_s);
rs06_status_t rs06_set_position_target(const rs06_t* motor, float position_rad);
rs06_status_t rs06_pp_goto(const rs06_t* motor, float position_rad, float speed_rad_s, float accel_rad_s2);
rs06_status_t rs06_csp_goto(const rs06_t* motor, float position_rad, float speed_rad_s);

rs06_status_t rs06_motion_control(const rs06_t* motor, float torque_nm, float position_rad, float velocity_rad_s, float kp, float kd);

bool rs06_parse_feedback(const can_bus_frame_t* frame, rs06_feedback_t* feedback);

#endif /* RS06_MOTOR_H */
