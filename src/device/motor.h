#ifndef _motor_h_
#define _motor_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define motor (*motor_instance)

#define MOTOR_ID_MIN 1U
#define MOTOR_ID_MAX 8U

#define MOTOR_RPM_MAX 210
#define MOTOR_RPM_MIN -210

#define MOTOR_FEEDBACK_CMD_SPEED 1U
#define MOTOR_FEEDBACK_CMD_POSITION 4U
#define MOTOR_FEEDBACK_CMD_ERROR 5U

#define MOTOR_STATUS_TABLE \
	SX(OK, 0) \
	SX(ERROR, 1) \
	SX(TIMEOUT, 2) \
	SX(PARAM_INVALID, 3) \
	SX(NO_DATA, 4)

#define SX(name, value) MOTOR_STATUS_##name = value,
typedef enum {
    MOTOR_STATUS_TABLE
} MotorStatus;
#undef SX

typedef struct {
    uint8_t id;
    int16_t fb_speed;
    int16_t e_curru;
    int16_t position;
    uint8_t err_code;
    uint8_t fb_mode;
} MotorReport;

typedef struct {
    uint8_t id;
    float spd;
    float torque;
    float pos;
    uint8_t err_code;
} MotorFeedback;

#define SX(name, value) const MotorStatus name;
extern const struct MotorInterface {
    struct {
        MOTOR_STATUS_TABLE
    };

    const char* (*status_str)(MotorStatus status);

    MotorStatus(*init)(void);
    MotorStatus(*set_feedback)(uint8_t feedback_cmd, uint8_t id);
    MotorStatus(*calibration)(void);
    MotorStatus(*set_id)(uint8_t id);
    MotorStatus(*set_mode_raw)(uint8_t mode);
    MotorStatus(*set_speed)(uint8_t id, int16_t rpm);
    MotorStatus(*request_report)(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3);
    MotorStatus(*update)(MotorFeedback* feedback);
    MotorStatus(*latest_report)(uint8_t id, MotorReport* report);
} *motor_instance;
#undef SX

extern const struct MotorInterface dm_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void motor_set_instance(const struct MotorInterface* instance);

const char* dm_motor_status_str(MotorStatus status);

MotorStatus dm_motor_init(void);
MotorStatus dm_motor_set_feedback(uint8_t feedback_cmd, uint8_t id);
MotorStatus dm_motor_calibration(void);
MotorStatus dm_motor_set_id(uint8_t id);
MotorStatus dm_motor_set_mode_raw(uint8_t mode);
MotorStatus dm_motor_set_speed(uint8_t id, int16_t rpm);
MotorStatus dm_motor_request_report(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3);
MotorStatus dm_motor_update(MotorFeedback* feedback);
MotorStatus dm_motor_latest_report(uint8_t id, MotorReport* report);

#endif