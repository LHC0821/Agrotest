#ifndef _motor_h_
#define _motor_h_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 电机单例，用户自定义名称
 */
#define motor (*motor_instance)

/**
 * @brief 达妙电机状态码表，使用 SX-Macro 定义，方便维护和扩展
 */
#define MOTOR_STATUS_TABLE \
    SX(OK, 0) \
    SX(ERROR, 1) \
    SX(TIMEOUT, 2) \
    SX(ID_MISMATCH, 3)

/**
 * @brief 达妙电机模式表，使用 SX-Macro 定义，方便维护和扩展
 */
#define MOTOR_MODE_TABLE \
    MX(MIT, 1) \
    MX(POS_SPD, 2) \
    MX(SPD, 3) \
    MX(POS_SPD_CUR, 4)

/**
 * @brief 达妙电机状态码，由 SX-Macro 自动生成枚举类型
 */
#define SX(name, value) MOTOR_##name = value,
typedef enum {
    MOTOR_STATUS_TABLE
} MotorStatus;
#undef SX

/**
 * @brief MotorMode 枚举类型，表示电机的工作模式
 */
#define MX(name, value) MOTOR_MODE_##name = value,
typedef enum {
    MOTOR_MODE_TABLE
} MotorMode;
#undef MX

/**
 * @brief 电机反馈信息结构体
 */
typedef struct {
    uint16_t id;
    uint8_t err_code;
    float pos;
    float spd;
    float torque;
} MotorFeedback;

typedef struct {
    int16_t fb_speed;
    int16_t e_curru;
    int16_t position;
    uint8_t err_code;
    uint8_t fb_mode;
} MotorReport;

/// @brief 电机控制命令的长度，单位为字节
#define MOTOR_CMD_LEN 8
/// @brief pos 上下限
#define MOTOR_POS_LIMIT 12.5f
/// @brief spd 上下限
#define MOTOR_SPD_LIMIT 10.0f
/// @brief torque 上下限
#define MOTOR_TORQUE_LIMIT 28.0f
/// @brief kp 上限
#define MOTOR_KP_LIMIT 500.0f
/// @brief kd 上限
#define MOTOR_KD_LIMIT 5.0f

/**
 * @brief 电机接口结构体，包含所有电机相关的函数指针
 */
#define SX(name, value) const MotorStatus name;
#define MX(name, value) const MotorMode name;
extern const struct MotorInterface {
    struct {
        MOTOR_STATUS_TABLE
    };
    struct {
        MOTOR_MODE_TABLE
    };

    const char* (*status_str)(MotorStatus status);
    const char* (*mode_str)(MotorMode mode);

    MotorStatus(*enable)(uint16_t id);
    MotorStatus(*disable)(uint16_t id);

    MotorStatus(*set_mit)(uint16_t id, float pos, float spd, float kp, float kd, float torque);
    MotorStatus(*set_pos_spd)(uint16_t id, float pos, float spd);
    MotorStatus(*set_spd)(uint16_t id, float spd);
    MotorStatus(*set_pos_spd_cur)(uint16_t id, float pos, float spd, float cur);

    MotorStatus(*get_feedback)(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
    MotorStatus(*get_err_code)(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
    MotorStatus(*get_pos)(uint16_t id, float* pos, uint32_t timeout_ms);
    MotorStatus(*get_spd)(uint16_t id, float* spd, uint32_t timeout_ms);
    MotorStatus(*get_torque)(uint16_t id, float* torque, uint32_t timeout_ms);

    MotorStatus(*request_feedback)(uint16_t id);
    MotorStatus(*update)(MotorFeedback* feedback);

    void (*init)(void);
    MotorStatus(*set_feedback)(uint8_t feedback_cmd, uint8_t id);
    void (*calibration)(void);
    void (*check)(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3, MotorReport* report);
    void (*obtain_report)(MotorReport* report);
    MotorStatus(*set_id)(uint8_t id);
    MotorStatus(*set_mode_raw)(uint8_t mode);
    void (*speed_control)(int16_t input_rpm, uint8_t id);
    void (*control_all)(int16_t rpms[4]);
    void (*speed_control_smooth)(int16_t input_rpm, uint8_t id);
    void (*stop_immediately)(uint8_t id);
    void (*monitor_read)(void);
}*motor_instance;
#undef SX
#undef MX

extern const struct MotorInterface dm_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void motor_set_instance(const struct MotorInterface* instance);

const char* dm_status_str(MotorStatus status);
const char* dm_mode_str(MotorMode mode);

MotorStatus dm_enable(uint16_t id);
MotorStatus dm_disable(uint16_t id);

MotorStatus dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque);
MotorStatus dm_set_pos_spd(uint16_t id, float pos, float spd);
MotorStatus dm_set_spd(uint16_t id, float spd);
MotorStatus dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur);

MotorStatus dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
MotorStatus dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
MotorStatus dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms);
MotorStatus dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms);
MotorStatus dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms);

MotorStatus dm_request_feedback(uint16_t id);
MotorStatus dm_update(MotorFeedback* feedback);

void dm_init(void);
MotorStatus dm_set_feedback(uint8_t feedback_cmd, uint8_t id);
void dm_calibration(void);
void dm_check(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3, MotorReport* report);
void dm_obtain_report(MotorReport* report);
MotorStatus dm_set_id(uint8_t id);
MotorStatus dm_set_mode_raw(uint8_t mode);
void dm_speed_control(int16_t input_rpm, uint8_t id);
void dm_control_all(int16_t rpms[4]);
void dm_speed_control_smooth(int16_t input_rpm, uint8_t id);
void dm_stop_immediately(uint8_t id);
void dm_monitor_read(void);

extern MotorReport dm_reporter_cache[4];
extern uint8_t dm_query_id;

#endif