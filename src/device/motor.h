#ifndef _motor_h_
#define _motor_h_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 电机单例，用户自定义名称
 */
#define motor dm_motor_instance

/**
 * @brief 达妙电机状态码表，使用 X-Macro 定义，方便维护和扩展
 */
#define DM_MOTOR_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(TIMEOUT, 2) \
    X(ID_MISMATCH, 3)

/**
 * @brief 达妙电机模式表，使用 X-Macro 定义，方便维护和扩展
 */
#define DM_MOTOR_MODE_TABLE \
    Y(MIT, 1) \
    Y(POS_SPD, 2) \
    Y(SPD, 3) \
    Y(POS_SPD_CUR, 4)

/**
 * @brief 达妙电机状态码，由 X-Macro 自动生成枚举类型
 */
#define X(name, value) DM_MOTOR_##name = value,
typedef enum {
    DM_MOTOR_STATUS_TABLE
} DmMotorStatus;
#undef X

/**
 * @brief DmMotorMode 枚举类型，表示电机的工作模式
 */
#define Y(name, value) DM_MOTOR_MODE_##name = value,
typedef enum {
    DM_MOTOR_MODE_TABLE
} DmMotorMode;
#undef Y

/**
 * @brief 电机反馈信息结构体
 */
typedef struct {
    uint16_t id;
    uint8_t err_code;
    float pos;
    float spd;
    float torque;
} DmMotorFeedback;

typedef struct {
    int16_t fb_speed;
    int16_t e_curru;
    int16_t position;
    uint8_t err_code;
    uint8_t fb_mode;
} DmMotorReport;

/// @brief 电机控制命令的长度，单位为字节
#define DM_MOTOR_CMD_LEN 8
/// @brief pos 上下限
#define DM_MOTOR_POS_LIMIT 12.5f
/// @brief spd 上下限
#define DM_MOTOR_SPD_LIMIT 10.0f
/// @brief torque 上下限
#define DM_MOTOR_TORQUE_LIMIT 28.0f
/// @brief kp 上限
#define DM_MOTOR_KP_LIMIT 500.0f
/// @brief kd 上限
#define DM_MOTOR_KD_LIMIT 5.0f

/**
 * @brief 电机接口结构体，包含所有电机相关的函数指针
 */
#define X(name, value) const DmMotorStatus name;
#define Y(name, value) const DmMotorMode name;
extern const struct DmMotorInterface {
    struct {
        DM_MOTOR_STATUS_TABLE
    };
    struct {
        DM_MOTOR_MODE_TABLE
    };

    const char* (*status_str)(DmMotorStatus status);
    const char* (*mode_str)(DmMotorMode mode);

    DmMotorStatus(*enable)(uint16_t id);
    DmMotorStatus(*disable)(uint16_t id);

    DmMotorStatus(*set_mit)(uint16_t id, float pos, float spd, float kp, float kd, float torque);
    DmMotorStatus(*set_pos_spd)(uint16_t id, float pos, float spd);
    DmMotorStatus(*set_spd)(uint16_t id, float spd);
    DmMotorStatus(*set_pos_spd_cur)(uint16_t id, float pos, float spd, float cur);

    DmMotorStatus(*get_feedback)(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
    DmMotorStatus(*get_err_code)(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
    DmMotorStatus(*get_pos)(uint16_t id, float* pos, uint32_t timeout_ms);
    DmMotorStatus(*get_spd)(uint16_t id, float* spd, uint32_t timeout_ms);
    DmMotorStatus(*get_torque)(uint16_t id, float* torque, uint32_t timeout_ms);

    DmMotorStatus(*request_feedback)(uint16_t id);
    DmMotorStatus(*update)(DmMotorFeedback* feedback);

    void (*init)(void);
    DmMotorStatus(*set_feedback)(uint8_t feedback_cmd, uint8_t id);
    void (*calibration)(void);
    void (*check)(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3, DmMotorReport* report);
    void (*obtain_report)(DmMotorReport* report);
    DmMotorStatus(*set_id)(uint8_t id);
    DmMotorStatus(*set_mode_raw)(uint8_t mode);
    void (*speed_control)(int16_t input_rpm, uint8_t id);
    void (*control_all)(int16_t rpms[4]);
    void (*speed_control_smooth)(int16_t input_rpm, uint8_t id);
    void (*stop_immediately)(uint8_t id);
    void (*monitor_read)(void);
} dm_motor_instance;
#undef X
#undef Y

// ! ========================= 接 口 函 数 声 明 ========================= ! //

const char* dm_status_str(DmMotorStatus status);
const char* dm_mode_str(DmMotorMode mode);

DmMotorStatus dm_enable(uint16_t id);
DmMotorStatus dm_disable(uint16_t id);

DmMotorStatus dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque);
DmMotorStatus dm_set_pos_spd(uint16_t id, float pos, float spd);
DmMotorStatus dm_set_spd(uint16_t id, float spd);
DmMotorStatus dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur);

DmMotorStatus dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
DmMotorStatus dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
DmMotorStatus dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms);
DmMotorStatus dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms);
DmMotorStatus dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms);

DmMotorStatus dm_request_feedback(uint16_t id);
DmMotorStatus dm_update(DmMotorFeedback* feedback);

void dm_init(void);
DmMotorStatus dm_set_feedback(uint8_t feedback_cmd, uint8_t id);
void dm_calibration(void);
void dm_check(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3, DmMotorReport* report);
void dm_obtain_report(DmMotorReport* report);
DmMotorStatus dm_set_id(uint8_t id);
DmMotorStatus dm_set_mode_raw(uint8_t mode);
void dm_speed_control(int16_t input_rpm, uint8_t id);
void dm_control_all(int16_t rpms[4]);
void dm_speed_control_smooth(int16_t input_rpm, uint8_t id);
void dm_stop_immediately(uint8_t id);
void dm_monitor_read(void);

extern DmMotorReport dm_reporter_cache[4];
extern uint8_t dm_query_id;

#endif