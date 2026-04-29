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

    /**
     * @brief 将电机状态转换为字符串
     * @param status 电机状态值
     * @return 状态对应的字符串
     */
    const char* (*status_str)(MotorStatus status);

    /**
     * @brief 初始化电机系统
     * @return 初始化状态
     */
    MotorStatus(*init)(void);
    /**
     * @brief 设置电机反馈类型
     * @param feedback_cmd 反馈命令 (MOTOR_FEEDBACK_CMD_SPEED / _POSITION / _ERROR)
     * @param id 电机 ID
     * @return 操作状态
     */
    MotorStatus(*set_feedback)(uint8_t feedback_cmd, uint8_t id);
    /**
     * @brief 电机标定
     * @return 操作状态
     */
    MotorStatus(*calibration)(void);
    /**
     * @brief 设置电机 ID
     * @param id 新的电机 ID
     * @return 操作状态
     */
    MotorStatus(*set_id)(uint8_t id);
    /**
     * @brief 设置电机工作模式 (原始数据)
     * @param mode 工作模式值
     * @return 操作状态
     */
    MotorStatus(*set_mode_raw)(uint8_t mode);
    /**
     * @brief 设置电机转速 (RPM)
     * @param id 电机 ID
     * @param rpm 目标转速 (转/分)
     * @return 操作状态
     */
    MotorStatus(*set_speed)(uint8_t id, int16_t rpm);
    /**
     * @brief 设置电机转速 (RPS)
     * @param id 电机 ID
     * @param rps 目标转速 (转/秒)
     * @return 操作状态
     */
    MotorStatus(*set_speed_rps)(uint8_t id, float rps);
    /**
     * @brief 请求电机反馈报文
     * @param id 电机 ID
     * @param check1 校验值 1
     * @param check2 校验值 2
     * @param check3 校验值 3
     * @return 操作状态
     */
    MotorStatus(*request_report)(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3);
    /**
     * @brief 获取电机反馈数据
     * @param feedback 指向反馈数据结构的指针
     * @return 操作状态
     */
    MotorStatus(*get_feedback)(MotorFeedback* feedback);
    /**
     * @brief 设置电机转速 (弧度/秒)
     * @param id 电机 ID
     * @param rads 目标转速 (弧度/秒)
     * @return 操作状态
     */
    MotorStatus(*set_speed_rads)(uint8_t id, float rads);
    /**
     * @brief 获取电机当前位置
     * @param id 电机 ID
     * @return 当前位置
     */
    float(*get_pos)(uint8_t id);
    /**
     * @brief 获取电机当前转速
     * @param id 电机 ID
     * @return 当前转速 (转/秒)
     */
    float(*get_spd)(uint8_t id);
    /**
     * @brief 获取电机当前扭矩
     * @param id 电机 ID
     * @return 当前扭矩 (N.m)
     */
    float(*get_tor)(uint8_t id);
    // --- 新增弧度制接口 ---
    /**
     * @brief 获取电机当前位置 (弧度制)
     * @param id 电机 ID
     * @return 当前位置 (弧度)
     */
    float(*get_pos_rad)(uint8_t id);
    /**
     * @brief 获取电机当前转速 (弧度制)
     * @param id 电机 ID
     * @return 当前转速 (弧度/秒)
     */
    float(*get_spd_rads)(uint8_t id);
    // --------------------
    /**
     * @brief 获取电机最新上报数据
     * @param id 电机 ID
     * @param report 指向上报数据结构的指针
     * @return 操作状态
     */
    MotorStatus(*latest_report)(uint8_t id, MotorReport* report);
} *motor_instance;
#undef SX

extern const struct MotorInterface dm_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 设置电机实例
 * @param instance 指向 MotorInterface 结构的指针
 * @return 无
 */
void motor_set_instance(const struct MotorInterface* instance);

/**
 * @brief 将电机状态转换为字符串
 * @param status 电机状态值
 * @return 状态对应的字符串
 */
const char* dm_motor_status_str(MotorStatus status);

/**
 * @brief 初始化电机系统
 * @return 初始化状态
 */
MotorStatus dm_motor_init(void);
/**
 * @brief 设置电机反馈类型
 * @param feedback_cmd 反馈命令 (MOTOR_FEEDBACK_CMD_SPEED / _POSITION / _ERROR)
 * @param id 电机 ID
 * @return 操作状态
 */
MotorStatus dm_motor_set_feedback(uint8_t feedback_cmd, uint8_t id);
/**
 * @brief 电机标定
 * @return 操作状态
 */
MotorStatus dm_motor_calibration(void);
/**
 * @brief 设置电机 ID
 * @param id 新的电机 ID
 * @return 操作状态
 */
MotorStatus dm_motor_set_id(uint8_t id);
/**
 * @brief 设置电机工作模式 (原始数据)
 * @param mode 工作模式值
 * @return 操作状态
 */
MotorStatus dm_motor_set_mode_raw(uint8_t mode);
/**
 * @brief 设置电机转速 (RPM)
 * @param id 电机 ID
 * @param rpm 目标转速 (转/分)
 * @return 操作状态
 */
MotorStatus dm_motor_set_speed(uint8_t id, int16_t rpm);
/**
 * @brief 设置电机转速 (RPS)
 * @param id 电机 ID
 * @param rps 目标转速 (转/秒)
 * @return 操作状态
 */
MotorStatus dm_motor_set_speed_rps(uint8_t id, float rps);
/**
 * @brief 设置电机转速 (弧度/秒)
 * @param id 电机 ID
 * @param rads 目标转速 (弧度/秒)
 * @return 操作状态
 */
MotorStatus dm_motor_set_speed_rads(uint8_t id, float rads);
/**
 * @brief 请求电机反馈报文
 * @param id 电机 ID
 * @param check1 校验值 1
 * @param check2 校验值 2
 * @param check3 校验值 3
 * @return 操作状态
 */
MotorStatus dm_motor_request_report(uint8_t id, uint8_t check1, uint8_t check2, uint8_t check3);
/**
 * @brief 获取电机反馈数据
 * @param feedback 指向反馈数据结构的指针
 * @return 操作状态
 */
MotorStatus dm_motor_get_feedback(MotorFeedback* feedback);
/**
 * @brief 获取电机当前位置
 * @param id 电机 ID
 * @return 当前位置
 */
float dm_motor_get_pos(uint8_t id);
/**
 * @brief 获取电机当前转速 (转/秒)
 * @param id 电机 ID
 * @return 当前转速 (转/秒)
 */
float dm_motor_get_spd(uint8_t id);
/**
 * @brief 获取电机当前扭矩
 * @param id 电机 ID
 * @return 当前扭矩 (N.m)
 */
float dm_motor_get_tor(uint8_t id);
// --- 新增函数声明 ---
/**
 * @brief 获取电机当前位置 (弧度制)
 * @param id 电机 ID
 * @return 当前位置 (弧度)
 */
float dm_motor_get_pos_rad(uint8_t id);
/**
 * @brief 获取电机当前转速 (弧度制)
 * @param id 电机 ID
 * @return 当前转速 (弧度/秒)
 */
float dm_motor_get_spd_rads(uint8_t id);
// ------------------
/**
 * @brief 获取电机最新上报数据
 * @param id 电机 ID
 * @param report 指向上报数据结构的指针
 * @return 操作状态
 */
MotorStatus dm_motor_latest_report(uint8_t id, MotorReport* report);

#endif