#ifndef _servo_h_
#define _servo_h_

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @def RS06_DEFAULT_HOST_ID
 * @brief RS06 私有协议默认主机 ID
 */
#ifndef RS06_DEFAULT_HOST_ID
#define RS06_DEFAULT_HOST_ID 0xFDU
#endif

/**
 * @def RS06_DEFAULT_TX_TIMEOUT_MS
 * @brief RS06 默认 CAN 发送等待超时时间，单位 ms
 */
#ifndef RS06_DEFAULT_TX_TIMEOUT_MS
#define RS06_DEFAULT_TX_TIMEOUT_MS CAN_BUS_TX_TIMEOUT_MS
#endif

/**
 * @brief RS06 私有协议常用参数索引
 * @note 参数索引用于 rs06_read_param() / rs06_write_param_xxx()
 */
#define RS06_PARAM_RUN_MODE        0x7005U  /**< 运行模式，uint8 */
#define RS06_PARAM_IQ_REF          0x7006U  /**< 电流模式 Iq 目标，float，单位 A */
#define RS06_PARAM_SPEED_REF       0x700AU  /**< 速度模式目标速度，float，单位 rad/s */
#define RS06_PARAM_POSITION_REF    0x7016U  /**< 位置目标，float，单位 rad */
#define RS06_PARAM_CSP_LIMIT_SPD   0x7017U  /**< CSP 速度限制，float，单位 rad/s */
#define RS06_PARAM_LIMIT_CUR       0x7018U  /**< 电流限制，float，单位 A */
#define RS06_PARAM_SPEED_ACCEL     0x7022U  /**< 速度模式加速度，float，单位 rad/s^2 */
#define RS06_PARAM_PP_LIMIT_SPD    0x7024U  /**< PP 速度限制，float，单位 rad/s */
#define RS06_PARAM_PP_ACCEL        0x7025U  /**< PP 加速度，float，单位 rad/s^2 */
#define RS06_PARAM_REPORT_TIME     0x7026U  /**< 主动上报周期，uint16，1 表示 10ms，之后每 +1 增加 5ms */
#define RS06_PARAM_CAN_TIMEOUT     0x7028U  /**< CAN 超时阈值，uint32，20000 约等于 1s */
#define RS06_PARAM_ZERO_STA        0x7029U  /**< 零点标志位，uint8，0: 0~2pi，1: -pi~pi */
#define RS06_PARAM_DAMPER          0x702AU  /**< 反驱阻尼开关，uint8 */
#define RS06_PARAM_ADD_OFFSET      0x702BU  /**< 零位偏置，float */
#define RS06_PARAM_ALVEOLOUS_OPEN  0x702CU  /**< 齿槽补偿开关，uint8 */
#define RS06_PARAM_IQ_TEST         0x702DU  /**< 初始化校准开关，uint8 */
#define RS06_PARAM_PP_DECEL        0x702EU  /**< PP 减速度，float，单位 rad/s^2 */

/**
 * @brief RS06 状态码
 */
typedef enum {
    RS06_OK = 0,       /**< 操作成功 */
    RS06_CAN_ERROR,    /**< CAN 底层发送或接收失败 */
    RS06_INVALID_ARG,  /**< 输入参数无效 */
} rs06_status_t;

/**
 * @brief RS06 私有协议运行模式
 */
typedef enum {
    RS06_MODE_MOTION = 0,   /**< 运控模式 */
    RS06_MODE_PP = 1,       /**< 插补位置模式 PP */
    RS06_MODE_SPEED = 2,    /**< 速度模式 */
    RS06_MODE_CURRENT = 3,  /**< 电流模式 */
    RS06_MODE_CSP = 5,      /**< 周期同步位置模式 CSP */
} rs06_mode_t;

/**
 * @brief RS06 反馈帧中的电机状态模式
 */
typedef enum {
    RS06_PATTERN_RESET = 0,  /**< Reset 复位模式 */
    RS06_PATTERN_CALI = 1,   /**< Cali 标定模式 */
    RS06_PATTERN_MOTOR = 2,  /**< Motor 运行模式 */
} rs06_pattern_t;

/**
 * @brief RS06 协议类型
 * @note 协议切换通常需要电机重新上电后生效
 */
typedef enum {
    RS06_PROTOCOL_PRIVATE = 0,  /**< 灵足私有协议 */
    RS06_PROTOCOL_CANOPEN = 1,  /**< CANopen 协议 */
    RS06_PROTOCOL_MIT = 2,      /**< MIT 协议 */
} rs06_protocol_t;

/**
 * @brief RS06 波特率配置
 */
typedef enum {
    RS06_BAUD_1M = 1,    /**< 1 Mbps */
    RS06_BAUD_500K = 2,  /**< 500 kbps */
    RS06_BAUD_250K = 3,  /**< 250 kbps */
    RS06_BAUD_125K = 4,  /**< 125 kbps */
} rs06_baud_t;

/**
 * @brief RS06 MIT 协议运行模式命令字
 */
typedef enum {
    RS06_MIT_RUN_MODE_MIT = 0,       /**< MIT 运控模式 */
    RS06_MIT_RUN_MODE_POSITION = 1,  /**< MIT 位置模式 */
    RS06_MIT_RUN_MODE_SPEED = 2,     /**< MIT 速度模式 */
} rs06_mit_run_mode_t;

/**
 * @brief RS06 电机实例
 */
typedef struct {
    FDCAN_HandleTypeDef* bus;  /**< 所属 FDCAN 句柄 */
    uint8_t host_id;           /**< 主机 ID，私有协议默认 0xFD */
    uint8_t motor_id;          /**< 电机 CAN ID */
    uint32_t tx_timeout_ms;    /**< 发送等待超时时间，单位 ms */
} rs06_t;

/**
 * @brief RS06 私有协议类型 2 反馈数据
 */
typedef struct {
    uint8_t motor_id;             /**< 电机 ID */
    uint8_t host_id;              /**< 主机 ID */
    uint8_t fault_bits;           /**< 反馈 ID 中的故障位 */
    rs06_pattern_t pattern;       /**< 电机状态模式 */
    float position_rad;           /**< 当前位置，单位 rad */
    float velocity_rad_s;         /**< 当前速度，单位 rad/s */
    float torque_nm;              /**< 当前力矩，单位 N·m */
    float temperature_c;          /**< 当前温度，单位 ℃ */
} rs06_feedback_t;

/**
 * @brief RS06 参数读取 / 写入响应
 */
typedef struct {
    uint8_t motor_id;     /**< 电机 ID */
    uint8_t host_id;      /**< 主机 ID 或响应目标 ID */
    uint16_t index;       /**< 参数索引 */
    uint8_t value_u8;     /**< Byte4 按 uint8 解释的参数值 */
    uint16_t value_u16;   /**< Byte4~5 按 uint16 小端解释的参数值 */
    uint32_t value_u32;   /**< Byte4~7 按 uint32 小端解释的参数值 */
    float value_float;    /**< Byte4~7 按 float 小端解释的参数值 */
    uint8_t raw[8];       /**< 原始数据区 */
} rs06_param_reply_t;

/**
 * @brief RS06 设备 ID 响应
 */
typedef struct {
    uint8_t motor_id;       /**< 电机 ID */
    uint8_t target_id;      /**< 响应目标 ID，通常为 0xFE 或主机 ID */
    uint8_t unique_id[8];   /**< MCU 64 bit 唯一标识原始数据 */
} rs06_device_id_reply_t;

/**
 * @brief RS06 版本号响应
 */
typedef struct {
    uint8_t motor_id;      /**< 电机 ID */
    uint8_t host_id;       /**< 主机 ID */
    uint32_t version;      /**< 版本号原始值，高字节在前 */
    uint8_t raw[8];        /**< 原始数据区 */
} rs06_version_reply_t;

/**
 * @brief RS06 MIT 协议反馈数据
 */
typedef struct {
    uint8_t motor_id;            /**< 电机 ID */
    uint8_t has_fault;           /**< 是否存在故障 */
    uint8_t has_warning;         /**< 是否存在预警 */
    rs06_pattern_t pattern;      /**< 电机状态模式 */
    float position_rad;          /**< 当前位置，单位 rad */
    float velocity_rad_s;        /**< 当前速度，单位 rad/s */
    float torque_nm;             /**< 当前力矩，单位 N·m */
    float temperature_c;         /**< 当前温度，单位 ℃ */
} rs06_mit_feedback_t;

/**
 * @brief RS06 MIT 协议异常状态反馈
 */
typedef struct {
    uint8_t motor_id;      /**< 电机 ID */
    uint32_t fault;        /**< fault 原始值 */
    uint32_t warning;      /**< warning 原始值；若协议未返回则为 0 */
    uint8_t raw[8];        /**< 原始数据区 */
} rs06_mit_fault_reply_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void rs06_init(rs06_t* motor, FDCAN_HandleTypeDef* bus, uint8_t motor_id, uint8_t host_id);
void rs06_set_tx_timeout(rs06_t* motor, uint32_t timeout_ms);

const char* rs06_status_to_str(rs06_status_t status);
const char* rs06_mode_to_str(rs06_mode_t mode);

rs06_status_t rs06_get_device_id(const rs06_t* motor);
rs06_status_t rs06_get_version(const rs06_t* motor);
rs06_status_t rs06_enable(const rs06_t* motor);
rs06_status_t rs06_stop(const rs06_t* motor, bool clear_error);
rs06_status_t rs06_clear_error(const rs06_t* motor);
rs06_status_t rs06_set_mechanical_zero(const rs06_t* motor);
rs06_status_t rs06_set_can_id(const rs06_t* motor, uint8_t new_motor_id);
rs06_status_t rs06_set_baudrate(const rs06_t* motor, rs06_baud_t baud);
rs06_status_t rs06_set_active_report(const rs06_t* motor, bool enable);
rs06_status_t rs06_save_config(const rs06_t* motor);
rs06_status_t rs06_set_protocol(const rs06_t* motor, rs06_protocol_t protocol);
rs06_status_t rs06_zeroing_and_save(const rs06_t* motor);

rs06_status_t rs06_read_param(const rs06_t* motor, uint16_t index);
rs06_status_t rs06_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value);
rs06_status_t rs06_write_param_u16(const rs06_t* motor, uint16_t index, uint16_t value);
rs06_status_t rs06_write_param_u32(const rs06_t* motor, uint16_t index, uint32_t value);
rs06_status_t rs06_write_param_float(const rs06_t* motor, uint16_t index, float value);

rs06_status_t rs06_set_mode(const rs06_t* motor, rs06_mode_t mode);
rs06_status_t rs06_set_zero_sta(const rs06_t* motor, uint8_t zero_sta, bool save);
rs06_status_t rs06_set_report_period(const rs06_t* motor, uint16_t period_value, bool save);
rs06_status_t rs06_set_can_timeout(const rs06_t* motor, uint32_t timeout_value, bool save);
rs06_status_t rs06_set_damper(const rs06_t* motor, bool enable, bool save);
rs06_status_t rs06_set_add_offset(const rs06_t* motor, float offset_rad, bool save);
rs06_status_t rs06_set_alveolous_comp(const rs06_t* motor, bool enable, bool save);
rs06_status_t rs06_set_iq_test(const rs06_t* motor, bool enable, bool save);

rs06_status_t rs06_set_current_target(const rs06_t* motor, float current_a);
rs06_status_t rs06_current_control(const rs06_t* motor, float current_a);

rs06_status_t rs06_set_speed_config(const rs06_t* motor, float limit_cur_a, float accel_rad_s2);
rs06_status_t rs06_set_speed_target(const rs06_t* motor, float speed_rad_s);
rs06_status_t rs06_speed_control(const rs06_t* motor, float speed_rad_s, float limit_cur_a, float accel_rad_s2);

rs06_status_t rs06_set_pp_config(const rs06_t* motor, float speed_rad_s, float accel_rad_s2);
rs06_status_t rs06_set_pp_decel(const rs06_t* motor, float decel_rad_s2, bool save);
rs06_status_t rs06_set_position_target(const rs06_t* motor, float position_rad);
rs06_status_t rs06_pp_goto(const rs06_t* motor, float position_rad, float speed_rad_s, float accel_rad_s2);

rs06_status_t rs06_set_csp_config(const rs06_t* motor, float speed_rad_s);
rs06_status_t rs06_csp_goto(const rs06_t* motor, float position_rad, float speed_rad_s);

rs06_status_t rs06_motion_control(const rs06_t* motor, float torque_nm, float position_rad, float velocity_rad_s, float kp, float kd);

bool rs06_parse_feedback(const can_bus_frame_t* frame, rs06_feedback_t* feedback);
bool rs06_parse_param_reply(const can_bus_frame_t* frame, rs06_param_reply_t* reply);
bool rs06_parse_device_id_reply(const can_bus_frame_t* frame, rs06_device_id_reply_t* reply);
bool rs06_parse_version_reply(const can_bus_frame_t* frame, rs06_version_reply_t* reply);

rs06_status_t rs06_mit_enable(const rs06_t* motor);
rs06_status_t rs06_mit_disable(const rs06_t* motor);
rs06_status_t rs06_mit_set_zero_pos(const rs06_t* motor);
rs06_status_t rs06_mit_clear_or_check_error(const rs06_t* motor, uint8_t cmd);
rs06_status_t rs06_mit_set_run_mode(const rs06_t* motor, rs06_mit_run_mode_t mode);
rs06_status_t rs06_mit_set_motor_id(const rs06_t* motor, uint8_t new_motor_id);
rs06_status_t rs06_mit_set_protocol(const rs06_t* motor, rs06_protocol_t protocol);
rs06_status_t rs06_mit_set_host_id(const rs06_t* motor, uint8_t host_id);
rs06_status_t rs06_mit_save_config(const rs06_t* motor);
rs06_status_t rs06_mit_set_active_report(const rs06_t* motor, bool enable);
rs06_status_t rs06_mit_read_param(const rs06_t* motor, uint16_t index);
rs06_status_t rs06_mit_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value);
rs06_status_t rs06_mit_write_param_u16(const rs06_t* motor, uint16_t index, uint16_t value);
rs06_status_t rs06_mit_write_param_u32(const rs06_t* motor, uint16_t index, uint32_t value);
rs06_status_t rs06_mit_write_param_float(const rs06_t* motor, uint16_t index, float value);
rs06_status_t rs06_mit_control(const rs06_t* motor, float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
rs06_status_t rs06_mit_position_control(const rs06_t* motor, float position_rad, float speed_rad_s);
rs06_status_t rs06_mit_speed_control(const rs06_t* motor, float speed_rad_s, float current_limit_a);

/* 兼容旧命名：旧工程可继续调用 rs06_mit_set_motor_type() */
rs06_status_t rs06_mit_set_motor_type(const rs06_t* motor, uint8_t mode_cmd);

bool rs06_mit_parse_feedback(const can_bus_frame_t* frame, rs06_mit_feedback_t* feedback);
bool rs06_mit_parse_fault_reply(const can_bus_frame_t* frame, rs06_mit_fault_reply_t* reply);
bool rs06_mit_parse_param_reply(const can_bus_frame_t* frame, rs06_param_reply_t* reply);
bool rs06_mit_parse_device_id_reply(const can_bus_frame_t* frame, rs06_device_id_reply_t* reply);

#endif
