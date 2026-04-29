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
#define SERVO_ID_MIN 1U
#define SERVO_ID_MAX 8U
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

    /**
     * @brief 将伺服状态转换为字符串
     * @param status 伺服状态值
     * @return 状态对应的字符串
     */
    const char* (*status_str)(ServoStatus status);
    /**
     * @brief 将伺服模式转换为字符串
     * @param mode 伺服模式值
     * @return 模式对应的字符串
     */
    const char* (*mode_str)(ServoMode mode);

    /**
     * @brief 初始化伺服系统
     * @param hfdcan CAN 句柄指针
     * @return 初始化状态
     */
    ServoStatus(*init)(FDCAN_HandleTypeDef* hfdcan);
    /**
     * @brief 使能指定电机
     * @param motor_id 电机 ID
     * @return 操作状态
     */
    ServoStatus(*enable)(uint8_t motor_id);
    /**
     * @brief 停止指定电机
     * @param motor_id 电机 ID
     * @return 操作状态
     */
    ServoStatus(*stop)(uint8_t motor_id);
    /**
     * @brief 修改电机 ID
     * @param old_id 旧的电机 ID
     * @param new_id 新的电机 ID
     * @return 操作状态
     */
    ServoStatus(*change_id)(uint8_t old_id, uint8_t new_id);
    /**
     * @brief 设置电机工作模式
     * @param motor_id 电机 ID
     * @param mode 工作模式
     * @return 操作状态
     */
    ServoStatus(*set_mode)(uint8_t motor_id, ServoMode mode);
    /**
     * @brief 设置电机位置 (位置模式)
     * @param motor_id 电机 ID
     * @param angle_rad 目标位置 (弧度)
     * @return 操作状态
     */
    ServoStatus(*set_position)(uint8_t motor_id, float angle_rad);
    /**
     * @brief 设置电机 MIT 模式 (力位置混合控制)
     * @param motor_id 电机 ID
     * @param angle_rad 目标位置 (弧度)
     * @param speed_rad_s 目标速度 (弧度/秒)
     * @param kp 位置比例系数
     * @param kd 速度微分系数
     * @param t_ff 扭矩前馈值
     * @return 操作状态
     */
    ServoStatus(*set_mit)(uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
    /**
     * @brief 复位伺服系统
     * @return 无
     */
    ServoStatus(*reset)(void);

    // ! ========================= 新 增 接口 ========================= ! //
    /**
     * @brief 配置电机反馈数据的上报周期
     * @param motor_id 电机 ID
     * @param interval_ms 上报周期 (毫秒)
     * @return 操作状态
     */
    ServoStatus(*config_reporting)(uint8_t motor_id, uint16_t interval_ms);
    /**
     * @brief 解析 CAN 反馈数据
     * @param id CAN 消息 ID
     * @param data 8 字节的 CAN 数据
     * @param res 指向反馈数据结构的指针，用于存储解析结果
     * @return 解析状态
     */
    ServoStatus(*parse_feedback)(uint32_t id, uint8_t data[8], ServoFeedback* res);

    /**
     * @brief 获取电机当前位置
     * @param motor_id 电机 ID
     * @return 当前位置 (弧度)
     */
    float(*get_pos)(uint8_t motor_id);
    /**
     * @brief 获取电机当前速度
     * @param motor_id 电机 ID
     * @return 当前速度 (弧度/秒)
     */
    float(*get_spd)(uint8_t motor_id);
    /**
     * @brief 获取电机当前扭矩
     * @param motor_id 电机 ID
     * @return 当前扭矩 (N.m)
     */
    float(*get_tor)(uint8_t motor_id);
}*servo_instance;
#undef SX
#undef MX
#undef TX

extern const struct ServoInterface rs06_servo_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 设置伺服实例
 * @param instance 指向 ServoInterface 结构的指针
 * @return 无
 */
void servo_set_instance(const struct ServoInterface* instance);
/**
 * @brief 初始化 RS06 伺服系统
 * @param hfdcan CAN 句柄指针
 * @return 初始化状态
 */
ServoStatus rs06_init(FDCAN_HandleTypeDef* hfdcan);
/**
 * @brief 将伺服状态转换为字符串
 * @param status 伺服状态值
 * @return 状态对应的字符串
 */
const char* rs06_status_str(ServoStatus status);
/**
 * @brief 将伺服模式转换为字符串
 * @param mode 伺服模式值
 * @return 模式对应的字符串
 */
const char* rs06_mode_str(ServoMode mode);
/**
 * @brief 使能指定电机
 * @param motor_id 电机 ID
 * @return 操作状态
 */
ServoStatus rs06_enable(uint8_t motor_id);
/**
 * @brief 停止指定电机
 * @param motor_id 电机 ID
 * @return 操作状态
 */
ServoStatus rs06_stop(uint8_t motor_id);
/**
 * @brief 修改电机 ID
 * @param old_id 旧的电机 ID
 * @param new_id 新的电机 ID
 * @return 操作状态
 */
ServoStatus rs06_change_id(uint8_t old_id, uint8_t new_id);
/**
 * @brief 设置电机工作模式
 * @param motor_id 电机 ID
 * @param mode 工作模式
 * @return 操作状态
 */
ServoStatus rs06_set_mode(uint8_t motor_id, ServoMode mode);
/**
 * @brief 设置电机位置 (位置模式)
 * @param motor_id 电机 ID
 * @param angle_rad 目标位置 (弧度)
 * @return 操作状态
 */
ServoStatus rs06_set_position(uint8_t motor_id, float angle_rad);
/**
 * @brief 设置电机 MIT 模式 (力位置混合控制)
 * @param motor_id 电机 ID
 * @param angle_rad 目标位置 (弧度)
 * @param speed_rad_s 目标速度 (弧度/秒)
 * @param kp 位置比例系数
 * @param kd 速度微分系数
 * @param t_ff 扭矩前馈值
 * @return 操作状态
 */
ServoStatus rs06_set_mit(uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
/**
 * @brief 复位伺服系统
 * @return 无
 */
ServoStatus rs06_reset(void);

// ! ========================= 新 增 函 数 声 明 ========================= ! //
/**
 * @brief 配置电机反馈数据的上报周期
 * @param motor_id 电机 ID
 * @param interval_ms 上报周期 (毫秒)
 * @return 操作状态
 */
ServoStatus rs06_config_reporting(uint8_t motor_id, uint16_t interval_ms);
/**
 * @brief 解析 CAN 反馈数据
 * @param id CAN 消息 ID
 * @param data 8 字节的 CAN 数据
 * @param res 指向反馈数据结构的指针，用于存储解析结果
 * @return 解析状态
 */
ServoStatus rs06_parse_feedback(uint32_t id, uint8_t data[8], ServoFeedback* res);
/**
 * @brief 获取电机反馈数据
 * @param out_data 指向反馈数据结构的指针，用于存储结果
 * @return 操作状态
 */
ServoStatus rs06_get_feedback(ServoFeedback* out_data);

/**
 * @brief 获取电机当前位置
 * @param motor_id 电机 ID
 * @return 当前位置 (弧度)
 */
float rs06_get_pos(uint8_t motor_id);
/**
 * @brief 获取电机当前速度
 * @param motor_id 电机 ID
 * @return 当前速度 (弧度/秒)
 */
float rs06_get_spd(uint8_t motor_id);
/**
 * @brief 获取电机当前扭矩
 * @param motor_id 电机 ID
 * @return 当前扭矩 (N.m)
 */
float rs06_get_tor(uint8_t motor_id);

#endif
