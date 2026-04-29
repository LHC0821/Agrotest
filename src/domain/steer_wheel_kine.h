#ifndef _steer_wheel_kine_h_
#define _steer_wheel_kine_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define swheel steer_wheel_interface

#define STEER_WHEEL_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_MODEL, "Invalid Model") 

#define X(name, str) STEER_WHEEL_##name,
typedef enum {
    STEER_WHEEL_STATUS_TABLE
} SteelWheelErrorCode;
#undef X

typedef struct {
    float wheel_omega;
    float steer_angle;
} WheelModule;

typedef struct {
    float length;
    float width;
    float wheel_radius;
    float max_wheel_linear_speed;
} SteerWheelModel;

typedef struct {
    WheelModule wheels[4];
    float vx;
    float vy;
    float wz;
} SteerWheelControl;

typedef struct {
    WheelModule cur_wheels[4];
    float cur_vx;
    float cur_vy;
    float cur_wz;
} SteerWheelState;

typedef struct {
    SteerWheelModel model;
    SteerWheelControl control;
    SteerWheelState state;
} SteerWheel;

#define X(name, str) SteelWheelErrorCode name;
extern const struct SteerWheelInterface {
    struct {
        STEER_WHEEL_STATUS_TABLE
    };
    /**
     * @brief 初始化转向轮系统
     * @param steer_wheel 指向 SteerWheel 结构的指针
     * @param model 转向轮模型参数
     * @return 初始化状态
     */
    SteelWheelErrorCode(*init)(SteerWheel* steer_wheel, SteerWheelModel model);
    /**
     * @brief 正向运动学 (由轮速和转向角计算底盘速度)
     * @param steer_wheel 指向 SteerWheel 结构的指针
     * @return 计算状态
     */
    SteelWheelErrorCode(*fk)(SteerWheel* steer_wheel);
    /**
     * @brief 逆向运动学 (由底盘速度计算轮速和转向角)
     * @param steer_wheel 指向 SteerWheel 结构的指针
     * @return 计算状态
     */
    SteelWheelErrorCode(*ik)(SteerWheel* steer_wheel);
    /**
     * @brief 将错误码转换为字符串
     * @param status 错误码值
     * @return 错误信息字符串
     */
    const char* (*error_code_to_str)(SteelWheelErrorCode status);
} steer_wheel_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化转向轮系统
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @param model 转向轮模型参数
 * @return 初始化状态
 */
SteelWheelErrorCode steer_wheel_init(SteerWheel* steer_wheel, SteerWheelModel model);
/**
 * @brief 正向运动学 (由轮速和转向角计算底盘速度)
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @return 计算状态
 */
SteelWheelErrorCode steer_wheel_fk(SteerWheel* steer_wheel);
/**
 * @brief 逆向运动学 (由底盘速度计算轮速和转向角)
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @return 计算状态
 */
SteelWheelErrorCode steer_wheel_ik(SteerWheel* steer_wheel);
/**
 * @brief 将错误码转换为字符串
 * @param status 错误码值
 * @return 错误信息字符串
 */
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status);

#endif
