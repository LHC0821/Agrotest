#ifndef _chassis_controller_h_
#define _chassis_controller_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define chassis (*chassis_controller_interface)

#define CHASSIS_CONTROLLER_STATUS_TABLE \
    SX(OK, "OK") \
    SX(INVALID_PARAM, "Invalid Parameter") \
    SX(INVALID_MODEL, "Invalid Model") \
    SX(BRAKE_FAILED, "Brake Failed") \
    SX(STOP_FAILED, "Stop Failed")

#define SX(name, str) CHASSIS_CONTROLLER_##name,
typedef enum {
    CHASSIS_CONTROLLER_STATUS_TABLE
} ChassisControllerStatus;
#undef SX

#define SX(name, str) const ChassisControllerStatus name;
extern const struct ChassisControllerInterface {
    struct {
        CHASSIS_CONTROLLER_STATUS_TABLE
    };
    /**
     * @brief 将底盘控制器状态转换为字符串
     * @param status 底盘控制器状态值
     * @return 状态对应的字符串
     */
    const char* (*status_str)(ChassisControllerStatus status);
    /**
     * @brief 初始化底盘控制器
     * @param length 底盘长度 (m)
     * @param width 底盘宽度 (m)
     * @param wheel_radius 车轮半径 (m)
     * @param max_wheel_linear_speed 车轮最大线速度 (m/s)
     * @return 初始化状态
     */
    ChassisControllerStatus(*init)(float length, float width, float wheel_radius, float max_wheel_linear_speed);
    /**
     * @brief 设置底盘运动速度 (底盘坐标系)
     * @param vx 纵向速度 (m/s)
     * @param vy 横向速度 (m/s)
     * @param wz 角速度 (rad/s)
     * @return 设置状态
     */
    ChassisControllerStatus(*set_chassis)(float vx, float vy, float wz);
    /**
     * @brief 直接设置各轮的角速度和转向角
     * @param wheel_omegas 4 个车轮的角速度数组 (rad/s)
     * @param steer_angles 4 个车轮的转向角数组 (rad)
     * @return 设置状态
     */
    ChassisControllerStatus(*set_wheels)(float wheel_omegas[4], float steer_angles[4]);
    /**
     * @brief 刹车 (快速停止)
     * @return 操作状态
     */
    ChassisControllerStatus(*brake)(void);
    /**
     * @brief 停止 (平稳停止)
     * @return 操作状态
     */
    ChassisControllerStatus(*stop)(void);
    /**
     * @brief 获取底盘当前运动状态
     * @param vx 指向纵向速度的指针
     * @param vy 指向横向速度的指针
     * @param wz 指向角速度的指针
     * @return 获取状态
     */
    ChassisControllerStatus(*get_chassis_state)(float* vx, float* vy, float* wz);
    /**
     * @brief 获取各轮的角速度和转向角状态
     * @param wheel_omegas 4 个车轮的角速度数组 (rad/s)
     * @param steer_angles 4 个车轮的转向角数组 (rad)
     * @return 获取状态
     */
    ChassisControllerStatus(*get_wheels_state)(float wheel_omegas[4], float steer_angles[4]);
    /**
     * @brief 获取底盘模型参数
     * @param length 指向底盘长度的指针 (m)
     * @param width 指向底盘宽度的指针 (m)
     * @param wheel_radius 指向车轮半径的指针 (m)
     * @param max_wheel_linear_speed 指向车轮最大线速度的指针 (m/s)
     * @return 获取状态
     */
    ChassisControllerStatus(*get_model)(float* length, float* width, float* wheel_radius, float* max_wheel_linear_speed);
    /**
     * @brief 更新底盘控制器状态 (需要定期调用)
     * @return 更新状态
     */
    ChassisControllerStatus(*update)(void);
}*chassis_controller_interface;
#undef SX

extern const struct ChassisControllerInterface agro_chassis_controller_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 将底盘控制器状态转换为字符串
 * @param status 底盘控制器状态值
 * @return 状态对应的字符串
 */
const char* agro_chassis_controller_status_str(ChassisControllerStatus status);
/**
 * @brief 初始化底盘控制器
 * @param length 底盘长度 (m)
 * @param width 底盘宽度 (m)
 * @param wheel_radius 车轮半径 (m)
 * @param max_wheel_linear_speed 车轮最大线速度 (m/s)
 * @return 初始化状态
 */
ChassisControllerStatus agro_chassis_controller_init(float length, float width, float wheel_radius, float max_wheel_linear_speed);
/**
 * @brief 设置底盘运动速度 (底盘坐标系)
 * @param vx 纵向速度 (m/s)
 * @param vy 横向速度 (m/s)
 * @param wz 角速度 (rad/s)
 * @return 设置状态
 */
ChassisControllerStatus agro_chassis_controller_set_chassis(float vx, float vy, float wz);
/**
 * @brief 直接设置各轮的角速度和转向角
 * @param wheel_omegas 4 个车轮的角速度数组 (rad/s)
 * @param steer_angles 4 个车轮的转向角数组 (rad)
 * @return 设置状态
 */
ChassisControllerStatus agro_chassis_controller_set_wheels(float wheel_omegas[4], float steer_angles[4]);
/**
 * @brief 刹车 (快速停止)
 * @return 操作状态
 */
ChassisControllerStatus agro_chassis_controller_brake(void);
/**
 * @brief 停止 (平稳停止)
 * @return 操作状态
 */
ChassisControllerStatus agro_chassis_controller_stop(void);
/**
 * @brief 获取底盘当前运动状态
 * @param vx 指向纵向速度的指针 (m/s)
 * @param vy 指向横向速度的指针 (m/s)
 * @param wz 指向角速度的指针 (rad/s)
 * @return 获取状态
 */
ChassisControllerStatus agro_chassis_controller_get_chassis_state(float* vx, float* vy, float* wz);
/**
 * @brief 获取各轮的角速度和转向角状态
 * @param wheel_omegas 4 个车轮的角速度数组 (rad/s)
 * @param steer_angles 4 个车轮的转向角数组 (rad)
 * @return 获取状态
 */
ChassisControllerStatus agro_chassis_controller_get_wheels_state(float wheel_omegas[4], float steer_angles[4]);
/**
 * @brief 获取底盘模型参数
 * @param length 指向底盘长度的指针 (m)
 * @param width 指向底盘宽度的指针 (m)
 * @param wheel_radius 指向车轮半径的指针 (m)
 * @param max_wheel_linear_speed 指向车轮最大线速度的指针 (m/s)
 * @return 获取状态
 */
ChassisControllerStatus agro_chassis_controller_get_model(float* length, float* width, float* wheel_radius, float* max_wheel_linear_speed);
/**
 * @brief 更新底盘控制器状态 (需要定期调用)
 * @return 更新状态
 */
ChassisControllerStatus agro_chassis_controller_update(void);

#endif
