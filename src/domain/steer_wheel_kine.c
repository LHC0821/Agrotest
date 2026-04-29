#include "steer_wheel_kine.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define sw steer_wheel_interface

#define SW_PI 3.14159265358979323846f
#define SW_2PI (2.0f * SW_PI)
#define SW_HALF_PI (0.5f * SW_PI)
#define SW_EPS 1e-6f

#define X(name, str) .name = STEER_WHEEL_##name,
const struct SteerWheelInterface steer_wheel_interface = {
    {
        STEER_WHEEL_STATUS_TABLE
    },
    .init = steer_wheel_init,
    .fk = steer_wheel_fk,
    .ik = steer_wheel_ik,
    .error_code_to_str = steer_wheel_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 获取浮点数的绝对值
 * @param x 输入值
 * @return 绝对值
 */
static float sw_absf(float x);
/**
 * @brief 将角度规范化到 (-π, π] 范围
 * @param angle 输入角度 (弧度)
 * @return 规范化后的角度
 */
static float sw_wrap_pi(float angle);
/**
 * @brief 获取四个车轮在底盘坐标系中的位置坐标
 * @param model 转向轮模型参数
 * @param x 4 个车轮的 x 坐标数组 (输出)
 * @param y 4 个车轮的 y 坐标数组 (输出)
 * @return 无
 */
static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]);
/**
 * @brief 优化轮模块控制 (处理轮向反转)
 * @param target_angle 目标转向角指针 (输入/输出)
 * @param target_speed 目标线速度指针 (输入/输出)
 * @param current_angle 当前转向角
 * @return 无
 */
static void sw_optimize_module(float* target_angle, float* target_speed, float current_angle);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化转向轮系统
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @param model 转向轮模型参数 (长、宽、轮半径、最大轮速)
 * @return 初始化状态
 */
SteelWheelErrorCode steer_wheel_init(SteerWheel* steer_wheel, SteerWheelModel model) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(model.length <= 0.0f || model.width <= 0.0f || model.wheel_radius <= 0.0f || model.max_wheel_linear_speed < 0.0f) return sw.INVALID_MODEL;

    steer_wheel->model = model;

    for(uint8_t i = 0; i < 4; ++i) {
        steer_wheel->control.wheels[i].wheel_omega = 0.0f;
        steer_wheel->control.wheels[i].steer_angle = 0.0f;
        steer_wheel->state.cur_wheels[i].wheel_omega = 0.0f;
        steer_wheel->state.cur_wheels[i].steer_angle = 0.0f;
    }

    steer_wheel->control.vx = 0.0f;
    steer_wheel->control.vy = 0.0f;
    steer_wheel->control.wz = 0.0f;
    steer_wheel->state.cur_vx = 0.0f;
    steer_wheel->state.cur_vy = 0.0f;
    steer_wheel->state.cur_wz = 0.0f;

    return sw.OK;
}

/**
 * @brief 正向运动学计算 (由轮速和转向角计算底盘速度)
 * @details 根据当前各轮的角速度和转向角，使用最小二乘法计算底盘在全局坐标系中的速度
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @return 计算状态
 */
SteelWheelErrorCode steer_wheel_fk(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(steer_wheel->model.length <= 0.0f || steer_wheel->model.width <= 0.0f || steer_wheel->model.wheel_radius <= 0.0f) return sw.INVALID_MODEL;

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    float sum_vix = 0.0f;
    float sum_viy = 0.0f;
    float sum_w_numer = 0.0f;
    float sum_w_denom = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float wheel_linear_speed = steer_wheel->state.cur_wheels[i].wheel_omega * steer_wheel->model.wheel_radius;
        const float steer_angle = steer_wheel->state.cur_wheels[i].steer_angle;

        const float vix = wheel_linear_speed * cosf(steer_angle);
        const float viy = wheel_linear_speed * sinf(steer_angle);

        sum_vix += vix;
        sum_viy += viy;
        sum_w_numer += (-y[i] * vix + x[i] * viy);
        sum_w_denom += (x[i] * x[i] + y[i] * y[i]);
    }

    steer_wheel->state.cur_vx = sum_vix / (float)4;
    steer_wheel->state.cur_vy = sum_viy / (float)4;
    steer_wheel->state.cur_wz = (sum_w_denom > SW_EPS) ? (sum_w_numer / sum_w_denom) : 0.0f;

    return sw.OK;
}

/**
 * @brief 逆向运动学计算 (由底盘速度计算轮速和转向角)
 * @details 根据期望的底盘速度和角速度，计算每个轮的目标角速度和转向角，并进行速度限制和轮向优化
 * @param steer_wheel 指向 SteerWheel 结构的指针
 * @return 计算状态
 */
SteelWheelErrorCode steer_wheel_ik(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(steer_wheel->model.length <= 0.0f || steer_wheel->model.width <= 0.0f || steer_wheel->model.wheel_radius <= 0.0f) return sw.INVALID_MODEL;

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    const float vx = steer_wheel->control.vx;
    const float vy = steer_wheel->control.vy;
    const float wz = steer_wheel->control.wz;

    float max_abs_linear_speed = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float vix = vx - wz * y[i];
        const float viy = vy + wz * x[i];

        float target_linear_speed = sqrtf(vix * vix + viy * viy);
        float target_steer_angle;

        if(target_linear_speed < SW_EPS) {
            target_linear_speed = 0.0f;
            target_steer_angle = steer_wheel->state.cur_wheels[i].steer_angle;
        }
        else target_steer_angle = atan2f(viy, vix);

        sw_optimize_module(&target_steer_angle, &target_linear_speed, steer_wheel->state.cur_wheels[i].steer_angle);

        steer_wheel->control.wheels[i].wheel_omega = target_linear_speed / steer_wheel->model.wheel_radius;
        steer_wheel->control.wheels[i].steer_angle = target_steer_angle;

        if(sw_absf(target_linear_speed) > max_abs_linear_speed) max_abs_linear_speed = sw_absf(target_linear_speed);
    }

    if(steer_wheel->model.max_wheel_linear_speed > SW_EPS && max_abs_linear_speed > steer_wheel->model.max_wheel_linear_speed) {
        const float scale = steer_wheel->model.max_wheel_linear_speed / max_abs_linear_speed;

        for(uint8_t i = 0; i < 4; ++i) steer_wheel->control.wheels[i].wheel_omega *= scale;
    }

    return sw.OK;
}

#define X(name, str) case STEER_WHEEL_##name: return str;
/**
 * @brief 将错误码转换为字符串
 * @param status 错误码值
 * @return 错误信息字符串
 */
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status) {
    switch(status) {
        STEER_WHEEL_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 获取浮点数的绝对值
 * @param x 输入值
 * @return 绝对值
 */
static float sw_absf(float x) {
    return (x >= 0.0f) ? x : -x;
}

/**
 * @brief 将角度规范化到 (-π, π] 范围
 * @param angle 输入角度 (弧度)
 * @return 规范化后的角度
 */
static float sw_wrap_pi(float angle) {
    while(angle > SW_PI) {
        angle -= SW_2PI;
    }
    while(angle <= -SW_PI) {
        angle += SW_2PI;
    }
    return angle;
}

static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]) {
    const float hx = model->length * 0.5f;
    const float hy = model->width * 0.5f;

    /* 约定顺序: FL, FR, RL, RR */
    x[0] = hx;  y[0] = hy;
    x[1] = hx;  y[1] = -hy;
    x[2] = -hx;  y[2] = hy;
    x[3] = -hx;  y[3] = -hy;
}

static void sw_optimize_module(float* target_angle, float* target_speed, float current_angle) {
    float err = sw_wrap_pi(*target_angle - current_angle);

    if(sw_absf(err) > SW_HALF_PI) {
        *target_angle = sw_wrap_pi(*target_angle + SW_PI);
        *target_speed = -*target_speed;
    }
}
