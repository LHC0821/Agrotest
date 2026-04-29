#include "chassis_controller.h"

#include "steer_wheel_kine.h"
#include "motor.h"
#include "servo.h"

#include <stdbool.h>
#include <math.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define SX(name, str) .name = CHASSIS_CONTROLLER_##name,
const struct ChassisControllerInterface agro_chassis_controller_instance = {
    {
        CHASSIS_CONTROLLER_STATUS_TABLE
    },
    .status_str = agro_chassis_controller_status_str,
    .init = agro_chassis_controller_init,
    .set_chassis = agro_chassis_controller_set_chassis,
    .set_wheels = agro_chassis_controller_set_wheels,
    .brake = agro_chassis_controller_brake,
    .stop = agro_chassis_controller_stop,
    .get_chassis_state = agro_chassis_controller_get_chassis_state,
    .get_wheels_state = agro_chassis_controller_get_wheels_state,
    .get_model = agro_chassis_controller_get_model,
    .update = agro_chassis_controller_update
};
#undef SX

const struct ChassisControllerInterface* chassis_controller_interface = &agro_chassis_controller_instance;
#ifndef chassis
#define chassis (*chassis_controller_instance)
#endif

static SteerWheel steer_wheel = { 0 };
static SteerWheel last_steer_wheel = { 0 };
static SteerWheelModel steer_wheel_model = { 0 };
static uint8_t g_feedback_query_id = 1U;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static ChassisControllerStatus convert_steer_wheel_error(SteelWheelErrorCode error_code);
static bool is_chassis_changed(const SteerWheel* now, const SteerWheel* last);
static bool is_wheels_changed(const SteerWheel* now, const SteerWheel* last);
static void update_feedback_cache(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define SX(name, str) case CHASSIS_CONTROLLER_##name: return str;
const char* agro_chassis_controller_status_str(ChassisControllerStatus status) {
    switch(status) {
        CHASSIS_CONTROLLER_STATUS_TABLE
        default: return "Unknown Status";
    }
}

ChassisControllerStatus agro_chassis_controller_init(float length, float width, float wheel_radius, float max_wheel_linear_speed) {
    steer_wheel_model.length = length;
    steer_wheel_model.width = width;
    steer_wheel_model.wheel_radius = wheel_radius;
    steer_wheel_model.max_wheel_linear_speed = max_wheel_linear_speed;
    g_feedback_query_id = 1U;

    return convert_steer_wheel_error(steer_wheel_interface.init(&steer_wheel, steer_wheel_model));
}

ChassisControllerStatus agro_chassis_controller_set_chassis(float vx, float vy, float wz) {
    steer_wheel.control.vx = vx;
    steer_wheel.control.vy = vy;
    steer_wheel.control.wz = wz;

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_set_wheels(float wheel_omegas[4], float steer_angles[4]) {
    if(wheel_omegas == NULL || steer_angles == NULL) return chassis.INVALID_PARAM;

    for(int i = 0; i < 4; i++) {
        steer_wheel.control.wheels[i].wheel_omega = wheel_omegas[i];
        steer_wheel.control.wheels[i].steer_angle = steer_angles[i];
    }

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_brake(void) {
    // TODO: 实现刹车逻辑
    return CHASSIS_CONTROLLER_BRAKE_FAILED;
}

ChassisControllerStatus agro_chassis_controller_stop(void) {
    float cur_steer_angles[4];

    steer_wheel.control.vx = 0.0f;
    steer_wheel.control.vy = 0.0f;
    steer_wheel.control.wz = 0.0f;

    for(int i = 0; i < 4; i++) cur_steer_angles[i] = steer_wheel.state.cur_wheels[i].steer_angle;
    chassis.set_wheels((float[4]) { 0, 0, 0, 0 }, cur_steer_angles);

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_get_chassis_state(float* vx, float* vy, float* wz) {
    if(vx == NULL || vy == NULL || wz == NULL) return chassis.INVALID_PARAM;

    *vx = steer_wheel.state.cur_vx;
    *vy = steer_wheel.state.cur_vy;
    *wz = steer_wheel.state.cur_wz;

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_get_wheels_state(float wheel_omegas[4], float steer_angles[4]) {
    if(wheel_omegas == NULL || steer_angles == NULL) return chassis.INVALID_PARAM;

    for(int i = 0; i < 4; i++) {
        wheel_omegas[i] = steer_wheel.state.cur_wheels[i].wheel_omega;
        steer_angles[i] = steer_wheel.state.cur_wheels[i].steer_angle;
    }

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_get_model(float* length, float* width, float* wheel_radius, float* max_wheel_linear_speed) {
    if(length == NULL || width == NULL || wheel_radius == NULL || max_wheel_linear_speed == NULL) return chassis.INVALID_PARAM;

    *length = steer_wheel.model.length;
    *width = steer_wheel.model.width;
    *wheel_radius = steer_wheel.model.wheel_radius;
    *max_wheel_linear_speed = steer_wheel.model.max_wheel_linear_speed;

    return chassis.OK;
}

ChassisControllerStatus agro_chassis_controller_update(void) {
    SteelWheelErrorCode error_code;
    bool should_send_wheels;

    update_feedback_cache();

    for(uint8_t i = 0U; i < 4U; i++) {
        uint8_t id = (uint8_t)(i + 1U);
        steer_wheel.state.cur_wheels[i].steer_angle = servo.get_pos(id);
        steer_wheel.state.cur_wheels[i].wheel_omega = motor.get_spd_rads(id);
    }

    error_code = swheel.fk(&steer_wheel);
    if(error_code != STEER_WHEEL_OK) return convert_steer_wheel_error(error_code);

    if(is_chassis_changed(&steer_wheel, &last_steer_wheel)) {
        error_code = swheel.ik(&steer_wheel);
        if(error_code != STEER_WHEEL_OK) return convert_steer_wheel_error(error_code);
    }

    should_send_wheels = is_wheels_changed(&steer_wheel, &last_steer_wheel);
    if(should_send_wheels) {
        for(uint8_t i = 0U; i < 4U; i++) {
            uint8_t id = (uint8_t)(i + 1U);
            servo.set_position(id, steer_wheel.control.wheels[i].steer_angle);
            motor.set_speed_rads(id, steer_wheel.control.wheels[i].wheel_omega);
        }
    }

    last_steer_wheel = steer_wheel;

    return chassis.OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static ChassisControllerStatus convert_steer_wheel_error(SteelWheelErrorCode error_code) {
    switch(error_code) {
        case STEER_WHEEL_OK: return CHASSIS_CONTROLLER_OK;
        case STEER_WHEEL_INVALID_PARAM: return CHASSIS_CONTROLLER_INVALID_PARAM;
        case STEER_WHEEL_INVALID_MODEL: return CHASSIS_CONTROLLER_INVALID_MODEL;
        default: return CHASSIS_CONTROLLER_INVALID_PARAM;
    }
}

static bool is_chassis_changed(const SteerWheel* now, const SteerWheel* last) {
    if(fabsf(now->control.vx - last->control.vx) > 1e-3f) return true;
    if(fabsf(now->control.vy - last->control.vy) > 1e-3f) return true;
    if(fabsf(now->control.wz - last->control.wz) > 1e-3f) return true;

    return false;
}

static bool is_wheels_changed(const SteerWheel* now, const SteerWheel* last) {
    for(int i = 0; i < 4; i++) {
        if(fabsf(now->control.wheels[i].wheel_omega - last->control.wheels[i].wheel_omega) > 1e-3f) return true;
        if(fabsf(now->control.wheels[i].steer_angle - last->control.wheels[i].steer_angle) > 1e-3f) return true;
    }

    return false;
}

static void update_feedback_cache(void) {
    MotorFeedback feedback;

    (void)motor.get_feedback(&feedback);

    if(motor.request_report(g_feedback_query_id,
        MOTOR_FEEDBACK_CMD_SPEED,
        MOTOR_FEEDBACK_CMD_POSITION,
        MOTOR_FEEDBACK_CMD_ERROR) == motor.OK) {
        g_feedback_query_id++;
        if(g_feedback_query_id > 4U) {
            g_feedback_query_id = 1U;
        }
    }
}

#undef chassis
