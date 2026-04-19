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
    const char* (*status_str)(ChassisControllerStatus status);
    ChassisControllerStatus(*init)(float length, float width, float wheel_radius, float max_wheel_linear_speed);
    ChassisControllerStatus(*set_chassis)(float vx, float vy, float wz);
    ChassisControllerStatus(*set_wheels)(float wheel_omegas[4], float steer_angles[4]);
    ChassisControllerStatus(*brake)(void);
    ChassisControllerStatus(*stop)(void);
    ChassisControllerStatus(*get_chassis_state)(float* vx, float* vy, float* wz);
    ChassisControllerStatus(*get_wheels_state)(float wheel_omegas[4], float steer_angles[4]);
    ChassisControllerStatus(*get_model)(float* length, float* width, float* wheel_radius, float* max_wheel_linear_speed);
    ChassisControllerStatus(*update)(void);
}*chassis_controller_interface;
#undef SX

extern const struct ChassisControllerInterface agro_chassis_controller_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

const char* agro_chassis_controller_status_str(ChassisControllerStatus status);
ChassisControllerStatus agro_chassis_controller_init(float length, float width, float wheel_radius, float max_wheel_linear_speed);
ChassisControllerStatus agro_chassis_controller_set_chassis(float vx, float vy, float wz);
ChassisControllerStatus agro_chassis_controller_set_wheels(float wheel_omegas[4], float steer_angles[4]);
ChassisControllerStatus agro_chassis_controller_brake(void);
ChassisControllerStatus agro_chassis_controller_stop(void);
ChassisControllerStatus agro_chassis_controller_get_chassis_state(float* vx, float* vy, float* wz);
ChassisControllerStatus agro_chassis_controller_get_wheels_state(float wheel_omegas[4], float steer_angles[4]);
ChassisControllerStatus agro_chassis_controller_get_model(float* length, float* width, float* wheel_radius, float* max_wheel_linear_speed);
ChassisControllerStatus agro_chassis_controller_update(void);

#endif
