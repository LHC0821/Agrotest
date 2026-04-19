#include "AGV_Chassis.h"

#include "motor.h"
#include "servo.h"
#include "tim.h"

#include <math.h>
#include <string.h>

#define CHASSIS_WHEEL_COUNT 4
#define CHASSIS_UPDATE_PERIOD_MS 2U
#define CHASSIS_WHEEL_RPM_LIMIT 210

#define MAX_RPM 3000
#define MIN_RPM -3000
#define RS06_KP 5.0f
#define RS06_KD 0.5f

typedef struct {
    int16_t target_rpm;
    float current_rpm;
    float accel_step;
} ChassisMotorSmoothCtrl;

ChassisParams chassis_params = {
    .wheel_radius = 0.05f,
    .max_speed = 1.0f
};

ChassisSpeed chassis_speed = { 0, 0, 0 };
RemoteData rc_data = { 0, 0, 0 };

static const float wheel_pos[CHASSIS_WHEEL_COUNT][2] = {
    { -0.2f, 0.2f },
    { -0.2f, -0.2f },
    { 0.2f, 0.2f },
    { 0.2f, -0.2f }
};

static float current_wheel_angles[CHASSIS_WHEEL_COUNT] = { 0, 0, 0, 0 };
static ChassisMotorSmoothCtrl g_chassis_motor_states[CHASSIS_WHEEL_COUNT];
static MotorReport g_chassis_report_cache[CHASSIS_WHEEL_COUNT];
static uint8_t g_chassis_query_id = 1;

static float normalize_angle(float angle);
static float angle_difference(float target, float current);
static int16_t limit_rpm(float rpm_float);
static int16_t chassis_clamp_motor_rpm(int16_t rpm);
static void control_wheel(uint8_t id);
static void chassis_tick_smooth_output(void);
static void chassis_monitor_read(void);

static float normalize_angle(float angle) {
    while(angle > 3.1415926f) {
        angle -= 6.2831853f;
    }
    while(angle < -3.1415926f) {
        angle += 6.2831853f;
    }
    return angle;
}

static float angle_difference(float target, float current) {
    float diff = target - current;
    if(diff > 3.1415926f) {
        diff -= 6.2831853f;
    }
    if(diff < -3.1415926f) {
        diff += 6.2831853f;
    }
    return diff;
}

static int16_t limit_rpm(float rpm_float) {
    int16_t rpm = (int16_t)rpm_float;
    if(rpm > MAX_RPM) {
        return MAX_RPM;
    }
    if(rpm < MIN_RPM) {
        return MIN_RPM;
    }
    return rpm;
}

static int16_t chassis_clamp_motor_rpm(int16_t rpm) {
    if(rpm > CHASSIS_WHEEL_RPM_LIMIT) {
        return CHASSIS_WHEEL_RPM_LIMIT;
    }
    if(rpm < -CHASSIS_WHEEL_RPM_LIMIT) {
        return -CHASSIS_WHEEL_RPM_LIMIT;
    }
    return rpm;
}

void Chassis_Init(void) {
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;

    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;

    for(uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
        current_wheel_angles[i] = 0;
        g_chassis_motor_states[i].target_rpm = 0;
        g_chassis_motor_states[i].current_rpm = 0.0f;
        g_chassis_motor_states[i].accel_step = 0.4f;
        memset(&g_chassis_report_cache[i], 0, sizeof(g_chassis_report_cache[i]));
    }

    g_chassis_query_id = 1;

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim15);
}

void Chassis_Set_Remote(float ch1, float ch2, float ch3) {
    rc_data.ch1 = ch1;
    rc_data.ch2 = ch2;
    rc_data.ch3 = ch3;
}

void Chassis_Set_WheelSpeedSmooth(uint8_t wheel_id, int16_t rpm) {
    if(wheel_id < 1 || wheel_id > CHASSIS_WHEEL_COUNT) {
        return;
    }

    g_chassis_motor_states[wheel_id - 1].target_rpm = chassis_clamp_motor_rpm(rpm);
}

void Chassis_Set_AllWheelSpeedSmooth(const int16_t rpms[CHASSIS_WHEEL_COUNT]) {
    if(rpms == NULL) {
        return;
    }

    for(uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
        Chassis_Set_WheelSpeedSmooth((uint8_t)(i + 1), rpms[i]);
    }
}

static void control_wheel(uint8_t id) {
    float x = wheel_pos[id][0];
    float y = wheel_pos[id][1];

    float vx_i = chassis_speed.vx - chassis_speed.vw * y;
    float vy_i = chassis_speed.vy + chassis_speed.vw * x;

    float wheel_speed = sqrtf(vx_i * vx_i + vy_i * vy_i);

    float target_angle = 0;
    if(fabsf(vx_i) > 0.001f || fabsf(vy_i) > 0.001f) {
        target_angle = atan2f(vy_i, vx_i);
    }
    else {
        target_angle = current_wheel_angles[id];
    }

    target_angle = normalize_angle(target_angle);

    float angle_diff = angle_difference(target_angle, current_wheel_angles[id]);
    if(fabsf(angle_diff) > 1.570796f) {
        wheel_speed = -wheel_speed;
        target_angle += 3.1415926f;
        target_angle = normalize_angle(target_angle);
    }

    current_wheel_angles[id] = target_angle;

    float wheel_angular_speed = 0;
    if(chassis_params.wheel_radius > 0.001f) {
        wheel_angular_speed = wheel_speed / chassis_params.wheel_radius;
    }

    float rpm_float = wheel_angular_speed * 9.5493f;
    int16_t rpm = limit_rpm(rpm_float);

    servo.set_mit((uint8_t)(id + 1), target_angle, 2.0f, RS06_KP, RS06_KD, 0);
    Chassis_Set_WheelSpeedSmooth((uint8_t)(id + 1), rpm);
}

void Chassis_Update(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    if(now - last_time < CHASSIS_UPDATE_PERIOD_MS) {
        return;
    }
    last_time = now;

    chassis_speed.vx = rc_data.ch2 * chassis_params.max_speed;
    chassis_speed.vy = rc_data.ch3 * chassis_params.max_speed;
    chassis_speed.vw = rc_data.ch1 * 2.0f;

    for(uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
        control_wheel(i);
    }
}

void Chassis_Stop(void) {
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;

    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;

    static const int16_t stop_rpms[CHASSIS_WHEEL_COUNT] = { 0, 0, 0, 0 };
    Chassis_Set_AllWheelSpeedSmooth(stop_rpms);
}

static void chassis_tick_smooth_output(void) {
    for(uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
        float diff = (float)g_chassis_motor_states[i].target_rpm - g_chassis_motor_states[i].current_rpm;

        if(diff > g_chassis_motor_states[i].accel_step) {
            g_chassis_motor_states[i].current_rpm += g_chassis_motor_states[i].accel_step;
        }
        else if(diff < -g_chassis_motor_states[i].accel_step) {
            g_chassis_motor_states[i].current_rpm -= g_chassis_motor_states[i].accel_step;
        }
        else {
            g_chassis_motor_states[i].current_rpm = (float)g_chassis_motor_states[i].target_rpm;
        }

        int16_t send_rpm = (int16_t)g_chassis_motor_states[i].current_rpm;
        (void)motor.set_speed((uint8_t)(i + 1), send_rpm);
    }
}

static void chassis_monitor_read(void) {
    (void)motor.request_report(g_chassis_query_id,
        MOTOR_FEEDBACK_CMD_SPEED,
        MOTOR_FEEDBACK_CMD_POSITION,
        MOTOR_FEEDBACK_CMD_ERROR);

    if(++g_chassis_query_id > CHASSIS_WHEEL_COUNT) {
        g_chassis_query_id = 1;
    }

    MotorFeedback fb;
    if(motor.update(&fb) != motor.OK) {
        return;
    }

    if(fb.id >= 1 && fb.id <= CHASSIS_WHEEL_COUNT) {
        MotorReport report;
        if(motor.latest_report(fb.id, &report) == motor.OK) {
            g_chassis_report_cache[fb.id - 1] = report;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if(htim->Instance == TIM6) {
        chassis_tick_smooth_output();
        return;
    }

    if(htim->Instance == TIM15) {
        chassis_monitor_read();
    }
}
