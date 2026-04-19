#include "AGV_Chassis.h"
#include "math.h"
#include "motor.h"
#include "servo.h"

// ���̲���
ChassisParams chassis_params = {
    .wheel_radius = 0.05f,  // 5cm
    .max_speed = 1.0f       // 1m/s
};

// ���̿��Ʊ���
ChassisSpeed chassis_speed = { 0, 0, 0 };
RemoteData rc_data = { 0, 0, 0 };

// ����λ��
static const float wheel_pos[4][2] = {
    {-0.2f,  0.2f},  // ��ǰ
    {-0.2f, -0.2f},  // ��ǰ
    { 0.2f,  0.2f},  // ���
    { 0.2f, -0.2f}   // �Һ�
};

// �������
#define MAX_RPM 3000
#define MIN_RPM -3000
#define RS06_KP 5.0f
#define RS06_KD 0.5f

// ��ǰ���ӽǶȼ�¼
static float current_wheel_angles[4] = { 0, 0, 0, 0 };

// �Ƕȹ�һ��
static float normalize_angle(float angle) {
    while(angle > 3.1415926f) angle -= 6.2831853f;
    while(angle < -3.1415926f) angle += 6.2831853f;
    return angle;
}

// ���������Ƕ�֮�����С��ֵ
static float angle_difference(float target, float current) {
    float diff = target - current;
    if(diff > 3.1415926f) diff -= 6.2831853f;
    if(diff < -3.1415926f) diff += 6.2831853f;
    return diff;
}

// �޷�
static int16_t limit_rpm(float rpm_float) {
    int16_t rpm = (int16_t)rpm_float;
    if(rpm > MAX_RPM) return MAX_RPM;
    if(rpm < MIN_RPM) return MIN_RPM;
    return rpm;
}

/**
  * @brief  ���̳�ʼ��
  */
void Chassis_Init(void) {
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;

    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;

    for(int i = 0; i < 4; i++) {
        current_wheel_angles[i] = 0;
    }
}

/**
  * @brief  ң������������
  * @param  ch1: ��ת���� (-1.0 ~ 1.0)
  * @param  ch2: ǰ������ (-1.0 ~ 1.0)
  * @param  ch3: ����ƽ�� (-1.0 ~ 1.0)
  */
void Chassis_Set_Remote(float ch1, float ch2, float ch3) {
    rc_data.ch1 = ch1;
    rc_data.ch2 = ch2;
    rc_data.ch3 = ch3;
}

/**
  * @brief  ���Ƶ�������
  */
static void control_wheel(uint8_t id) {
    float x = wheel_pos[id][0];
    float y = wheel_pos[id][1];

    // ���������ٶ�
    float vx_i = chassis_speed.vx - chassis_speed.vw * y;
    float vy_i = chassis_speed.vy + chassis_speed.vw * x;

    // �����������ٶȴ�С
    float wheel_speed = sqrt(vx_i * vx_i + vy_i * vy_i);

    // ����ת��Ƕ�
    float target_angle = 0;
    if(fabs(vx_i) > 0.001f || fabs(vy_i) > 0.001f) {
        target_angle = atan2(vy_i, vx_i);
    }
    else {
        // �ٶ�Ϊ0ʱ���ֵ�ǰ�Ƕ�
        target_angle = current_wheel_angles[id];
    }

    // ��һ���Ƕ�
    target_angle = normalize_angle(target_angle);

    // ����ǶȲ�ֵ����������180�ȷ�ת
    float angle_diff = angle_difference(target_angle, current_wheel_angles[id]);
    if(fabs(angle_diff) > 1.570796f)  // �����ֵ����90��
    {
        // ��ת��������
        wheel_speed = -wheel_speed;
        // ����Ŀ��Ƕ�
        target_angle += 3.1415926f;  // �Ӧ�
        target_angle = normalize_angle(target_angle);
    }

    // ���µ�ǰ�Ƕ�
    current_wheel_angles[id] = target_angle;

    // �������ӽ��ٶ�
    float wheel_angular_speed = 0;
    if(chassis_params.wheel_radius > 0.001f) {
        wheel_angular_speed = wheel_speed / chassis_params.wheel_radius;
    }

    // �����ٶ�ת��ΪRPM
    float rpm_float = wheel_angular_speed * 9.5493f;

    // �޷�
    int16_t rpm = limit_rpm(rpm_float);

    // ���Ƶ��
    servo.set_position(&hfdcan1, id + 1, target_angle, 2.0f, RS06_KP, RS06_KD, 0);
    motor.speed_control_smooth(rpm, id + 1);
}

/**
  * @brief  ���̸���
  */
void Chassis_Update(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    // ��������: 2ms
    if(now - last_time < 2) return;
    last_time = now;

    // ��ң������������ٶ�
    chassis_speed.vx = rc_data.ch2 * chassis_params.max_speed;
    chassis_speed.vy = rc_data.ch3 * chassis_params.max_speed;
    chassis_speed.vw = rc_data.ch1 * 2.0f;

    // �����ĸ�����
    for(int i = 0; i < 4; i++) {
        control_wheel(i);
    }
}

/**
  * @brief  ֹͣ����
  */
void Chassis_Stop(void) {
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;

    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;

    for(int i = 0; i < 4; i++) {
        motor.speed_control_smooth(0, i + 1);
    }
}
