#include "AGV_Chassis.h"
#include "math.h"

// 底盘参数
ChassisParams chassis_params = {
    .wheel_radius = 0.05f,  // 5cm
    .max_speed = 1.0f       // 1m/s
};

// 底盘控制变量
ChassisSpeed chassis_speed = {0, 0, 0};
RemoteData rc_data = {0, 0, 0};

// 轮子位置
static const float wheel_pos[4][2] = {
    {-0.2f,  0.2f},  // 左前
    {-0.2f, -0.2f},  // 右前
    { 0.2f,  0.2f},  // 左后
    { 0.2f, -0.2f}   // 右后
};

// 电机参数
#define MAX_RPM 3000
#define MIN_RPM -3000
#define RS06_KP 5.0f
#define RS06_KD 0.5f

// 当前轮子角度记录
static float current_wheel_angles[4] = {0, 0, 0, 0};

// 角度归一化
static float normalize_angle(float angle)
{
    while(angle > 3.1415926f) angle -= 6.2831853f;
    while(angle < -3.1415926f) angle += 6.2831853f;
    return angle;
}

// 计算两个角度之间的最小差值
static float angle_difference(float target, float current)
{
    float diff = target - current;
    if(diff > 3.1415926f) diff -= 6.2831853f;
    if(diff < -3.1415926f) diff += 6.2831853f;
    return diff;
}

// 限幅
static int16_t limit_rpm(float rpm_float)
{
    int16_t rpm = (int16_t)rpm_float;
    if(rpm > MAX_RPM) return MAX_RPM;
    if(rpm < MIN_RPM) return MIN_RPM;
    return rpm;
}

/**
  * @brief  底盘初始化
  */
void Chassis_Init(void)
{
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;
    
    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;
    
    for(int i = 0; i < 4; i++)
    {
        current_wheel_angles[i] = 0;
    }
}

/**
  * @brief  遥控器数据输入
  * @param  ch1: 旋转控制 (-1.0 ~ 1.0)
  * @param  ch2: 前进后退 (-1.0 ~ 1.0)
  * @param  ch3: 左右平移 (-1.0 ~ 1.0)
  */
void Chassis_Set_Remote(float ch1, float ch2, float ch3)
{
    rc_data.ch1 = ch1;
    rc_data.ch2 = ch2;
    rc_data.ch3 = ch3;
}

/**
  * @brief  控制单个轮子
  */
static void control_wheel(uint8_t id)
{
    float x = wheel_pos[id][0];
    float y = wheel_pos[id][1];
    
    // 计算轮子速度
    float vx_i = chassis_speed.vx - chassis_speed.vw * y;
    float vy_i = chassis_speed.vy + chassis_speed.vw * x;
    
    // 计算轮子线速度大小
    float wheel_speed = sqrt(vx_i * vx_i + vy_i * vy_i);
    
    // 计算转向角度
    float target_angle = 0;
    if(fabs(vx_i) > 0.001f || fabs(vy_i) > 0.001f)
    {
        target_angle = atan2(vy_i, vx_i);
    }
    else
    {
        // 速度为0时保持当前角度
        target_angle = current_wheel_angles[id];
    }
    
    // 归一化角度
    target_angle = normalize_angle(target_angle);
    
    // 计算角度差值，避免轮子180度反转
    float angle_diff = angle_difference(target_angle, current_wheel_angles[id]);
    if(fabs(angle_diff) > 1.570796f)  // 如果差值大于90度
    {
        // 反转驱动方向
        wheel_speed = -wheel_speed;
        // 调整目标角度
        target_angle += 3.1415926f;  // 加π
        target_angle = normalize_angle(target_angle);
    }
    
    // 更新当前角度
    current_wheel_angles[id] = target_angle;
    
    // 计算轮子角速度
    float wheel_angular_speed = 0;
    if(chassis_params.wheel_radius > 0.001f)
    {
        wheel_angular_speed = wheel_speed / chassis_params.wheel_radius;
    }
    
    // 将角速度转换为RPM
    float rpm_float = wheel_angular_speed * 9.5493f;
    
    // 限幅
    int16_t rpm = limit_rpm(rpm_float);
    
    // 控制电机
    RS06_Set_Position(&hfdcan1, id+1, target_angle, 2.0f, RS06_KP, RS06_KD, 0);
    Motor_Speed_Control_Smooth(rpm, id+1);
}

/**
  * @brief  底盘更新
  */
void Chassis_Update(void)
{
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    
    // 控制周期: 2ms
    if(now - last_time < 2) return;
    last_time = now;
    
    // 从遥控器计算底盘速度
    chassis_speed.vx = rc_data.ch2 * chassis_params.max_speed;
    chassis_speed.vy = rc_data.ch3 * chassis_params.max_speed;
    chassis_speed.vw = rc_data.ch1 * 2.0f;
    
    // 控制四个轮子
    for(int i = 0; i < 4; i++)
    {
        control_wheel(i);
    }
}

/**
  * @brief  停止底盘
  */
void Chassis_Stop(void)
{
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;
    
    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;
    
    for(int i = 0; i < 4; i++)
    {
        Motor_Speed_Control_Smooth(0, i+1);
    }
}
