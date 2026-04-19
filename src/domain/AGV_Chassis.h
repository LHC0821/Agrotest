#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include "main.h"

// 外部电机控制函数声明
void RS06_Set_Position(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, 
                       float angle_rad, float speed_rad_s, 
                       float kp, float kd, float t_ff);
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID);

// 底盘速度结构体
typedef struct {
    float vx;    // 前进/后退速度
    float vy;    // 左/右平移速度
    float vw;    // 旋转速度
} ChassisSpeed;

// 遥控器结构
typedef struct {
    float ch1;  // 旋转控制
    float ch2;  // 前进后退
    float ch3;  // 左右平移
} RemoteData;

// 底盘参数
typedef struct {
    float wheel_radius;  // 轮子半径
    float max_speed;     // 最大速度
} ChassisParams;

// 外部变量
extern ChassisSpeed chassis_speed;
extern ChassisParams chassis_params;
extern RemoteData rc_data;
extern FDCAN_HandleTypeDef hfdcan1;

// 函数声明
void Chassis_Init(void);
void Chassis_Update(void);
void Chassis_Stop(void);

#endif
