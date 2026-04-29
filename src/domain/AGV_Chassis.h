#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include <stdint.h>

/**
 * @brief 底盘速度结构体
 */
typedef struct {
    float vx;    /**< 纵向速度 (m/s) */
    float vy;    /**< 横向速度 (m/s) */
    float vw;    /**< 角速度 (rad/s) */
} ChassisSpeed;

/**
 * @brief 遥控器数据结构体
 */
typedef struct {
    float ch1;   /**< 通道 1 */
    float ch2;   /**< 通道 2 */
    float ch3;   /**< 通道 3 */
} RemoteData;

/**
 * @brief 底盘参数结构体
 */
typedef struct {
    float wheel_radius;  /**< 车轮半径 (m) */
    float max_speed;     /**< 最大速度 (m/s) */
} ChassisParams;

extern ChassisSpeed chassis_speed;
extern ChassisParams chassis_params;
extern RemoteData rc_data;

/**
 * @brief 初始化底盘系统
 * @return 无
 */
void Chassis_Init(void);
/**
 * @brief 设置底盘远程控制数据
 * @param ch1 通道 1 数据
 * @param ch2 通道 2 数据
 * @param ch3 通道 3 数据
 * @return 无
 */
void Chassis_Set_Remote(float ch1, float ch2, float ch3);
/**
 * @brief 平滑设置指定车轮转速
 * @param wheel_id 车轮 ID
 * @param rpm 目标转速 (转/分)
 * @return 无
 */
void Chassis_Set_WheelSpeedSmooth(uint8_t wheel_id, int16_t rpm);
/**
 * @brief 更新底盘状态 (需要定期调用)
 * @return 无
 */
void Chassis_Update(void);
/**
 * @brief 停止底盘运动
 * @return 无
 */
void Chassis_Stop(void);

#endif
