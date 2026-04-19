#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include <stdint.h>

typedef struct {
    float vx;
    float vy;
    float vw;
} ChassisSpeed;

typedef struct {
    float ch1;
    float ch2;
    float ch3;
} RemoteData;

typedef struct {
    float wheel_radius;
    float max_speed;
} ChassisParams;

extern ChassisSpeed chassis_speed;
extern ChassisParams chassis_params;
extern RemoteData rc_data;

void Chassis_Init(void);
void Chassis_Set_Remote(float ch1, float ch2, float ch3);
void Chassis_Set_WheelSpeedSmooth(uint8_t wheel_id, int16_t rpm);
void Chassis_Update(void);
void Chassis_Stop(void);

#endif
