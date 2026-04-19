#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include "main.h"

// �����ٶȽṹ��
typedef struct {
    float vx;    // ǰ��/�����ٶ�
    float vy;    // ��/��ƽ���ٶ�
    float vw;    // ��ת�ٶ�
} ChassisSpeed;

// ң�����ṹ
typedef struct {
    float ch1;  // ��ת����
    float ch2;  // ǰ������
    float ch3;  // ����ƽ��
} RemoteData;

// ���̲���
typedef struct {
    float wheel_radius;  // ���Ӱ뾶
    float max_speed;     // ����ٶ�
} ChassisParams;

// �ⲿ����
extern ChassisSpeed chassis_speed;
extern ChassisParams chassis_params;
extern RemoteData rc_data;
extern FDCAN_HandleTypeDef hfdcan1;

// ��������
void Chassis_Init(void);
void Chassis_Update(void);
void Chassis_Stop(void);

#endif
