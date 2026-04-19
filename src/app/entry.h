#ifndef _entry_h_
#define _entry_h_

// ! 系 统 文 件 ! //
#include "main.h"   // IWYU pragma: keep
#include "dma.h"    // IWYU pragma: keep
#include "fdcan.h"  // IWYU pragma: keep
#include "tim.h"    // IWYU pragma: keep
#include "usart.h"  // IWYU pragma: keep
#include "gpio.h"   // IWYU pragma: keep

// #include <stdio.h>
#include <string.h>

// ! 平 台 层 ! //
#include "key.h"
// #include "can.h"

// ! 基 础 设 施 层 ! //
#include "delay.h"

// ! 领 域 层 ! //


// ! 设 备 层 ! //
#include "motor.h"
#include "servo.h"

// ! 服 务 层 ! //


// ! 应 用 层 ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static uint8_t current = 0;
static uint8_t last_key_state = 0; // 用于存储上一次按键状态，实现边沿检测

extern reporter Motor_Reporter_Data;
extern uint8_t query_id;
extern reporter Motor_Reporter_Cache[4];

// ! ========================= 接 口 函 数 声 明 ========================= ! //

static inline void entry_init(void) {
    Motor_Driver_Init();
    memset(Motor_Reporter_Cache, 0, sizeof(Motor_Reporter_Cache));

    HAL_TIM_Base_Start_IT(&htim15);

    RS06_turn();

    delay_ms_init(HAL_GetTick);

    // Motor_SetMode(0x01);
    // delay_ms(1000);
}

static inline void entry_loop(void) {
    uint8_t key_now = Key_Scan();
    if(key_now == 1 && last_key_state == 0) {
        current = !current;
    }
    last_key_state = key_now;
    if(current == 0) {
        Motor_Speed_Control_Smooth(0, 0x01);
    }
    else {
        Motor_Speed_Control_Smooth(100, 0x01);
    }
}

#endif
