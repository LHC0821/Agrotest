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

// ! 平 台 层 ! //
#include "key.h"
// #include "can.h"

// ! 基 础 设 施 层 ! //
#include "delay.h"

// ! 领 域 层 ! //
#include "AGV_Chassis.h"


// ! 设 备 层 ! //
#include "motor.h"
#include "servo.h"

// ! 服 务 层 ! //


// ! 应 用 层 ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static uint8_t current = 0;
static uint8_t last_key_state = 0; // 用于存储上一次按键状态，实现边沿检测

// ! ========================= 接 口 函 数 声 明 ========================= ! //

static inline void entry_init(void) {
    motor.init();
    servo.init(&hfdcan1);
    Chassis_Init();

    delay_ms_init(HAL_GetTick);

    // motor.set_mode_raw(0x01);
    // delay_ms(1000);
}

static inline void entry_loop(void) {
    uint8_t key_now = key_scan();
    if(key_now == 1 && last_key_state == 0) {
        current = !current;
    }
    last_key_state = key_now;
    if(current == 0) {
        Chassis_Set_WheelSpeedSmooth(1, 0);
    }
    else {
        Chassis_Set_WheelSpeedSmooth(1, 100);
    }
}

#endif
