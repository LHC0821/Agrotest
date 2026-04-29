#ifndef _entry_h_
#define _entry_h_

// ! 系 统 文 件 ! //
#include "main.h"   // IWYU pragma: keep
#include "dma.h"    // IWYU pragma: keep
#include "fdcan.h"  // IWYU pragma: keep
#include "tim.h"    // IWYU pragma: keep
#include "usart.h"  // IWYU pragma: keep
#include "gpio.h"   // IWYU pragma: keep

#include <stdio.h>

// ! 平 台 层 ! //
#include "key.h"
// #include "can.h"

// ! 基 础 设 施 层 ! //
#include "delay.h"

// ! 设 备 层 ! //
#include "motor.h"
#include "servo.h"

// ! 服 务 层 ! //
#include "chassis_controller.h"


// ! 应 用 层 ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static uint8_t current = 0;
static uint8_t last_key_state = 0; // 用于存储上一次按键状态，实现边沿检测

#define ENTRY_WHEEL_BASE_LENGTH 0.4f
#define ENTRY_WHEEL_BASE_WIDTH  0.4f
#define ENTRY_WHEEL_RADIUS      0.05f
#define ENTRY_MAX_WHEEL_SPEED   1.0f
#define ENTRY_TEST_SPEED        0.2f
#define ENTRY_SERVO_REPORT_MS   10U

static ms_t chassis_update_task = 0;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

static inline void entry_init(void) {
    motor.init();
    servo.init(&hfdcan2);
    chassis.init(ENTRY_WHEEL_BASE_LENGTH, ENTRY_WHEEL_BASE_WIDTH, ENTRY_WHEEL_RADIUS, ENTRY_MAX_WHEEL_SPEED);

    for(uint8_t id = 5U; id <= 8U; id++) {
        (void)servo.set_mode(id, servo.PP);
        (void)servo.enable(id);
        (void)servo.config_reporting(id, ENTRY_SERVO_REPORT_MS);
    }

    delay_ms_init(HAL_GetTick);

    // servo.reset();
    delay_ms(1000);
    printf("Initialization complete.\r\n");
}

static inline void entry_loop(void) {
    if(delay_nb_ms(&chassis_update_task, 10)) { // 每100ms更新一次底盘
        chassis.update();
        chassis.set_chassis(0.0f, 0.0f, 0.0f);
    }
}

#endif
