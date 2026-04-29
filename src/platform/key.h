#ifndef __KEY_H
#define __KEY_H

#include "stm32h7xx_hal.h"

/**
 * @brief 按键扫描
 * @return 按键状态值
 */
uint8_t key_scan(void);

#endif
