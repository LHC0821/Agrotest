#include "stm32h7xx_hal.h"

#include "delay.h"

uint8_t Key_Scan(void) {
    uint8_t KeyNum = 0;

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) {
        delay_ms(20);
        while((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0));
        delay_ms(20);

        KeyNum = 1;
    }

    return KeyNum;
}
