#include "stm32h7xx_hal.h"

#include "delay.h"

uint8_t key_scan(void) {
    uint8_t key = 0;

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) {
        delay_ms(20);
        while((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0));
        delay_ms(20);

        key = 1;
    }

    return key;
}
