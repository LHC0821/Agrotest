#include "stm32h7xx_hal.h"

uint8_t Key_Scan(void)
{
		uint8_t KeyNum = 0;
	
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0)
		{
				HAL_Delay(20);	
				while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0));
				HAL_Delay(20);	
			
				KeyNum = 1;
		}		
	
		return KeyNum;
}
