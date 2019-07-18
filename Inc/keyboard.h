#ifndef __KEY_BORD_EXTI_H
#define __KEY_BORD_EXTI_H
#include "stm32f1xx_hal.h"

void Key_Bord_Init(void);
uint8_t Get_KeyNum(uint16_t GPIO_Pin);

#endif

