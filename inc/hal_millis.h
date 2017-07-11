
#ifndef MILLIS_H
  #define MILLIS_H

#include "stm32f4xx.h"
extern volatile uint32_t ticker;

void SysTick_Init(void);
extern uint32_t micros(void);
extern uint32_t millis(void);
extern void delay(uint32_t);
extern void delayUs(uint32_t);

#endif
