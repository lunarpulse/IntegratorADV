#include "hal_millis.h"

volatile uint32_t ticker;

void SysTick_Init(void)
{

	if(SysTick_Config(SystemCoreClock / 1000))
		while(1) //SystemCoreClock / 1000 or 53760 here roughly 1ms 1kHz
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetPriority(SysTick_IRQn,0);
	//SCB->SHP[0x0B] = 0X00; //high priority low number? or high number?
	NVIC_EnableIRQ(SysTick_IRQn);

  ticker = 0;
}

uint32_t micros(void)
{

	return ticker * 1000 + 1000 - SysTick->VAL/(SystemCoreClock/1000000UL);

}

uint32_t millis(void)
{
	return ticker;
}

void delay(uint32_t nTime)
{
	uint32_t curTime = ticker;
  while((nTime-(ticker-curTime)) > 0);
}

void delayUs(uint32_t nTime)
{
	uint32_t curTime = micros();
	while((nTime-(micros()-curTime)) > 0);
}
