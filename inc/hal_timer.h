#ifndef HAL_TIMER
#define HAL_TIMER
 /* 	TIMER	|CHANNEL 1				|CHANNEL 2				|CHANNEL 3				|CHANNEL 4
 * 			|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3
 *
 * 	TIM 2	|PA0	PA5		PA15	|PA1	PB3		-		|PA2	PB10	-		|PA3	PB11	-
 * 	TIM 3	|PA6	PB4		PC6		|PA7	PB5		PC7		|PB0	PC8		-		|PB1	PC9		-
 * 	TIM 4	|PB6	PD12	-		|PB7	PD13	-		|PB8	PD14	-		|PB9	PD15	-
 * 	TIM 5	|PA0	PH10	-		|PA1	PH11	-		|PA2	PH12	-		|PA3	PI0		-
 * 	TIM 8	|PC6	PI5		-		|PC7	PI6		-		|PC8	PI7		-		|PC9	PI2		-
 * 	TIM 9	|PA2	PE5		-		|PA3	PE6		-		|-		-		-		|-		-		-
 * 	TIM 10	|PB8	PF6		-		|-		-		-		|-		-		-		|-		-		-
 * 	TIM 11	|PB9	PF7		-		|-		-		-		|-		-		-		|-		-		-
 * 	TIM 12	|PB14	PH6		-		|PB15	PH9		-		|-		-		-		|-		-		-
 * 	TIM 13	|PA6	PF8		-		|-		-		-		|-		-		-		|-		-		-
 * 	TIM 14	|PA7	PF9		-		|-		-		-		|-		-		-		|-		-		-
 *
 * 	- PPx: Pins Pack 1 to 3, for 3 possible channel outputs on timer.
 *
 * Notes on table above
 * 	- Not all timers are available on all STM32F4xx devices
 * 	- All timers have 16bit prescaler
 * 	- TIM6 and TIM7 do not have PWM feature, they are only basic timers
 * 	- TIM2 and TIM5 are 32bit timers
 * 	- TIM9 and TIM12 have two PWM channels
 * 	- TIM10, TIM11, TIM13 and TIM14 have only one PWM channel
 * 	- All channels at one timer have the same PWM frequency!
 */
/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>


typedef struct {
	uint32_t TimerFrequency;
	uint32_t MaxPeriod;
	uint32_t MaxPrescaler;
	uint32_t Period;
	uint32_t Prescaler;
	uint32_t Frequency;
} TIMER_PROPERTIES_t;

typedef enum {
	TIMER_PROPERTIES_Result_Ok,
	TIMER_PROPERTIES_Result_Error,
	TIMER_PROPERTIES_Result_TimerNotValid,
	TIMER_PROPERTIES_Result_FrequencyTooHigh,
	TIMER_PROPERTIES_Result_FrequencyTooLow
} TIMER_PROPERTIES_Result_t;

typedef enum {
	PWM_Result_Ok = 0,
	PWM_Result_FrequencyTooHigh,
	PWM_Result_FrequencyTooLow,
	PWM_Result_PulseTooHigh,
	PWM_Result_TimerNotValid,
	PWM_Result_ChannelNotValid,
	PWM_Result_PinNotValid
} PWM_Result_t;

typedef struct {
	uint32_t Period;
	uint32_t Prescaler;
	uint32_t Frequency;
	uint32_t Micros;
} PWM_TIM_t;

typedef enum {
	PWM_Channel_1,
	PWM_Channel_2,
	PWM_Channel_3,
	PWM_Channel_4
} PWM_Channel_t;

typedef enum {
	PWM_PinsPack_1,
	PWM_PinsPack_2,
	PWM_PinsPack_3
} PWM_PinsPack_t;

extern TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_GetTimerProperties(TIM_TypeDef* TIMx, TIMER_PROPERTIES_t* Timer_Data);
extern TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_GenerateDataForWorkingFrequency(TIMER_PROPERTIES_t* Timer_Data, double frequency);
extern TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_EnableClock(TIM_TypeDef* TIMx);
extern TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_DisableClock(TIM_TypeDef* TIMx);

extern PWM_Result_t PWM_InitTimer(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, double PWMFrequency);
extern PWM_Result_t PWM_InitChannel(TIM_TypeDef* TIMx, PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
extern PWM_Result_t PWM_SetChannel(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, uint32_t Pulse);
extern PWM_Result_t PWM_SetChannelPercent(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, float percent);
extern PWM_Result_t PWM_SetChannelMicros(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, uint32_t micros);

int timer_init(TIM_TypeDef * Timer, uint32_t period, uint32_t accuracy);

#ifdef __cplusplus
}
#endif

#endif
