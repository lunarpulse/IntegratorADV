#include "hal_timer.h"


TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_GetTimerProperties(TIM_TypeDef* TIMx, TIMER_PROPERTIES_t* Timer_Data) {
	RCC_ClocksTypeDef RCC_ClocksStruct;

	/* Get clocks */
	RCC_GetClocksFreq(&RCC_ClocksStruct);

	/* All timers have 16-bit prescaler */
	Timer_Data->MaxPrescaler = 0xFFFF;

	if ( /* 32bit timers with PCLK2 max frequency */
		TIMx == TIM2 ||
		TIMx == TIM5
	) {
		Timer_Data->TimerFrequency = RCC_ClocksStruct.PCLK2_Frequency;	/* Clock */
		Timer_Data->MaxPeriod = 0xFFFFFFFF;							/* Max period */

		return TIMER_PROPERTIES_Result_Ok;
	} else if (	/* 16bit timers with HCLK clock frequency */
		TIMx == TIM1 ||
		TIMx == TIM8 ||
		TIMx == TIM9 ||
		TIMx == TIM10 ||
		TIMx == TIM11
	) {
		Timer_Data->TimerFrequency = RCC_ClocksStruct.HCLK_Frequency;	/* Clock */
		Timer_Data->MaxPeriod = 0xFFFF;								/* Max period */

		return TIMER_PROPERTIES_Result_Ok;
	} else if (	/* 16bit timers with PCLK2 clock frequency */
		TIMx == TIM3 ||
		TIMx == TIM4 ||
		TIMx == TIM6 ||
		TIMx == TIM7 ||
		TIMx == TIM12 ||
		TIMx == TIM13 ||
		TIMx == TIM14
	) {
		Timer_Data->TimerFrequency = RCC_ClocksStruct.PCLK2_Frequency;	/* Clock */
		Timer_Data->MaxPeriod = 0xFFFF;								/* Max period */

		return TIMER_PROPERTIES_Result_Ok;
	}
	/* Timer is not valid */
	return TIMER_PROPERTIES_Result_TimerNotValid;
}

TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_GenerateDataForWorkingFrequency(TIMER_PROPERTIES_t* Timer_Data, double frequency) {
	if (frequency > Timer_Data->TimerFrequency) {
		/* Reset values */
		Timer_Data->Prescaler = 0;
		Timer_Data->Period = 0;
		Timer_Data->Frequency = 0;

		/* Frequency too high */
		return TIMER_PROPERTIES_Result_FrequencyTooHigh;
	} else if (frequency == 0) {
		/* Reset values */
		Timer_Data->Prescaler = 0;
		Timer_Data->Period = 0;
		Timer_Data->Frequency = 0;

		/* Not valid frequency */
		return TIMER_PROPERTIES_Result_FrequencyTooLow;
	}

	/* Fix for 16/32bit timers */
	if (Timer_Data->MaxPeriod <= 0xFFFF) {
		Timer_Data->MaxPeriod++;
	}

	/* Get minimum prescaler and maximum resolution for timer */
	Timer_Data->Prescaler = 0;
	do {
		/* Get clock */
		Timer_Data->Period = (Timer_Data->TimerFrequency / (Timer_Data->Prescaler + 1));
		/* Get period */
		Timer_Data->Period = (Timer_Data->Period / frequency);
		/* Increase prescaler value */
		Timer_Data->Prescaler++;
	} while (Timer_Data->Period > (Timer_Data->MaxPeriod) && Timer_Data->Prescaler <= (Timer_Data->MaxPrescaler + 1));
	/* Check for too low frequency */
	if (Timer_Data->Prescaler > (Timer_Data->MaxPrescaler + 1)) {
		/* Reset values */
		Timer_Data->Prescaler = 0;
		Timer_Data->Period = 0;
		Timer_Data->Frequency = 0;

		/* Prescaler too high, frequency is too low for use */
		return TIMER_PROPERTIES_Result_FrequencyTooLow;
	}

	/* Set frequency */
	Timer_Data->Frequency = frequency;

	/* Return OK */
	return TIMER_PROPERTIES_Result_Ok;
}

TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_EnableClock(TIM_TypeDef* TIMx) {
	if (TIMx == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	} else if (TIMx == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	} else if (TIMx == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	} else if (TIMx == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	} else if (TIMx == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	} else if (TIMx == TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	} else if (TIMx == TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	} else if (TIMx == TIM8) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	} else if (TIMx == TIM9) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	} else if (TIMx == TIM10) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	} else if (TIMx == TIM11) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	} else if (TIMx == TIM12) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	} else if (TIMx == TIM13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	} else if (TIMx == TIM14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}
	/* Return OK */
	return TIMER_PROPERTIES_Result_Ok;
}
/*
 * @brief:
 *
 */
TIMER_PROPERTIES_Result_t TIMER_PROPERTIES_DisableClock(TIM_TypeDef* TIMx) {
	if (TIMx == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
	} else if (TIMx == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
	} else if (TIMx == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
	} else if (TIMx == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
	} else if (TIMx == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
	} else if (TIMx == TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
	} else if (TIMx == TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
	} else if (TIMx == TIM8) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, DISABLE);
	} else if (TIMx == TIM9) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, DISABLE);
	} else if (TIMx == TIM10) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
	} else if (TIMx == TIM11) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, DISABLE);
	} else if (TIMx == TIM12) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, DISABLE);
	} else if (TIMx == TIM13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, DISABLE);
	} else if (TIMx == TIM14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, DISABLE);
	}
	/* Return OK */
	return TIMER_PROPERTIES_Result_Ok;
}

int timer_init(TIM_TypeDef * Timer, uint32_t period, uint32_t accuracy)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	TIM_OCInitTypeDef TIM_OCInitStructure ;

	// enable timer clock
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2 , ENABLE );
	// configure timer
	// PWM frequency = 100 hz with 24 ,000 ,000 hz system clock
	// 	24 ,000 ,000/240 = 100 ,000
	// 	100 ,000/1000 = 100
	int freq =period * accuracy;
	TIM_TimeBaseStructInit (& TIM_TimeBaseStructure );
	TIM_TimeBaseStructure . TIM_Prescaler = SystemCoreClock /freq - 1; // 0..239
	TIM_TimeBaseStructure . TIM_Period = accuracy - 1; // 0..999
	TIM_TimeBaseStructure . TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit (TIM2 , & TIM_TimeBaseStructure );
	// PWM1 Mode configuration : Channel2
	// 	Edge - aligned ; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure );
	TIM_OCInitStructure . TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure . TIM_OutputState = TIM_OutputState_Enable ;
	TIM_OC2Init (TIM2 , & TIM_OCInitStructure );
	// Enable Timer
	TIM_Cmd (TIM2 , ENABLE );

	return 1;
}

/* Private functions */
//void PWM_INT_EnableClock(TIM_TypeDef* TIMx);

//PWM_Result_t PWM_INT_GetTimerProperties(TIM_TypeDef* TIMx, uint32_t* frequency, uint32_t* period);

PWM_Result_t PWM_INT_InitTIM1Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM2Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM3Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM4Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM5Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM8Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM9Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM10Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM11Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM12Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM13Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
PWM_Result_t PWM_INT_InitTIM14Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);

PWM_Result_t PWM_InitTimer(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, double PWMFrequency) {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIMER_PROPERTIES_t Timer_Data;

	/* Check valid timer */
	if (TIMx == TIM6 || TIMx == TIM7) {
		return PWM_Result_TimerNotValid;
	}

	/* Get timer properties */
	TIMER_PROPERTIES_GetTimerProperties(TIMx, &Timer_Data);
	/* Check for maximum timer frequency */
	if (PWMFrequency > Timer_Data.TimerFrequency) {
		/* Frequency too high */
		return PWM_Result_FrequencyTooHigh;
	} else if (PWMFrequency == 0) {
		/* Not valid frequency */
		return PWM_Result_FrequencyTooLow;
	}

	/* Generate settings */
	TIMER_PROPERTIES_GenerateDataForWorkingFrequency(&Timer_Data, PWMFrequency);

	/* Check valid data */
	if (Timer_Data.Period == 0) {
		return PWM_Result_FrequencyTooHigh;
	}

	/* Tests are OK */
	TIM_Data->Frequency = PWMFrequency;
	TIM_Data->Micros = 1000000 / PWMFrequency;
	TIM_Data->Period = Timer_Data.Period;
	TIM_Data->Prescaler = Timer_Data.Prescaler;

	/* Enable clock for Timer */
	TIMER_PROPERTIES_EnableClock(TIMx);

	/* Set timer options */
	TIM_BaseStruct.TIM_Prescaler = Timer_Data.Prescaler - 1;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = Timer_Data.Period - 1;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;

	/* Initialize timer */
	TIM_TimeBaseInit(TIMx, &TIM_BaseStruct);
	/* Start timer */
	TIM_Cmd(TIMx, ENABLE);

	/* Return OK */
	return PWM_Result_Ok;
}


PWM_Result_t PWM_InitChannel(TIM_TypeDef* TIMx, PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	if (TIMx == TIM1) {
		return PWM_INT_InitTIM1Pins(Channel, PinsPack);
	} else if (TIMx == TIM2) {
		return PWM_INT_InitTIM2Pins(Channel, PinsPack);
	} else if (TIMx == TIM3) {
		return PWM_INT_InitTIM3Pins(Channel, PinsPack);
	} else if (TIMx == TIM4) {
		return PWM_INT_InitTIM4Pins(Channel, PinsPack);
	} else if (TIMx == TIM5) {
		return PWM_INT_InitTIM5Pins(Channel, PinsPack);
	} else if (TIMx == TIM8) {
		return PWM_INT_InitTIM8Pins(Channel, PinsPack);
	} else if (TIMx == TIM9) {
		return PWM_INT_InitTIM9Pins(Channel, PinsPack);
	} else if (TIMx == TIM10) {
		return PWM_INT_InitTIM10Pins(Channel, PinsPack);
	} else if (TIMx == TIM11) {
		return PWM_INT_InitTIM11Pins(Channel, PinsPack);
	} else if (TIMx == TIM12) {
		return PWM_INT_InitTIM12Pins(Channel, PinsPack);
	} else if (TIMx == TIM13) {
		return PWM_INT_InitTIM13Pins(Channel, PinsPack);
	} else if (TIMx == TIM14) {
		return PWM_INT_InitTIM14Pins(Channel, PinsPack);
	}
	/* Timer is not valid */
	return PWM_Result_TimerNotValid;
}

PWM_Result_t PWM_SetChannel(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, uint32_t Pulse) {
	TIM_OCInitTypeDef TIM_OCStruct;

	/* Check pulse length */
	if (Pulse > (TIM_Data->Period - 1)) {
		return PWM_Result_PulseTooHigh;
	}

	/* Common settings */
	TIM_OCStruct.TIM_Pulse = Pulse;
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	switch (Channel) {
		case PWM_Channel_1:
			TIM_OC1Init(TIMx, &TIM_OCStruct);
			TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
			break;
		case PWM_Channel_2:
			TIM_OC2Init(TIMx, &TIM_OCStruct);
			TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
			break;
		case PWM_Channel_3:
			TIM_OC3Init(TIMx, &TIM_OCStruct);
			TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
			break;
		case PWM_Channel_4:
			TIM_OC4Init(TIMx, &TIM_OCStruct);
			TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
			break;
		default:
			break;
	}

	return PWM_Result_Ok;
}

PWM_Result_t PWM_SetChannelPercent(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, float percent) {
	if (percent > 100) {
		return PWM_SetChannel(TIMx, TIM_Data, Channel, TIM_Data->Period);
	} else if (percent <= 0) {
		return PWM_SetChannel(TIMx, TIM_Data, Channel, 0);
	}
	return PWM_SetChannel(TIMx, TIM_Data, Channel, (uint32_t)((float)(TIM_Data->Period - 1) * percent) / 100);
}

PWM_Result_t PWM_SetChannelMicros(TIM_TypeDef* TIMx, PWM_TIM_t* TIM_Data, PWM_Channel_t Channel, uint32_t micros) {
	if (micros > TIM_Data->Micros) {
		return PWM_Result_PulseTooHigh;
	}
	return PWM_SetChannel(TIMx, TIM_Data, Channel, (uint32_t)((TIM_Data->Period - 1) * micros) / TIM_Data->Micros);
}

PWM_Result_t PWM_INT_InitTIM1Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM2Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_3:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM3Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_3:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_3:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM4Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;					/* Set pin */
					GPIO_Init(GPIOD, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;					/* Set pin */
					GPIO_Init(GPIOD, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;					/* Set pin */
					GPIO_Init(GPIOD, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;					/* Set pin */
					GPIO_Init(GPIOD, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM5Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;					/* Set pin */
					GPIO_Init(GPIOH, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;					/* Set pin */
					GPIO_Init(GPIOH, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;					/* Set pin */
					GPIO_Init(GPIOH, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOI, GPIO_PinSource0, GPIO_AF_TIM5);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;					/* Set pin */
					GPIO_Init(GPIOI, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM8Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;					/* Set pin */
					GPIO_Init(GPIOI, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOI, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_3:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOI, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_4:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOC, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_TIM8);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;					/* Set pin */
					GPIO_Init(GPIOI, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM9Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOE, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM10Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOF, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM11Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_TIM11);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOF, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM12Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOH, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		case PWM_Channel_2:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;					/* Set pin */
					GPIO_Init(GPIOB, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOH, GPIO_PinSource9, GPIO_AF_TIM12);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOH, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM13Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;					/* Set pin */
					GPIO_Init(GPIOF, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}

PWM_Result_t PWM_INT_InitTIM14Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack) {
	PWM_Result_t result;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Common settings */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	switch (Channel) {
		case PWM_Channel_1:
			switch (PinsPack) {
				case PWM_PinsPack_1:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;					/* Set pin */
					GPIO_Init(GPIOA, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				case PWM_PinsPack_2:
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	/* Enable clock */
					GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);	/* Alternate function */
					GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;					/* Set pin */
					GPIO_Init(GPIOF, &GPIO_InitStruct);						/* Initialize pin */
					result = PWM_Result_Ok;								/* Result OK */
					break;
				default:
					result = PWM_Result_PinNotValid;
					break;
			}
			break;
		default:
			result = PWM_Result_ChannelNotValid;
			break;
	}

	return result;
}
