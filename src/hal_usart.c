#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>

#include "hal_usart.h"

int putchar_poll (USART_TypeDef* USARTx, int c)
{
	while ( USART_GetFlagStatus (USARTx , USART_FLAG_TXE ) == RESET);
	USARTx ->DR = (c & 0xff);
	return 0;
}

int getchar_poll (USART_TypeDef* USARTx, void)
{
	while ( USART_GetFlagStatus (USARTx , USART_FLAG_RXNE ) == RESET);
	return USARTx ->DR & 0xff;
}

int uart_init(USART_TypeDef* USARTx, uint32_t USART_BaudRate, uint32_t flags)
{
	// XXX TODO: make a function to provide pins and port for each USART depends on USARTx
	uint16_t pin_ck, pin_tx,pin_rx,pin_rts,pin_cts;
	GPIO_TypeDef * port;
	uint32_t apb, ahb;
	switch(USARTx){
		case USART1:
			pin_ck = GPIO_Pin_8;
			pin_tx = GPIO_Pin_9;
			pin_rx = GPIO_Pin_10;
			pin_rts = GPIO_Pin_11;
			pin_cts = GPIO_Pin_12;
			port = GPIOA;
			apb= RCC_APB2Periph_USART1;
			ahb = RCC_AHB1Periph_GPIOA;
			break;
		case USART2:
			pin_ck = GPIO_Pin_7;
			pin_tx = GPIO_Pin_5;
			pin_rx = GPIO_Pin_6;
			pin_rts = GPIO_Pin_4;
			pin_cts = GPIO_Pin_3;
			port = GPIOD;
			apb= RCC_APB1Periph_USART2;
			ahb = RCC_AHB1Periph_GPIOD;
			break;
		case USART3:
			pin_ck = GPIO_Pin_10;
			pin_tx = GPIO_Pin_8;
			pin_rx = GPIO_Pin_9;
			pin_rts = GPIO_Pin_12;
			pin_cts = GPIO_Pin_11;
			port = GPIOD;
			apb= RCC_APB1Periph_USART3;
			ahb = RCC_AHB1Periph_GPIOD;
			break;
		case USART6:
			pin_ck = GPIO_Pin_7;
			pin_tx = GPIO_Pin_14;
			pin_rx = GPIO_Pin_9;
			pin_rts = GPIO_Pin_12;
			pin_cts = GPIO_Pin_13;
			port = GPIOG;
			apb= RCC_APB2Periph_USART6;
			ahb = RCC_AHB1Periph_GPIOG;
			break;
	/* USART4, USART5 uses pins of USART3 */
	}



	//RCC_APB2Periph_AFIO is required?
	//TODO: XXX check if the USART1 is on GPIOA if not change the GIO accordingly
	RCC_APB2PeriphClockCmd ( apb | ahb , ENABLE );

	GPIO_InitTypeDef GPIO_InitStruct ;
	GPIO_StructInit (& GPIO_InitStruct );

	// Initialize USART1_Tx
	GPIO_InitStruct . GPIO_Pin = pin_tx ;
	GPIO_InitStruct . GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init (port , & GPIO_InitStruct );

	// Initialize USART1_RX
	GPIO_InitStruct . GPIO_Pin = pin_rx ;
	GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;

	GPIO_Init (port , & GPIO_InitStruct );

	// see stm32f10x_usart .h
	USART_InitTypeDef USARTinit;

	// Initialize USART structure
	USART_StructInit (&USARTinit );
	// Modify USARTx for non - default values , e.g.
	// USARTx . USART_BaudRate = 9600; //or 38400
	USARTinit.USART_BaudRate = USART_BaudRate;
	USARTinit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
	USART_Init (USARTx ,&USARTinit );
	USART_Cmd (USARTx , ENABLE );

	return 1;
}

int uart_close ( USART_TypeDef * USARTx )
{
	USART_DeInit(USARTx);
	return 1;
}
