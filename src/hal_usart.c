#include "hal_usart.h"

int putchar_poll (USART_TypeDef* USARTx, int c)
{
	while ( USART_GetFlagStatus (USARTx , USART_FLAG_TXE ) == RESET);
	USARTx ->DR = (c & 0xff);
	return 0;
}


int getchar_poll (USART_TypeDef* USARTx)
{
	while ( USART_GetFlagStatus (USARTx , USART_FLAG_RXNE ) == RESET);
	return USARTx ->DR & 0xff;
}


int getchar_int (USART_TypeDef* USARTx)
{
	struct Queue* que;
	GPIO_TypeDef * port;
	uint16_t pin_cts;

	if(USARTx == USART1){
		que = &UART1_RXq;
		pin_cts = GPIO_Pin_12;
		port = GPIOA;
	}
	else if(USARTx == USART2)
	{
		que = &UART2_RXq;
		pin_cts = GPIO_Pin_3;
		port = GPIOD;
	}
	else if(USARTx == USART3)
	{
		que = &UART3_RXq;
		pin_cts = GPIO_Pin_11;
		port = GPIOD;
	}
	else if(USARTx == USART2)
	{
		que = &UART6_RXq;
		pin_cts = GPIO_Pin_13;
		port = GPIOG;
	}
	else{
		return 0;
	}

	uint8_t data;
	while (! Dequeue (que , &data));

	if(USARTx->CR3 == USART_HardwareFlowControl_CTS)
	{
		// If the queue has fallen below high water mark , enable nRTS
		if ( QueueAvail (que) <= HIGH_WATER )
		GPIO_WriteBit (port , pin_cts , 0);
	}

	return data;
}

int putchar_int (USART_TypeDef* USARTx, int c)
{
	struct Queue* que;
	USART_TypeDef* usart;

	if(USARTx == USART1)
	{
		que = &UART1_TXq;
		usart = USART1;
	}
	else if(USARTx == USART2)
	{
		que = &UART2_TXq;
		usart = USART2;
	}
	else if(USARTx == USART3)
	{
		que = &UART3_TXq;
		usart = USART3;
	}
	else if(USARTx == USART6)
	{
		que = &UART6_TXq;
		usart = USART6;
	}
	else{
		return 0;
	}

	while (! Enqueue (que , c));
	if (! TxPrimed ) {
		TxPrimed = 1;
		USART_ITConfig (usart , USART_IT_TXE , ENABLE );
		return 1;
	}else{
		return 0;
	}
}

int uart_init(USART_TypeDef* USARTx, uint32_t USART_BaudRate, uint32_t flags, uint8_t flowcontrol)
{
	// XXX TODO: make a function to provide pins and port for each USART depends on USARTx
	uint16_t pin_ck, pin_tx,pin_rx,pin_rts,pin_cts;
	GPIO_TypeDef * port;
	uint32_t apb, ahb;
	if(USARTx == USART1){
		pin_ck = GPIO_Pin_8;
		pin_tx = GPIO_Pin_9;
		pin_rx = GPIO_Pin_10;
		pin_rts = GPIO_Pin_11;
		pin_cts = GPIO_Pin_12;
		port = GPIOA;
		apb= RCC_APB2Periph_USART1;
		ahb = RCC_AHB1Periph_GPIOA;
	}else if(USARTx == USART2)
	{
		pin_ck = GPIO_Pin_7;
		pin_tx = GPIO_Pin_5;
		pin_rx = GPIO_Pin_6;
		pin_rts = GPIO_Pin_4;
		pin_cts = GPIO_Pin_3;
		port = GPIOD;
		apb= RCC_APB1Periph_USART2;
		ahb = RCC_AHB1Periph_GPIOD;
	}else if(USARTx == USART3)
	{
		pin_ck = GPIO_Pin_10;
		pin_tx = GPIO_Pin_8;
		pin_rx = GPIO_Pin_9;
		pin_rts = GPIO_Pin_12;
		pin_cts = GPIO_Pin_11;
		port = GPIOD;
		apb= RCC_APB1Periph_USART3;
		ahb = RCC_AHB1Periph_GPIOD;
	}else if(USARTx == USART6)
	{
		pin_ck = GPIO_Pin_7;
		pin_tx = GPIO_Pin_14;
		pin_rx = GPIO_Pin_9;
		pin_rts = GPIO_Pin_12;
		pin_cts = GPIO_Pin_13;
		port = GPIOG;
		apb= RCC_APB2Periph_USART6;
		ahb = RCC_AHB1Periph_GPIOG;
	}else{
		return 0;
	}
	/* USART4, USART5 use pins of USART3 */

	//RCC_APB2Periph_AFIO is required?
	//DONE: XXX check if the USART1 is on GPIOA if not change the GIO accordingly
	RCC_APB2PeriphClockCmd ( apb  , ENABLE );
	RCC_AHB1PeriphClockCmd ( ahb, ENABLE);
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

	if(flowcontrol)
	{
		// Configure CTS pin
		GPIO_InitStruct . GPIO_Pin = pin_cts ;
		GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	GPIO_InitStruct . GPIO_Speed = GPIO_Speed_2MHz ;
		GPIO_Init (port , & GPIO_InitStruct );

		// Configure RTS pin -- software controlled
		GPIO_WriteBit (port , pin_rts , 1);

		// nRTS disabled
		GPIO_InitStruct . GPIO_Pin = pin_rts ;
		GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct . GPIO_Speed = GPIO_Speed_2MHz ;
		GPIO_Init (port , & GPIO_InitStruct );
	}


	// see stm32f10x_usart .h
	USART_InitTypeDef USARTinit;

	// Initialize USART structure
	USART_StructInit (&USARTinit );
	// Modify USARTx for non - default values , e.g.
	// USARTx . USART_BaudRate = 9600; //or 38400
	USARTinit.USART_BaudRate = USART_BaudRate;
	USARTinit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
	if(flowcontrol){
		USARTinit.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
	}
	USART_Init (USARTx ,&USARTinit );
	USART_Cmd (USARTx , ENABLE );

	if(flowcontrol)
	{
		// nRTS enabled
		GPIO_WriteBit (GPIOA , pin_rts , 0);
	}

	return 1;
}

int uart_close ( USART_TypeDef * USARTx )
{
	USART_DeInit(USARTx);
	return 1;
}
