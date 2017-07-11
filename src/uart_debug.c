
/* Source code for PRINTF implementation over UART */

#include <stdio.h>
#include <stdarg.h>
//#include "hal_gpio.h"
#include "hal_usart.h"
#include "uart_debug.h"

//uart_handle_t debug_uart_handle;
//USART_TypeDef* USARTx;

/*this function initializes the debug uart */
void hal_debug_uart_init(uint32_t baudrate)
{
	uint32_t apb = RCC_APB1Periph_UART5;
	uint32_t ahb = RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD;
	int pin_tx = GPIO_Pin_12;
	int pin_rx = GPIO_Pin_2;
	GPIO_TypeDef * port_t = GPIOC;
	GPIO_TypeDef * port_r = GPIOD;

	RCC_APB2PeriphClockCmd ( apb  , ENABLE );
	RCC_AHB1PeriphClockCmd ( ahb, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct ;
	GPIO_StructInit (& GPIO_InitStruct );

	GPIO_PinAFConfig(port_t, GPIO_PinSource12, GPIO_AF_UART4);	/* Alternate function */

	// Initialize USART1_Tx
	GPIO_InitStruct . GPIO_Pin = pin_tx ;
	GPIO_InitStruct . GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init (port_t , & GPIO_InitStruct );

	GPIO_PinAFConfig(port_r, GPIO_PinSource2, GPIO_AF_UART4);	/* Alternate function */

	// Initialize USART1_RX
	GPIO_InitStruct . GPIO_Pin = pin_rx ;
	GPIO_InitStruct . GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;

	GPIO_Init (port_r , & GPIO_InitStruct );


	// see stm32f10x_usart .h
	USART_InitTypeDef USARTinit;

	// Initialize USART structure
	USART_StructInit (&USARTinit );
	// Modify USARTx for non - default values , e.g.
	// USARTx . USART_BaudRate = 9600; //or 38400
	USARTinit.USART_BaudRate = baudrate;
	USARTinit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
	USARTinit.USART_WordLength = USART_WordLength_8b;
	USARTinit.USART_Parity = USART_Parity_No;
	USARTinit.USART_StopBits = USART_StopBits_1;
	USART_Init (UART5 ,&USARTinit );
	USART_Cmd (UART5 , ENABLE );

	return ;

	//debug_uart_handle.Init.WordLength   = USART_WL_1S8B;
	//debug_uart_handle.Init.StopBits     = UART_STOPBITS_1;
	//debug_uart_handle.Init.Parity       = UART_PARITY_NONE;
	//debug_uart_handle.Init.Mode         = UART_MODE_TX_RX;
	//debug_uart_handle.Init.OverSampling = USART_OVER16_ENABLE;

	//debug_uart_handle.tx_cmp_cb = 0;
	//debug_uart_handle.rx_cmp_cb = 0;

	//hal_uart_init(&debug_uart_handle);

	/*configure the GPIO_PORT_B_PIN_10 for the TX functionality */
	//uart_pin_conf.pin = DEBUG_UART_TX_PIN;
	//uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	//uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	//uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	//uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	//hal_gpio_set_alt_function(GPIOB,DEBUG_UART_TX_PIN,GPIO_AF_VALUE_USART2);
	//hal_gpio_init(GPIOB ,&uart_pin_conf);

	/*configure the GPIO_PORT_B_PIN_11 for the RX functionality */
	//uart_pin_conf.pin = DEBUG_UART_RX_PIN;
	//hal_gpio_set_alt_function(GPIOB,DEBUG_UART_RX_PIN,GPIO_AF_VALUE_USART2);
	//hal_gpio_init(GPIOB ,&uart_pin_conf);


}


/*this function implements printf over USART3 */
 void uart_printf(char *format,...)
 {
	char str[80];
	char *s;

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	s=str;

	/* until NULL, send out single byte in a blocking fashion */
	while(*s){
	// wait until data register is empty
	while( !(UART5->SR & USART_FLAG_TXE) );
	UART5->DR = *s;
	s++;
	}
	va_end(args);
}
