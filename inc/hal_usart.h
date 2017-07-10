#ifndef HAL_USART
#define HAL_USART

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>


int putchar_poll (int c);
int getchar_poll (void);

void uart_init(USART_TypeDef* USARTx, uint32_t USART_BaudRate);
int uart_close ( USART_TypeDef * USARTx );


#ifdef __cplusplus
}
#endif

#endif
