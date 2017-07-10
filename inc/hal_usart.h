#ifndef HAL_USART
#define HAL_USART

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>

#define HIGH_WATER ( QUEUE_SIZE - 6)
struct Queue UART1_TXq , UART1_RXq ,UART2_TXq , UART2_RXq, UART3_TXq , UART3_RXq,UART6_TXq , UART6_RXq;

int putchar_poll (USART_TypeDef* USARTx,int c);
int getchar_poll (USART_TypeDef* USARTx);
int getchar_int (USART_TypeDef* USARTx);
int putchar_int (USART_TypeDef* USARTx, int c);

int uart_init(USART_TypeDef* USARTx, uint32_t USART_BaudRate, uint32_t flags, uint8_t flowcontrol);
int uart_close ( USART_TypeDef * USARTx );


#ifdef __cplusplus
}
#endif

#endif
