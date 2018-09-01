


#include "stm32l496xx.h"
#include <stdint.h>


volatile uint8_t uart_receive[12];
uint8_t uart_index;

USART_TypeDef *pUART2;


void xbee_uart_init(GPIO_TypeDef *pGPIOrx, GPIO_TypeDef *pGPIOtx, USART_TypeDef *pUART);

void xbee_recieve(USART_TypeDef *pUART);
