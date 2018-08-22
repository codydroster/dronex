


#include "stm32l496xx.h"
#include <stdint.h>





void xbee_uart_init(GPIO_TypeDef *pGPIOrx, GPIO_TypeDef *pGPIOtx, USART_TypeDef *pUART);

void xbee_recieve(USART_TypeDef *pUART);
