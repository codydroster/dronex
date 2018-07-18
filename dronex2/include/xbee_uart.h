


#include "stm32l496xx.h"
#include <stdint.h>


#define XBEE_RX_PIN 3UL
#define XBEE_TX_PIN 2UL


void xbee_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART);

void xbee_recieve(USART_TypeDef *pUART);
