


#include "stm32l4xx.h"
#include <stdint.h>


#define RX_PIN 3UL
#define TX_PIN 2UL


void xbee_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART);
