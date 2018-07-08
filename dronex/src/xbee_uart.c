


#include "xbee_uart.h"



xbee_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART) {


//alternate function
pGPIO->MODER &= (2U << RX_PIN) | (2UL << 3);




}
