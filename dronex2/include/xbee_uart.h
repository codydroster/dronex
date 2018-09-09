

#include "init.h"
#include "stm32l496xx.h"
#include <stdint.h>


volatile uint8_t uart_receive[12];
uint8_t uart_index;





void xbee_recieve(USART_TypeDef *pUART);


void xbee_uart_init(void);

void DMA_init_Xbee(void);
