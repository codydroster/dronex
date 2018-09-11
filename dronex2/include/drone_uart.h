#include "init.h"
#include <stdint.h>
#include "stm32l496xx.h"


#define RX_PIN 10UL
#define TX_PIN 9UL





uint8_t transmit_data[16];


//USART_TypeDef *pUART2;




void uart_transmit(USART_TypeDef *pUART);

void timer_init2(void);


void drone_uart_init(void);

void timer_init(void);

