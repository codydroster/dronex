
#include <stdint.h>
#include "stm32l496xx.h"


#define RX_PIN 10UL
#define TX_PIN 9UL




uint8_t transmit_data[16];






void drone_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART);

void uart_transmit(USART_TypeDef *pUART);

//void TIM2_IRQHandler(void);


void timer_init(TIM_TypeDef *pTIM);

