#ifndef __INTERRUPT
#define __INTERRUPT

#include "init.h"


void DMA2_Channel2_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);

void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);	//SPI2 RX
void DMA1_Channel5_IRQHandler(void);	//SPI2 TX


void SPI2_IRQHandler(void);
void UART5_IRQHandler(void);
void USART3_IRQHandler(void);


void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

uint8_t *spi_index = 0;

#endif
