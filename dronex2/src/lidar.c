


#include "lidar.h"
#include "pindefines.h"






void lidar_uart_init(void)
{


	//alternate function rx
	pGPIOC->MODER &= ~(3U << (lidar_rx*2));		//clear mode RX
	pGPIOC->MODER |= (2U << (lidar_rx*2));		//set alternate mode RX

	//alternate function tx
	pGPIOC->MODER &= ~(3U << (lidar_tx*2));		//clear mode TX
	pGPIOC->MODER |= (2U << (lidar_tx*2));		//set alternate mode TX


	pGPIOC->OSPEEDR |= (2U << (lidar_rx*2)); //speed HIGH RX


	pGPIOC->OSPEEDR |= (2U << (lidar_tx*2)); //speed HIGH TX
	pGPIOC->PUPDR |= (2U << (lidar_tx*2));	//PULL-UP enable


	pGPIOC->AFR[0] |= (7U << (lidar_rx*4));	//alternate function 8: USART5 RX

	pGPIOC->AFR[0] |= (7U << (lidar_tx*4));	//alternate function 8: USART5 TX


	//UART setup

	pUARTLID->CR1 |= (1 << 5); 			//interrupt RXNE
	pUARTLID->BRR = 0xD0UL;				// 115200 BAUD

	pUARTLID->CR1 |= (1 << 0);				//USART Enable
	pUARTLID->CR1 |= (1 << 2) | (1 << 3); 		//RX Enable
	pUARTLID->CR3 |= (1 << 6);			//DMA Enable


}


void DMA_init_lidar(void)
{
	pDMA1C3->CPAR = (uint32_t) &pUARTLID->RDR;
	pDMA1C3->CMAR = (uint32_t) &lidar_receive;

	pDMA1C3->CNDTR = 9U;	//8 bytes
	pDMA1SEL->CSELR |= (2U << 8);	//channel selection
	pDMA1C3->CCR |= (1 << 7);	//memory increment
	pDMA1C3->CCR |= (1 << 5);	//circular mode
	pDMA1C3->CCR |= (1 << 1); 	//transfer complete interrupt enable


}

