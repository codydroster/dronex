


#include "xbee_uart.h"
#include "pindefines.h"

void xbee_uart_init(void)
{


	//alternate function rx
	pGPIOD->MODER &= ~(3U << (XBEE_RX_PIN*2));		//clear mode RX
	pGPIOD->MODER |= (2U << (XBEE_RX_PIN*2));		//set alternate mode RX

	//alternate function tx
	pGPIOC->MODER &= ~(3U << (XBEE_TX_PIN*2));		//clear mode TX
	pGPIOC->MODER |= (2U << (XBEE_TX_PIN*2));		//set alternate mode TX


	pGPIOD->OSPEEDR |= (2U << (XBEE_RX_PIN*2)); //speed HIGH RX
	//pGPIOD->PUPDR |= (2U << (XBEE_RX_PIN*2));	//PULL-UP enable

	pGPIOC->OSPEEDR |= (2U << (XBEE_TX_PIN*2)); //speed HIGH TX
	pGPIOC->PUPDR |= (2U << (XBEE_TX_PIN*2));	//PULL-UP enable


	pGPIOD->AFR[0] |= (8U << (XBEE_RX_PIN*4));	//alternate function 8: USART5 RX

	pGPIOC->AFR[1] |= (8U << 16);	//alternate function 8: USART5 TX


	//UART setup

	pUART5->CR1 |= (1 << 4); 			//interrupt idle line
	pUART5->BRR = 0xD0UL;				// 115200 BAUD

	pUART5->CR1 |= (1 << 0);				//USART Enable
	pUART5->CR1 |= (1 << 2) | (1 << 3); 	//RX, TX Enable
	pUART5->CR3 |= (1 << 6) | (1 << 7);	//DMA Enable TX, RX


}



void DMA_init_Xbee(void)	//xbee
{

	//channel 2
	pDMA2C2->CPAR = (uint32_t) &pUART5->RDR;
	pDMA2C2->CMAR = (uint32_t) &uart_receive;

	pDMA2C2->CNDTR = 12U;	//12 bytes
	pDMA2SEL->CSELR |= (2U << 4);	//channel selection
	pDMA2C2->CCR |= (1 << 7);	//memory increment
	pDMA2C2->CCR |= (1 << 5);	//circular mode
	pDMA2C2->CCR |= (1 << 1); 	//transfer complete interrupt enable


	//channel 1
	pDMA2C1->CPAR = (uint32_t) &pUART5->TDR;
	//pDMA2C1->CMAR = (uint32_t) &lidar_transmit;		test lidar

	pDMA2C1->CCR |= (1 << 4);	//DIR: mem to per
	pDMA2C1->CNDTR = 1U;	//1 bytes
	pDMA2C1->CCR |= (1 << 5);	//circular mode
	pDMA2SEL->CSELR |= (2U << 0);	//channel selection
	pDMA2C1->CCR |= (1 << 1); 	//transfer complete interrupt enable

	pUART5->CR1 |= (1 << 3);		//UART TX ENABLE
	pUART5->ICR |= (1 << 6);		//TC flag clear
}






