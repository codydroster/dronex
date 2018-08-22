


#include "xbee_uart.h"
#include "pindefines.h"


void xbee_uart_init(GPIO_TypeDef *pGPIOrx, GPIO_TypeDef *pGPIOtx, USART_TypeDef *pUART) {


	//alternate function rx
	pGPIOrx->MODER &= ~(3U << (XBEE_RX_PIN*2));		//clear mode RX
	pGPIOrx->MODER |= (2U << (XBEE_RX_PIN*2));		//set alternate mode RX

	//alternate function tx
	pGPIOtx->MODER &= ~(3U << (XBEE_TX_PIN*2));		//clear mode TX
	pGPIOtx->MODER |= (2U << (XBEE_TX_PIN*2));		//set alternate mode TX


	pGPIOrx->OSPEEDR |= (2U << (XBEE_RX_PIN*2)); //speed HIGH RX
	//pGPIOrx->PUPDR |= (2U << (XBEE_RX_PIN*2));	//PULL-UP enable

	pGPIOtx->OSPEEDR |= (2U << (XBEE_TX_PIN*2)); //speed HIGH TX
	pGPIOtx->PUPDR |= (2U << (XBEE_TX_PIN*2));	//PULL-UP enable


	pGPIOrx->AFR[0] |= (8U << (XBEE_RX_PIN*4));	//alternate function 8: USART5 RX

	pGPIOtx->AFR[1] |= (8U << 16);	//alternate function 8: USART5 TX


	//UART setup

	pUART->CR1 |= (1 << 5); 			//interrupt RXNE
	pUART->BRR = 0xD0UL;				// 115200 BAUD
	//pUART->CR2 |= (0x68 << 24); 		//ADD[7:0]
	pUART->CR1 |= (1 << 0);				//USART Enable
	pUART->CR1 |= (1 << 2) | (1 << 3); 	//RX Enable
	pUART->CR3 |= (1 << 6);			//DMA Enable




}


