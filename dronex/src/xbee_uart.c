


#include "xbee_uart.h"



void xbee_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART) {


//alternate function
pGPIO->MODER &= ~((3U << XBEE_RX_PIN*2) | (3U << XBEE_TX_PIN*2));		//clear mode TX , RX
pGPIO->MODER |= ((2U << XBEE_RX_PIN*2) | (2U << XBEE_TX_PIN*2));		//set alternate mode TX, RX

pGPIO->OSPEEDR |= (2U << XBEE_RX_PIN*2) | (2U << XBEE_TX_PIN*2); //speed HIGH TX, RX
pGPIO->PUPDR |= (2U << XBEE_TX_PIN*2);	//PULL-UP enable


pGPIO->AFR[0] |= (7U << XBEE_RX_PIN*4) | (7U << XBEE_TX_PIN*4);	//alternate function 7: USART2


//UART setup

pUART->CR1 |= (1 << 14); 			//CMIE interrupt
pUART->BRR = 0xD0UL;				// 115200 BAUD
pUART->CR2 |= (0x68 << 24); 	//ADD[7:0]
pUART->CR1 |= (1 << 0);				//USART Enable
pUART->CR1 |= (1 << 3) | (1 << 2); 	//TX, RX Enable





}


