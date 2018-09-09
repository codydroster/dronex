/*
 * drone_uart.c
 *
 *  Created on: Jun 23, 2018
 *      Author: codydroster
 */

#include "drone_uart.h"
#include "pindefines.h"








void uart_transmit(USART_TypeDef *pUART)
{

	for(int i = 0; i < 16; i++){
		pUART->TDR = transmit_data[i];
		while(!(pUART->ISR & (1 << 7)));	//transmit data register flag


	}



}





void drone_uart_init(void)
{
	//tx init (PA2)
	pGPIOA->MODER &= ~(3UL << (FC_TX * 2));
	pGPIOA->MODER |= (2U << (FC_TX * 2));

	//rx init(PA3)
	pGPIOA->MODER &= ~(3UL << (FC_RX * 2));
	pGPIOA->MODER |= (2UL << (FC_RX * 2));

	//alternate function setup
	pGPIOA->AFR[0] |= (7UL << (FC_RX * 4)) | (7UL << (FC_TX * 4));

	pGPIOA->PUPDR |= (1UL << (FC_RX * 2)) | (1UL << (FC_TX * 2));	//pull up
	pGPIOA->OSPEEDR |= (3UL << (FC_RX * 2)) | (3UL << (FC_TX *2)); //speed




	//USART initialize
	pUART2->CR1 = (0UL << 12) | (0UL << 28); //word length

	//BAUD rate
	pUART2->BRR = 0xD0UL;


	pUART2->CR1 |= (1 << 0); //USART Enable

	pUART2->CR1 |= (1UL << 3); //TE, RE enable


}


void timer_init(void)
{
	pTIM2->ARR = 0x40740UL;
	pTIM2->CR1 |=  (1 << 7); //ARPE
	pTIM2->CR1 |= (1 << 0); 	//CEN
	pTIM2->DIER |= (1 << 0); //update interrupt


}



