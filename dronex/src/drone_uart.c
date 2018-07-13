/*
 * drone_uart.c
 *
 *  Created on: Jun 23, 2018
 *      Author: codydroster
 */

#include "drone_uart.h"


void timer_init(TIM_TypeDef *pTIM)
{
	pTIM->ARR = 0x40740UL;
	pTIM->CR1 |=  (1 << 7); //ARPE
	pTIM->CR1 |= (1 << 0); 	//CEN
	pTIM->DIER |= (1 << 0); //update interrupt


}

void drone_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART)
{
	//tx init (A09)
	pGPIO->MODER &= ~(0x3UL << (TX_PIN * 2));
	pGPIO->MODER |= (2U << (TX_PIN * 2));

	//rx init(A10)
	pGPIO->MODER &= ~(0x3UL << (RX_PIN * 2));
	pGPIO->MODER |= (2UL << (RX_PIN * 2));


	pGPIO->PUPDR |= (1UL << 18) | (1UL << 20);	//pull up
	pGPIO->OSPEEDR |= (3UL << 20) | (3UL << 18); //speed

	//alternate function setup
	pGPIO->AFR[1] |= (0x7UL << 4) | (0x7UL << 8);


	//USART initialize
	pUART->CR1 = (0UL << 12) | (0UL << 28); //word length

	//BAUD rate
	pUART->BRR = 0xD0UL;

	//pUART->CR2 |= (1 <<19);	//MSB
	pUART->CR1 |= (1 << 0); //USART Enable

	pUART->CR1 |= (1UL << 3); //TE, RE enable








}

void uart_transmit(USART_TypeDef *pUART)
{

	for(int i = 0; i < 16; i++){
		pUART->TDR = transmit_data[i];
		while(!(pUART->ISR & (1 << 7)));	//transmit data register flag


	}



}






