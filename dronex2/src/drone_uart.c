/*
 * drone_uart.c
 *
 *  Created on: Jun 23, 2018
 *      Author: codydroster
 */

#include "drone_uart.h"
#include "pindefines.h"


void timer_init(TIM_TypeDef *pTIM)
{
	pTIM->ARR = 0x40740UL;
	pTIM->CR1 |=  (1 << 7); //ARPE
	pTIM->CR1 |= (1 << 0); 	//CEN
	pTIM->DIER |= (1 << 0); //update interrupt


}

void drone_uart_init(GPIO_TypeDef *pGPIO, USART_TypeDef *pUART)
{
	//tx init (PA2)
	pGPIO->MODER &= ~(3UL << (FC_TX * 2));
	pGPIO->MODER |= (2U << (FC_TX * 2));

	//rx init(PA3)
	pGPIO->MODER &= ~(3UL << (FC_RX * 2));
	pGPIO->MODER |= (2UL << (FC_RX * 2));

	//alternate function setup
	pGPIO->AFR[0] |= (7UL << (FC_RX * 4)) | (7UL << (FC_TX * 4));

	pGPIO->PUPDR |= (1UL << (FC_RX * 2)) | (1UL << (FC_TX * 2));	//pull up
	pGPIO->OSPEEDR |= (3UL << (FC_RX * 2)) | (3UL << (FC_TX *2)); //speed




	//USART initialize
	pUART->CR1 = (0UL << 12) | (0UL << 28); //word length

	//BAUD rate
	pUART->BRR = 0xD0UL;


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









