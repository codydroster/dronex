

/* Includes ------------------------------------------------------------------*/
#include "stm32l432xx.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"

#include <stdint.h>
#include "drone_uart.h"
#include "xbee_uart.h"
#include "main.h"

#define BASE_ADDRESS 0X20000000

char const my_data[] = "hello world";

//initialize port A
GPIO_TypeDef *pGPIOA = GPIOA;

//initialize clock
RCC_TypeDef *pRCC = RCC;

//UART init
USART_TypeDef *pUART1 = USART1;

//UART2 init
USART_TypeDef *pUART2 = USART2;

//TIM2 typedef
TIM_TypeDef *pTIM2 = TIM2;

//DMA
DMA_Channel_TypeDef *pDMA1C6 = DMA1_Channel6;
DMA_TypeDef *pDMA1 = DMA1;
DMA_Request_TypeDef *pDMA1SEL = DMA1_CSELR;



uint16_t throttle_trans;
uint16_t throttle_value;

uint16_t roll_trans;
uint16_t roll_value;

uint16_t pitch_trans;
uint16_t pitch_value;

uint16_t yaw_trans;
uint16_t yaw_value;

uint16_t AUX1_trans;
uint16_t AUX1_value;


volatile uint16_t uart_recieve;

int main(void)
{
	system_init();
	DMA_init();
	drone_uart_init(pGPIOA, pUART1);
	xbee_uart_init(pGPIOA, pUART2);


	//GPIO output pin.
	pGPIOA->MODER &= ~(3U << 0); //clear
	pGPIOA->MODER |= (1U << 0);	//output

	//pGPIOA->ODR |= (1U << 0);
	pUART2->CR3 |= (1 << 6);

while(1) {

	update_channel_values();



	}

return 0;
}



void TIM2_IRQHandler(void)
{
pTIM2->SR &= ~(1UL << 0);	//clear interrupt flag
uart_transmit(pUART1);

}




//interrupt after uart DMA reads in 2 bytes to uart_recieve.
void DMA1_Channel6_IRQHandler(void)
{
		if(((uart_recieve >> 12) & 0xF) == 1) {
			throttle_value =  (uart_recieve & 0xFFF);
			throttle_trans = (uint16_t) (0x8000U | (throttle_value + 24U));
			} else


		if(((uart_recieve >> 12) & 0xF) == 2) {
			roll_value = (uart_recieve & 0xFFF);
			roll_trans = (uint16_t) (0x800U | (roll_value + 24U));
			} else

		if(((uart_recieve >> 12) & 0xF) == 3) {
			pitch_value = (uart_recieve & 0xFFF);
			pitch_trans = (uint16_t) (0x1000U | (pitch_value + 24U));
			} else

		if(((uart_recieve >> 12) & 0xF) == 4) {
			yaw_value = (uart_recieve & 0XFFF);
			yaw_trans = (uint16_t) (0x1800U | (yaw_value + 24U));
			} else

		if(((uart_recieve >> 12) & 0xF) == 5) {
			AUX1_value = (uart_recieve & 0XFFF);
			AUX1_trans = (uint16_t) (0x2000U | (AUX1_value + 24U));
			}
		pDMA1->IFCR |= (1 << 21);


}


void DMA_init(void)
{


	pDMA1C6->CPAR = (uint32_t) &pUART2->RDR;
	pDMA1C6->CMAR = (uint32_t) &uart_recieve;

	pDMA1C6->CNDTR = 2U;	//2 bytes
	pDMA1SEL->CSELR |= (2U << 20);	//channel selection
	pDMA1C6->CCR |= (1 << 7);	//memory increment
	pDMA1C6->CCR |= (1 << 5);	//circular mode
	pDMA1C6->CCR |= (1 << 1); 	//transfer complete interrupt enable
	pDMA1C6->CCR |= (1 << 0); 	//EN


}

void system_init(void)
{

	while(!(pRCC->CR & (1 << 1)));

	//Frequency Setup
	pRCC->CR &= ~(0xfUL << 4);	//clear MSI
	pRCC->CR |= (0x9UL << 4);	//MSI range	24MHz
	pRCC->CCIPR |= (1 << 0);	//UART clock select //sys clock
	pRCC->CFGR |= (2UL << 24);	//MSI Clock
	pRCC->CR |= (1 << 3);	//MSI range in CR

	//interrupts
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	__enable_irq();

	//peripheral clock PORTA
	pRCC->AHB2ENR |= (1 << 0);

	//USART1 clock enable
	pRCC->APB2ENR |= (1 << 14);
	//USART2 clock enable
	pRCC->APB1ENR1 |= (1 << 17);

	//TIM2 CLK
	pRCC->APB1ENR1 |= (1 << 0);

	pRCC->AHB1ENR |= (1 << 0);


	timer_init(pTIM2);

}



void update_channel_values(void)
{

	transmit_data[0] = 0x0;	// missed frames [1]
	transmit_data[1] = 0x0; // missed frames [2]

	transmit_data[2] = (uint8_t) (throttle_trans >> 8UL);	// channel 1	//throttle
	transmit_data[3] = (throttle_trans  & 0xFFUL);

	transmit_data[4] = (uint8_t) (roll_trans >> 8UL); // channel 2		//roll
	transmit_data[5] = (roll_trans & 0xFFUL);

	transmit_data[6] = (uint8_t) (pitch_trans >> 8UL); // channel 3		//pitch
	transmit_data[7] = (pitch_trans & 0xFFUL);

	transmit_data[8] = (uint8_t) (yaw_trans >> 8UL); // channel 4		//yaw
	transmit_data[9] = (yaw_trans & 0xFFUL);

	transmit_data[10] = (uint8_t) (AUX1_trans >> 8UL); // channel 5 AUX1
	transmit_data[11] = (AUX1_trans & 0xFFUL);

	transmit_data[12] = 0xff;
	transmit_data[13] = 0xff;
	transmit_data[14] = 0xff;
	transmit_data[15] = 0xff;




}




