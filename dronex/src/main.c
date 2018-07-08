

/* Includes ------------------------------------------------------------------*/
#include "stm32l432xx.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"

#include <stdint.h>
#include "drone_uart.h"
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



void TIM2_IRQHandler(void);


uint16_t throttle_trans;
uint16_t throttle_value;

uint16_t yaw_trans;
uint16_t yaw_value;
int main(void)
{



	system_init();
	drone_uart_init(pGPIOA, pUART1);




	throttle_value =  50U;
	throttle_trans = (0x8000UL | (throttle_value + 24UL));

	yaw_value = 1000UL;
	yaw_trans =  (0x1800UL | (yaw_value + 24UL));



////////////////////////////////////////////////////////////////////////////////////////
	transmit_data[0] = 0x0;	// missed frames [1]
	transmit_data[1] = 0x0; // missed frames [2]

	transmit_data[2] = (throttle_trans >> 8UL);	// channel 1	//throttle
	transmit_data[3] = (throttle_trans  & 0xFFUL);

	transmit_data[4] = 0xCUL; // channel 2		//roll
	transmit_data[5] = 0x0;

	transmit_data[6] = 0x14UL; // channel 3		//pitch
	transmit_data[7] = 0x0;

	transmit_data[8] = (yaw_trans >> 8UL); // channel 4		//yaw
	transmit_data[9] = (yaw_trans & 0xFFUL);

	transmit_data[10] = 0xff;
	transmit_data[11] = 0xff;
	transmit_data[12] = 0xff;
	transmit_data[13] = 0xff;
	transmit_data[14] = 0xff;
	transmit_data[15] = 0xff;

	return 0;


}



void TIM2_IRQHandler(void)
{
pTIM2->SR &= ~(1UL << 0);
uart_transmit(pUART1);

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
	__enable_irq();

	//peripheral clock PORTA
	pRCC->AHB2ENR |= (1 << 0);

	//USART1 clock enable
	pRCC->APB2ENR |= (1 << 14);

	//TIM2 CLK
	pRCC->APB1ENR1 |= (1 << 0);



	timer_init(pTIM2);




}


