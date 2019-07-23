#include "pindefines.h"
#include "init.h"
#include <stdint.h>
#include "xbee_uart.h"
#include "drone_uart.h"
#include "lidar.h"





GPIO_TypeDef *pGPIOA = GPIOA;

GPIO_TypeDef *pGPIOB = GPIOB;

//port C
GPIO_TypeDef *pGPIOC = GPIOC;

//port D
GPIO_TypeDef *pGPIOD = GPIOD;

//initialize clock
RCC_TypeDef *pRCC = RCC;

//UART init - FC
USART_TypeDef *pUART2 = USART2;

//UART2 init - Xbee
USART_TypeDef *pUART5 = USART1;

//UART3 init - Lidar
USART_TypeDef *pUARTLID = USART3;

//TIM2 typedef
TIM_TypeDef *pTIM2 = TIM2;

//TIM3 typedef
TIM_TypeDef *pTIM3 = TIM3;

//TIM4 typedef
TIM_TypeDef *pTIM4 = TIM4;



//SPI typedef
SPI_TypeDef *pSPI2 = SPI2;

//DMA
DMA_TypeDef *pDMA1 = DMA1;
DMA_TypeDef *pDMA2 = DMA2;





//DMA1
DMA_Channel_TypeDef *pDMA1C3 = DMA1_Channel3;

//DMA_Channel_TypeDef *pDMASPIRX = DMA1_Channel4;
//DMA_Channel_TypeDef *pDMASPITX = DMA1_Channel5;

//DMA_Request_TypeDef *pDMA1SEL = DMA1_CSELR;




//DMA2
DMA_Channel_TypeDef *pDMA2C2 = DMA2_Channel2;
DMA_Channel_TypeDef *pDMA2C1 = DMA2_Channel1;

//DMA_Request_TypeDef *pDMA2SEL = DMA2_CSELR;






void system_init(void)
{

		while(!((pRCC->CR >> 1) & 1U));	//Wait until MSI oscillator is stable.

		//Frequency Setup
		pRCC->CR &= ~(0xfUL << 4);	//clear MSI
		pRCC->CR |= (0x9UL << 4);	//MSI range	24MHz
		//pRCC->CCIPR |= (1 << 2) | (1 << 8);	//UART clock select //sys clock
		pRCC->CFGR |= (2UL << 24);	//MSI Clock
		pRCC->CR |= (1 << 3);	//MSI range in CR

		//interrupts	//nvic->ISER
		NVIC_EnableIRQ(TIM2_IRQn);			//fc transmit
		NVIC_EnableIRQ(DMA2_Channel2_IRQn);
		NVIC_EnableIRQ(DMA2_Channel1_IRQn);
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);
		NVIC_EnableIRQ(TIM3_IRQn);
	//	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	//	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	//	NVIC_EnableIRQ(SPI2_IRQn);
		//NVIC_EnableIRQ(UART5_IRQn);
		//NVIC_EnableIRQ(USART3_IRQn);


		NVIC_SetPriority(DMA2_Channel2_IRQn, 0);
		NVIC_SetPriority(SPI2_IRQn, 1);

		__enable_irq();

		//peripheral clock PORTA
		pRCC->AHB2ENR |= (1 << 0);
		//clock PORTB
		pRCC->AHB2ENR |= (1 << 1);
		//clock PORTC
		pRCC->AHB2ENR |= (1 << 2);
		//clock PORTD
		pRCC->AHB2ENR |= (1 << 3);
		//USART2 clock enable
		pRCC->APB1ENR1 |= (1 << 17);
		//USART3 clock enable
		pRCC->APB1ENR1 |= (1 << 18);
		//USART5 clock enable
		pRCC->APB1ENR1 |= (1 << 20);
		//TIM2 CLK
		pRCC->APB1ENR1 |= (1 << 0);
		//TIM3
		pRCC->APB1ENR1 |= (1 << 1);
		//TIM4
		pRCC->APB1ENR1 |= (1 << 2);

		//DMA2
		pRCC->AHB1ENR |= (1 << 1);
		//DMA1
		pRCC->AHB1ENR |= (1 << 0);
		//SPI2 clock enable
		pRCC->APB1ENR1 |= (1 << 14);


		timer_init2();

}





