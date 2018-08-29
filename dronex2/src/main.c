

/* Includes ------------------------------------------------------------------*/
#include "stm32l496xx.h"
#include <stdio.h>


#include <stdint.h>
#include "drone_uart.h"
#include "xbee_uart.h"
#include "LSM9DS1.h"
#include "lidar.h"
#include "main.h"
#include "pindefines.h"

#include "math.h"

uint8_t test = 0;

//initialize port A
GPIO_TypeDef *pGPIOA = GPIOA;

//port B
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
USART_TypeDef *pUART5 = UART5;

//UART3 init - Lidar
USART_TypeDef *pUARTLID = USART3;

//TIM2 typedef
TIM_TypeDef *pTIM2 = TIM2;

//SPI typedef
SPI_TypeDef *pSPI2 = SPI2;



//DMA
DMA_TypeDef *pDMA1 = DMA1;
DMA_TypeDef *pDMA2 = DMA2;


//DMA1
DMA_Channel_TypeDef *pDMA1C3 = DMA1_Channel3;
DMA_Request_TypeDef *pDMA1SEL = DMA1_CSELR;


//DMA2
DMA_Channel_TypeDef *pDMA2C2 = DMA2_Channel2;
DMA_Channel_TypeDef *pDMA2C1 = DMA2_Channel1;
DMA_Request_TypeDef *pDMA2SEL = DMA2_CSELR;




//Values transmitted to FC
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




uint32_t spi_receive[4];
uint8_t spi_index;



int main(void)
{
	system_init();
	DMA_init();
	DMA_init_lidar();

	drone_uart_init(pGPIOA, pUART2);
	xbee_uart_init(pGPIOD, pGPIOC, pUART5);
	lidar_uart_init(pGPIOC, pGPIOC, pUARTLID);
	SPI_init(pGPIOB, pGPIOC, pSPI2);
	pDMA2C2->CCR |= (1 << 0);	//enable DMA Channel 2
	pDMA2C1->CCR |= (1 << 0);	//enable DMA Channel 1
	pDMA1C3->CCR |= (1 << 0);	//enable DMA channel 3


	AG_init();

	//read_imu_mult(STATUS_REG, pSPI2, pGPIOC);


	while(1) {

		idle_line_reset_xbee();
		if(!(pDMA1C3->CCR & 1U)){
		//idle_line_reset_lidar();
		}
		update_channel_values();
		spi_index = 0;
		read_imu_mult(STATUS_REG, pSPI2, pGPIOC);
		//pUARTLID->TDR = (spi_receive[1] & 0xff);		//accel output temp

	}

return 0;
}







//trys to reset DMA to initial values if UART out of sync
void idle_line_reset_xbee(void)
{
	if((pUART5->ISR >> 4) & 1U) {					//if idle line
		pDMA2C2->CCR &= ~(1U << 0);					//disable DMA Channel 2

		pDMA2C2->CMAR = (uint32_t) &uart_receive;	//reset address
		pDMA2C2->CNDTR = 12U;						//reset DMA counter
		pDMA2C2->CCR |= (1 << 0);					//enable DMA Channel 2
		pUART5->ICR |= (1U << 4);					//clear flag idle


		}
}


void idle_line_reset_lidar(void)
{
	if((pUARTLID->ISR >> 4) & 1U) {					//if idle line
		//pDMA1C3->CCR &= ~(1U << 0);		//disable DMA Channel 2

		pDMA1C3->CMAR = (uint32_t) &lidar_receive;	//reset address
		pDMA1C3->CNDTR = 9U;						//reset DMA counter
		pDMA1C3->CCR |= (1 << 0);					//enable DMA Channel 2
		pUARTLID->ICR |= (1U << 4);			//clear flag idle


		}
}





void DMA_init(void)
{

	//channel 2
	pDMA2C2->CPAR = (uint32_t) &pUART5->RDR;
	pDMA2C2->CMAR = (uint32_t) &uart_receive;

	pDMA2C2->CNDTR = 12U;	//12 bytes
	pDMA2SEL->CSELR |= (2U << 4);	//channel selection
	pDMA2C2->CCR |= (1 << 7);	//memory increment
	//pDMA2C2->CCR |= (1 << 5);	//circular mode
	pDMA2C2->CCR |= (1 << 1); 	//transfer complete interrupt enable


	//channel 1
	pDMA2C1->CPAR = (uint32_t) &pUART5->TDR;
	pDMA2C1->CMAR = (uint32_t) &lidar_transmit;

	pDMA2C1->CCR |= (1 << 4);	//DIR: mem to per
	pDMA2C1->CNDTR = 1U;	//1 bytes
	pDMA2C1->CCR |= (1 << 5);	//circular mode
	pDMA2SEL->CSELR |= (2U << 0);	//channel selection
	pDMA2C1->CCR |= (1 << 1); 	//transfer complete interrupt enable

	pUART5->CR1 |= (1 << 3);		//UART TX ENABLE
	pUART5->ICR |= (1 << 6);		//TC flag clear
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



void AG_init(void)
{
		read_imu(WHO_AM_I, pSPI2, pGPIOC);

		write_imu(CTRL_REG1_G,	0xC0, pSPI2, pGPIOC);
		write_imu(CTRL_REG2_G,	0x00, pSPI2, pGPIOC);
		write_imu(CTRL_REG3_G, 	0x00, pSPI2, pGPIOC);
		write_imu(CTRL_REG4,	0x38, pSPI2, pGPIOC);
		write_imu(CTRL_REG5_XL,	0x38, pSPI2, pGPIOC);
		write_imu(CTRL_REG6_XL, 0x00, pSPI2, pGPIOC);
		write_imu(CTRL_REG7_XL, 0x00, pSPI2, pGPIOC);
		write_imu(CTRL_REG8,	0x04, pSPI2, pGPIOC);


}






void system_init(void)
{

		while(!((pRCC->CR >> 1) & 1U));	//Wait until MSI oscillator is stable.

		//Frequency Setup
		pRCC->CR &= ~(0xfUL << 4);	//clear MSI
		pRCC->CR |= (0x9UL << 4);	//MSI range	24MHz
		//pRCC->CCIPR |= (1 << 2) | (1 << 8);	//UART clock select //sys clock
		pRCC->CFGR |= (2UL << 24);	//MSI Clock
		pRCC->CR |= (1 << 3);	//MSI range in CR

		//interrupts
		NVIC_EnableIRQ(TIM2_IRQn);			//fc transmit
		NVIC_EnableIRQ(DMA2_Channel2_IRQn);
		NVIC_EnableIRQ(DMA2_Channel1_IRQn);
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);
		NVIC_EnableIRQ(SPI2_IRQn);

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
		//DMA2
		pRCC->AHB1ENR |= (1 << 1);
		//DMA1
		pRCC->AHB1ENR |= (1 << 0);
		//SPI2 clock enable
		pRCC->APB1ENR1 |= (1 << 14);




		timer_init(pTIM2);

}


//Array  to be transmitted to FC, Spectrum 2048 protocol.
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







/*************************INTERRUPTS******************************/

void SPI2_IRQHandler(void)
{
	if(spi_index > 12) {
		spi_index = 0;
	}
	spi_receive[spi_index] = pSPI2->DR;
	spi_index++;

}



void TIM2_IRQHandler(void)
{
	uart_transmit(pUART2);
pTIM2->SR &= ~(1UL << 0);	//clear interrupt flag

}




void DMA2_Channel2_IRQHandler(void)		//xbee rx
{

	if(uart_receive[0] == 0x42 && uart_receive[1] ==0x43){

		throttle_value = (uint16_t) ((uart_receive[2] << 8) | (uart_receive[3] & 0xff));
		roll_value = (uint16_t) ((uart_receive[4] << 8) | (uart_receive[5] & 0xff));
		pitch_value = (uint16_t) ((uart_receive[6] << 8) | (uart_receive[7] & 0xff));
		yaw_value = (uint16_t) ((uart_receive[8] << 8) | (uart_receive[9] & 0xff));
		AUX1_value = (uint16_t) ((uart_receive[10] << 8) | (uart_receive[11] & 0xff));

	}

	throttle_trans = (uint16_t) (0x8000U | (throttle_value + 24U));
	roll_trans = (uint16_t) (0x800U | (roll_value + 24U));
	pitch_trans = (uint16_t) (0x1000U | (pitch_value + 24U));
	yaw_trans = (uint16_t) (0x1800U | (yaw_value + 24U));
	AUX1_trans = (uint16_t) (0x2000U | (AUX1_value + 24U));


	pDMA2->IFCR |= (1 << 5); //transfer complete flag clear
	//DMA off *reset with idle line reset. //if(dma = on) {do not reset}

}


void DMA2_Channel1_IRQHandler(void)		//xbee TX
{

	pDMA2->IFCR |= (1 << 1); //transfer complete flag clear
	pUART5->ICR |= (1 << 6);		//TC flag clear


}



void DMA1_Channel3_IRQHandler(void)	//LIDAR
{

	lidar_transmit = lidar_receive[2];

	pDMA1->IFCR |= (1 << 9); //transfer complete flag clear

	test++;

}

