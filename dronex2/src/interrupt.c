
#include "pindefines.h"
#include "interrupt.h"
#include "main.h"
#include "lidar.h"
#include "LSM9DS1.h"
#include "xbee_uart.h"
#include "drone_uart.h"
#include "stm32l496xx.h"



void SPI2_IRQHandler(void)
{

	spi_receive[spi_index] = (uint32_t) pSPI2->DR;




}



void TIM2_IRQHandler(void)
{
	uart_transmit(pUART2);
pTIM2->SR &= ~(1UL << 0);	//clear interrupt flag

}


void TIM3_IRQHandler(void)
{
		spi_index = 0;
		pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG
		read_imu(OUT_X_L_XL);
		spi_receive[0] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[1] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[2] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[3] = (uint16_t) pSPI2->DR;
	//	spi_receive[4] = pSPI2->DR;
		while((pSPI2->SR >> 7) & 1U);	//while busy
		pGPIOC->ODR |= (1U << cs_ag);

		pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG
		read_imu(OUT_X_L_G);
		spi_receive[5] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[6] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[7] = (uint16_t) pSPI2->DR;
		write_imu(0,0);
		spi_receive[8] = (uint16_t) pSPI2->DR;
		//spi_receive[9] = (uint16_t) pSPI2->DR;
		while((pSPI2->SR >> 7) & 1U);	//while busy
		pGPIOC->ODR |= (1U << cs_ag);





		pTIM3->SR &= ~(1UL << 0);	//clear interrupt flag

}



void DMA2_Channel2_IRQHandler(void)		//xbee rx
{

	pDMA2C2->CCR &= ~(1U << 0);

	if(uart_receive[0] == 0x42 && uart_receive[1] ==0x43){

		throttle_value = (uint16_t) ((uart_receive[2] << 8) | (uart_receive[3] & 0xff));
		roll_value = (uint16_t) ((uart_receive[4] << 8) | (uart_receive[5] & 0xff));
		pitch_value = (uint16_t) ((uart_receive[6] << 8) | (uart_receive[7] & 0xff));
		yaw_value = (uint16_t) ((uart_receive[8] << 8) | (uart_receive[9] & 0xff));
		AUX1_value = (uint16_t) ((uart_receive[10] << 8) | (uart_receive[11] & 0xff));

		throttle_trans = (uint16_t) (0x8000U | (throttle_value + 24U));
		roll_trans = (uint16_t) (0x800U | (roll_value + 24U));
		pitch_trans = (uint16_t) (0x1000U | (pitch_value + 24U));
		yaw_trans = (uint16_t) (0x1800U | (yaw_value + 24U));
		AUX1_trans = (uint16_t) (0x2000U | (AUX1_value + 24U));



	} else {
		for(int i = 0; i < 12; i++) {		//if no char match, clear array
			uart_receive[i] = 0;
		}
	}


	pDMA2->IFCR |= (1 << 5); //transfer complete flag clear
	pDMA2->IFCR |= (1 << 4) | (1 << 6); //global interrupt flag clear

	pDMA2C2->CNDTR = 12U;	//12 bytes	reset
	pDMA2C2->CCR |= (1U << 0);	//enable

}


void DMA2_Channel1_IRQHandler(void)		//xbee TX
{

	pDMA2->IFCR |= (1 << 1); //transfer complete flag clear
	pUART5->ICR |= (1 << 6);		//TC flag clear


}



void DMA1_Channel3_IRQHandler(void)	//LIDAR
{
	if(!(lidar_receive[0] == 0x59)) {
		pDMA1C3->CCR &= ~(1U << 0);
	}
	lidar_transmit = ((lidar_receive[3] << 8) | lidar_receive[2]);

	pDMA1->IFCR |= (1 << 9); //transfer complete flag clear

}











