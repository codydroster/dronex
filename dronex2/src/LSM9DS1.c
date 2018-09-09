/*
 * LSM9DS1.c
 *
 *  Created on: Jul 17, 2018
 *      Author: codydroster
 */


#include "LSM9DS1.h"
#include "init.h"
#include "pindefines.h"

uint16_t spi_init_TXbuffer[] =
{
	//(0x8000 | (WHO_AM_I << 8)),
	((CTRL_REG1_G << 8) | 0xC0),
	(CTRL_REG2_G << 8),
	(CTRL_REG3_G << 8),
	((CTRL_REG4 << 8) | 0x38),
	((CTRL_REG5_XL << 8) | 0x38),
	(CTRL_REG6_XL << 8),
	(CTRL_REG7_XL << 8),
	((CTRL_REG8 << 8) | 0x04),
	//(0x8000 | (WHO_AM_I << 8))
};


uint16_t spi_init_RXbuffer[20];


uint16_t spi_transmit[] =
{
		(0x8000 | (OUT_X_L_XL << 8)),
		0,
		0,
		0,
		(0x8000 | (OUT_X_L_G << 8)),
		0,
		0,
		0
};


void SPI_init(void)
{


	pGPIOB->MODER &= ~(3U << (imu_sck*2));
	pGPIOB->MODER &= ~(3U << (imu_miso*2));
	pGPIOB->MODER &= ~(3U << (imu_mosi*2));

	pGPIOB->MODER |= (2U << (imu_sck*2));		//set alternate mode sck
	pGPIOB->MODER |= (2U << (imu_miso*2));		//set alternate mode miso
	pGPIOB->MODER |= (2U << (imu_mosi*2));		//set alternate mode mosi


	pGPIOB->OSPEEDR |= (2U << (imu_sck*2)); //speed HIGH sck


	pGPIOB->OSPEEDR |= (2U << (imu_miso*2)); //speed HIGH miso



	pGPIOB->OSPEEDR |= (2U << (imu_mosi*2)); //speed HIGH mosi



	pGPIOB->AFR[1] |= (5U << ((imu_sck - 8)* 4));	//alternate function 5: sck
	pGPIOB->AFR[1] |= (5U << ((imu_miso - 8)* 4));	//alternate function 5: sck
	pGPIOB->AFR[1] |= (5U << ((imu_mosi - 8)* 4));	//alternate function 5: sck

	pGPIOC->MODER &= ~(3U << (cs_ag*2));
	pGPIOC->MODER |=  (1U << (cs_ag*2));
	pGPIOC->ODR |= (1 << cs_ag);

	//SPI setup
	pSPI2->CR1 |= (7U << 3);	//BRR 3MHz
	pSPI2->CR1 |= (1 << 1); 	//CK 1 when idle
	pSPI2->CR1 |= (1 << 2);	//Master Mode

	pSPI2->CR2 |= (15U << 8); //DS
	pSPI2->CR2 |= (1 << 2);	//SSOE
	pSPI2->CR2 |= (1 << 12);	//FRXTH //1/4 fifo
	pSPI2->CR1 |= (1 << 0);	//clock phase
	pSPI2->CR2 |= (1 << 6);	//TXNEIE interrupt
	pSPI2->CR2 |= (1 << 3); //nss
	pSPI2->CR2 |= (1 << 1) | (1 << 0); //dma enable rx, tx

//	pSPI2->CR1 |= (1 << 6);	//SPI enable

}


void AG_init(void)
{

	pDMASPIRX->CCR |= (1 << 0);	//enable DMA1 channel 4
	pDMASPITX->CCR |= (1 << 0);	//enable DMA1 channel 5

	pSPI2->CR1 |= (1 << 6);	//SPI enable
	pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG


}

void AG_read(void)
{
	pDMASPITX->CMAR = (uint32_t) &spi_transmit;
	pDMASPIRX->CCR |= (1 << 0);	//enable DMA1 channel 4
	pDMASPITX->CCR |= (1 << 0);	//enable DMA1 channel 5
	pSPI2->CR1 |= (1U << 6);
	pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG
}







void SPI_DMA_Init(void)
{
	//SPI RX - channel 4
	pDMASPIRX->CPAR = (uint32_t) &pSPI2->DR;
	pDMASPIRX->CMAR = (uint32_t) &spi_receive;

	pDMASPIRX->CNDTR = 14U;	//num bytes
	pDMA1SEL->CSELR |= (1U << 12U);	//channel selection
	pDMASPIRX->CCR |= (1 << 7);	//memory increment
	//pDMASPIRX->CCR |= (1 << 5);	//circular mode								/****disable at start****/
	pDMASPIRX->CCR |= (1 << 1); 	//transfer complete interrupt enable
	//pDMASPITX->CCR |= (1 << 10);




	//SPI TX - channel 5
	pDMASPITX->CPAR = (uint32_t) &pSPI2->DR;
	pDMASPITX->CMAR = (uint32_t) &spi_init_TXbuffer;
	pDMASPITX->CCR |= (1 << 4);						//DIR: mem to per


	//pDMASPITX->CCR |= (1 << 10);
	pDMASPITX->CNDTR = 16U;	//num bytes
	pDMA1SEL->CSELR |= (1U << 16U);	//channel selection
	pDMASPITX->CCR |= (1 << 7);	//memory increment
	//pDMASPITX->CCR |= (1 << 5);	//circular mode								/****disable at start****/
	pDMASPITX->CCR |= (1 << 1); 	//transfer complete interrupt enable
	//pDMASPITX->CCR |= (1 << 0);	//enable DMA1 channel 5



}
