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
	pSPI2->CR2 |= (1 << 6);	//RXNEIE interrupt


	pSPI2->CR1 |= (1 << 6);	//SPI enable

}


void write_imu(uint8_t address, uint8_t data)
{

	pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG
	pSPI2->DR = (uint16_t) ((address << 8) | data);

	while((pSPI2->SR >> 7) & 1U);	//while busy
	pGPIOC->ODR |= (1U << cs_ag);


}

void read_imu(uint8_t address)
{

	pGPIOC->ODR &= ~(1U << cs_ag);	//SS AG

	pSPI2->DR = (uint16_t) (0x8000 | (address << 8));

	while((pSPI2->SR >> 7) & 1U);	//while busy
	pGPIOC->ODR |= (1U << cs_ag);


}




void AG_init(void)
{
		read_imu(WHO_AM_I);

		write_imu(CTRL_REG1_G,	0xC0);
		write_imu(CTRL_REG2_G,	0x00);
		write_imu(CTRL_REG3_G, 	0x00);
		write_imu(CTRL_REG4,	0x38);
		write_imu(CTRL_REG5_XL,	0x38);
		write_imu(CTRL_REG6_XL, 0x00);
		write_imu(CTRL_REG7_XL, 0x00);
		write_imu(CTRL_REG8,	0x04);


}

void timer_init3(void)
{
		pTIM3->ARR = 0x74000UL;
		pTIM3->CR1 |=  (1 << 7); //ARPE
		pTIM3->CR1 |= (1 << 0); 	//CEN
		pTIM3->DIER |= (1 << 0); //update interrupt


}

