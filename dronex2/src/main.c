

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "init.h"
#include "stm32l496xx.h"
#include <stdio.h>


#include <stdint.h>
#include "drone_uart.h"
#include "xbee_uart.h"
#include "LSM9DS1.h"
#include "lidar.h"

#include "pindefines.h"
#include "control.h"

#include "math.h"

uint8_t test = 0;






int main(void)
{
	system_init();
	DMA_init_Xbee();
	DMA_init_lidar();
	SPI_DMA_Init();

	drone_uart_init();
	xbee_uart_init();
	lidar_uart_init();
	SPI_init();



	pDMA2C2->CCR |= (1 << 0);	//enable DMA2 Channel 2
	pDMA2C1->CCR |= (1 << 0);	//enable DMA2 Channel 1
	pDMA1C3->CCR |= (1 << 0);	//enable DMA1 channel 3
	pDMASPIRX->CCR |= (1 << 0);	//enable DMA1 channel 4
	pDMASPITX->CCR |= (1 << 0);	//enable DMA1 channel 5


	AG_init();

	//read_imu_mult(STATUS_REG, pSPI2, pGPIOC);


	while(1) {


		update_channel_values();


		//pUART5->TDR = lidar_transmit;		//accel output temp

	}

return 0;
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

	test = 14;
}







/*************************INTERRUPTS******************************/

