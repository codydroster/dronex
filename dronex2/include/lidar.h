

#include "init.h"
#include "stm32g431xx.h"
#include <stdint.h>



volatile uint8_t lidar_receive[8];

uint16_t lidar_transmit;

//void lidar_uart_init(void);

void DMA_init_lidar(void);

void lidar_uart_init(void);
