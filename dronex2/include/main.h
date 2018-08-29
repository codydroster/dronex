#ifndef __MAIN
#define __MAIN

#include "stm32l496xx.h"









void system_init(void);

void update_channel_values(void);

void init_ctrl_values(void);

void TIM2_IRQHandler(void);

void DMA_init(void);

void DMA2_Channel2_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

void SPI2_IRQHandler(void);

void AG_init(void);


void idle_line_reset_xbee(void);
void idle_line_reset_lidar(void);


void DMA_init_lidar(void);




#endif
