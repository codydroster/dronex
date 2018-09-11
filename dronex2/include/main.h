#ifndef __MAIN
#define __MAIN

#include "stm32l496xx.h"





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








void system_init(void);

void update_channel_values(void);

void init_ctrl_values(void);



void DMA_init_Xbee(void);







void idle_line_reset_xbee(void);
void idle_line_reset_lidar(void);







#endif
