#include <stdint.h>
#include "stm32l496xx.h"




uint16_t mag_z_val;



int16_t spi_receive[10];

uint16_t spi_transmit[8];



void SPI_init(void);



void AG_init(void);
void AG_read(void);

void timer_init3(void);
void timer_init4(void);

void write_imu(uint8_t address, uint8_t data);
void write_imu_init(uint8_t address, uint8_t data);


void read_imu(uint8_t address);
void read_imu_mult(uint8_t address);

void AG_init(void);



//accelerometer & gyro registers

#define ACT_THS               0x04
#define ACT_DUR               0x05
#define INT_GEN_CFG_XL        0x06
#define INT_GEN_THS_X_XL      0x07
#define INT_GEN_THS_Y_XL      0x08
#define INT_GEN_THS_Z_XL      0x09
#define INT_GEN_DUR_XL        0x0A
#define REFERENCE_G           0x0B
#define INT1_CTRL             0x0C
#define INT2_CTRL             0x0D

#define WHO_AM_I              0x0F
#define CTRL_REG1_G           0x10
#define CTRL_REG2_G           0x11
#define CTRL_REG3_G           0x12
#define ORIENT_CFG_G          0x13
#define INT_GEN_SRC_G         0x14
#define OUT_TEMP_L            0x15
#define OUT_TEMP_H            0x16
#define STATUS_REG0           0x17
#define OUT_X_L_G             0x18 // gyro register start
#define OUT_X_H_G             0x19 //
#define OUT_Y_L_G             0x1A //
#define OUT_Y_H_G             0x1B //
#define OUT_Z_L_G             0x1C //
#define OUT_Z_H_G             0x1D // gyro register end
#define CTRL_REG4             0x1E
#define CTRL_REG5_XL          0x1F
#define CTRL_REG6_XL          0x20
#define CTRL_REG7_XL          0x21
#define CTRL_REG8             0x22
#define CTRL_REG9             0x23
#define CTRL_REG10            0x24

#define INT_GEN_SRC_XL        0x26
#define STATUS_REG            0x27
#define OUT_X_L_XL            0x28 // accel register start
#define OUT_X_H_XL            0x29 //
#define OUT_Y_L_XL            0x2A //
#define OUT_Y_H_XL            0x2B //
#define OUT_Z_L_XL            0x2C //
#define OUT_Z_H_XL            0x2D // accel register end
#define FIFO_CTRL             0x2E
#define FIFO_SRC              0x2F
#define INT_GEN_CFG_G         0x30
#define INT_GEN_THS_XH_G      0x31
#define INT_GEN_THS_XL_G      0x32
#define INT_GEN_THS_YH_G      0x33
#define INT_GEN_THS_YL_G      0x34
#define INT_GEN_THS_ZH_G      0x35
#define INT_GEN_THS_ZL_G      0x36
#define INT_GEN_DUR_G         0x37


//magnetometer registers

#define OFFSET_X_REG_L_M      0x05
#define OFFSET_X_REG_H_M      0x06
#define OFFSET_Y_REG_L_M      0x07
#define OFFSET_Y_REG_H_M      0x08
#define OFFSET_Z_REG_L_M      0x09
#define OFFSET_Z_REG_H_M      0x0A

#define WHO_AM_I_M            0F

#define CTRL_REG1_M           0x20
#define CTRL_REG2_M           0x21
#define CTRL_REG3_M           0x22
#define CTRL_REG4_M           0x23
#define CTRL_REG5_M           0x24

#define STATUS_REG_M          0x27
#define OUT_X_L_M             0x28
#define OUT_X_H_M             0x29
#define OUT_Y_L_M             0x2A
#define OUT_Y_H_M             0x2B
#define OUT_Z_L_M             0x2C
#define OUT_Z_H_M             0x2D

#define INT_CFG_M             0x30
#define INT_SRC_M             0x31
#define INT_THS_L_M           0x32
#define INT_THS_H_M           0x33




