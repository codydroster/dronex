#include "init.h"
#include "LSM9DS1.h"
#include "lidar.h"
#include "fc.h"
#include "math.h"
//#include "arm_math.h"



void sensor_update(void)
{
acclRaw_x = (int16_t) spi_receive[1];
acclRaw_y = (int16_t) spi_receive[2];
acclRaw_z = (int16_t) spi_receive[3];



//  245Deg/Sec   8.75millidegrees/LSB

/*****would like to know why x and y are swithed*********/
pTIM4->CNT = 0;
gyrY = ((float) spi_receive[6]) / 87.5;	//convert to degrees/second
gyrX = ((float) spi_receive[7]) / 87.5;
gyrZ = ((float) spi_receive[8]) / 87.5;



}


void angle_update(void)
{



angleAccl_x = atan2f(acclRaw_x, sqrtf(acclRaw_y*acclRaw_y + acclRaw_z*acclRaw_z)) * 57.3;	//degrees
angleAccl_y = atan2f(acclRaw_y, sqrtf(acclRaw_x*acclRaw_x + acclRaw_z*acclRaw_z)) * 57.3;	//degrees

angleintX = (int16_t) angleAccl_x;
angleintY = (int16_t) angleAccl_y;

// angle = .98 * (angle + gyrData * dt) + .02 * accData
//complementary angle in degrees
dtTim4 = ((float) pTIM4->CNT) / 6000000.0f;	//tim since gyro read in seconds
compX = (float) (.98f * (compX + gyrX * dtTim4) + .02 * angleAccl_x);
compY = (float) (.98f * (compY - gyrY * dtTim4) + .02 * angleAccl_y);



}


void lidar_update(void)
{
	altitude = sqrtf((lidar_transmit * cosf(angleAccl_x))*(lidar_transmit * cosf(angleAccl_x)) +
				(lidar_transmit * cosf(angleAccl_y))*(lidar_transmit * cosf(angleAccl_y)));

}
