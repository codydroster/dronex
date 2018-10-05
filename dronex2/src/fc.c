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

gyrRaw_x = (int16_t) spi_receive[6];
gyrRaw_y = (int16_t) spi_receive[7];
gyrRaw_z = (int16_t) spi_receive[8];


}


void angle_update(void)
{



angleAccl_x = atan2f(acclRaw_x, sqrtf(acclRaw_y*acclRaw_y + acclRaw_z*acclRaw_z)) * 57.3;
angleAccl_y = atan2f(acclRaw_y, sqrtf(acclRaw_x*acclRaw_x + acclRaw_z*acclRaw_z)) * 57.3;

angleintx = (int16_t) angleAccl_x;
}


void lidar_update(void)
{
	altitude = sqrtf((lidar_transmit * cosf(angleAccl_x))*(lidar_transmit * cosf(angleAccl_x)) +
				(lidar_transmit * cosf(angleAccl_y))*(lidar_transmit * cosf(angleAccl_y)));

}
