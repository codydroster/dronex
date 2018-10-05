#include "LSM9DS1.h"



float acclRaw_x;
float acclRaw_y;
float acclRaw_z;

float angleAccl_x;
float angleAccl_y;
int16_t angleintx;

float sqr;

float altitude;

int16_t gyrRaw_x;
int16_t gyrRaw_y;
int16_t gyrRaw_z;


void sensor_update(void);


void angle_update(void);



void lidar_update(void);
