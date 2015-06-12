#ifndef _HAL_SENSOR_J_h_
#define _HAL_SENSOR_J_h_

#include "KalmanJ.h"
#include "mpu6050_J.h"

typedef float sensor_t;

typedef struct sensor_struct_t{
sensor_t roll;
sensor_t pitch;
sensor_t yaw;
mpu6050_struct_t mpu;
}sensor_struct_t;

typedef enum {
KALMAN,
COMPLEMANTARY,
}filter_t;


void quat_to_euler(float *ro,float* pit, float* ya );

int sensor_init(sensor_struct_t* angle); 
int sensor_read(sensor_struct_t* angle); //read mpu data
int sensor_filtering(filter_t fil, sensor_struct_t* angle,float dt); //read filter


#endif
