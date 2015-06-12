#include "KalmanJ.h"
#include "HAL_sensor_J.h"

Kalman_t kalmanX; // Create the Kalman instances
Kalman_t kalmanY;

void quat_to_euler(float *ro,float* pit, float* ya ){

	*ya = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)*RAD_TO_DEG;
	*pit = -asin(2*q1*q3 + 2*q0*q2)*RAD_TO_DEG;
	*ro = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1)*RAD_TO_DEG;

}

int sensor_read(sensor_struct_t* angle){
	mpu6050_read(angle->mpu);
}

int sensor_init(sensor_struct_t* angle){
mpu6050_init();
mpu6050_struct_t data;
sensor_mpu6050_read(&data);
sensor_t roll,pitch;
roll  = atan2(data.accY, data.accZ) * RAD_TO_DEG;
pitch = atan(-data.accX / sqrt(data.accY * data.accY + data.accZ * data.accZ)) * RAD_TO_DEG;	
	
KalmanInit(&kalmanX);
KalmanInit(&kalmanY);

	KalmansetAngle(&kalmanX,roll); // Set starting angle
	KalmansetAngle(&kalmanY,pitch);
	angle->gyroXangle = roll;
	angle->gyroYangle = pitch;
	angle->compAngleX = roll;
	angle->compAngleY = pitch;
}


int sensor_filtering(filter_t fil, sensor_struct_t* angle, sensor_t dt){
sensor_t accX = angle->mpu->accX;
	sensor_t accY = angle->mpu->accY;
	sensor_t accZ = angle->mpu->accZ;
	sensor_t compAngleX,compAngleY;
sensor_t square = accY*accY + accZ*accZ;
	if(square<0){
		square=0;
	}
sensor_t roll  = atan2f(accY, accZ) * RAD_TO_DEG;
	sensor_t pitch = atan2f(-accX, sqrt(square))*RAD_TO_DEG;
	//sensor_t gyroXrate = gyroX*250/32768; // Convert to deg/s
	//sensor_t gyroYrate = gyroY*250/32768; // Convert to deg/s
	sensor_t gyroXrate = gyroX/131; // Convert to deg/s
	sensor_t gyroYrate = gyroY/131; // Convert to deg/s
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		KalmansetAngle(&kalmanX,roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	} else
		kalAngleX = KalmangetAngle(&kalmanX,roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = KalmangetAngle(&kalmanY,pitch, gyroYrate, dt);

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;

if(fil==KALMAN){
angle->roll = kalAngleX;
angle->pitch = kalAngleY;
}else if(fil==COMPLEMANTARY){
angle->roll = compAngleX;
angle->pitch = compAngleY;
}
	return 0;
}


