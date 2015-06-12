#ifndef _HAL_PID_J_h_
#define _HAL_PID_J_h_


/* MOTOR control */
#include "HAL_sensor_J.h"
#define RAD_TO_DEG  57.295779f
#define DEG_TO_RAD	0.0174532925f

#define WEIGHT 0.7
typedef float motor_t;


#define MAX_SYSTEM 3
typedef enum {
ROLL,
PITCH,
YAW,

}system_t;

typedef struct pid_control_t{
	motor_t Kp;
	motor_t Ki;
	motor_t Kd;
	motor_t err;
	motor_t errD;
	motor_t errI;
//	motor_t err_prev;
//	motor_t errD_prev;
}pid_control_t;

void pid_init(pid_control_t *p);
void pid_setErr(pid_control_t *p, motor_t val);
void pid_setErrD(pid_control_t *p, sensor_t val,sensor_t dt);
void pid_setErrI(pid_control_t *p, motor_t dt);


int pid_control(pid_control_t* control);

uint32_t pid_feedforward(int m_val);

#endif
