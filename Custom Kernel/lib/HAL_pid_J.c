

#include "HAL_pid_J.h"
#include "HAL_motor_J.h"
//#include "HAL_sensor_J.h"

void pid_init(pid_control_t *p){
p->Kp=0;
p->Ki=0;
p->Kd=0;
p->err=0;
p->errD=0;
p->errI=0;
}



void pid_setErr(pid_control_t *p, sensor_t val){
p->err = val;
}

void pid_setErrD(pid_control_t *p, sensor_t val,sensor_t dt){

p->errD = (val/131)/dt;

}
void pid_setErrI(pid_control_t *p, sensor_t dt){
p->errI += p->err*dt;
}


int pid_control(pid_control_t* control){
	
	MV = (int)(control->Kp*control->err + control->Ki*control->errI + control->Kd*control->errD);
	//control->err_prev = control->err;
	//control->errD_prev = control->errD;
	return (int)MV;
}


/*
int TF_function(TF_t t ,motor_t val,sensor_t dt){
	motor_t err;
	int MV;
	//motor_t delta_mv;
	motor_t errD;
	//motor_t MV;
	err = val-target[t];
	//mconst[t][errI] += err*dt;
	//PID[t].errI += err*dt;

	//errD = (WEIGHT*(err-mconst[t][err_prev])/dt+(1-WEIGHT)*(mconst[t][errD_prev]));

	if(t == ROLL){
		errD = gyroX*250/32768/dt;
	}else{
		errD = gyroY*250/32768/dt;
	}

	//errD = (err-mconst[t][err_prev])/dt;
	//MV = (int)(mconst[t][Kp]*err+ mconst[t][Ki]*mconst[t][errI] + mconst[t][Kd]*errD);
	//MV = (int)(PID[t].Kp*err + PID[t].Ki*PID[t].errI + PID[t].Kd*errD);
	if(MV>=10000){
		MV=10000;
		//PID[t].errI -= err*dt;
		//mconst[t][errI] -= err*dt;
	//	mconst[t][errI] -=err;
	}else if(MV<=-10000){
		MV=-10000;
		//PID[t].errI -= err*dt;
		//mconst[t][errI] -= err*dt;
	//	mconst[t][errI] -=err;
	}
	PID[t].err_prev = err;
	PID[t].errD_prev = errD;
	//mconst[t][err_prev]=err;
	//mconst[t][errD_prev]=errD;
	return (int)MV;
}*/

uint32_t pid_feedforward(int m_val){
	if(m_val>10000){
		return 10000;
	}else if(m_val<0){
		return 0;
	}
	//motor_t x = ((motor_t)m_val)*0.065725; 
	motor_t x1 = m_val*657.25/10000;
	motor_t x2 = powf(x,2);
	motor_t x3 = x2*x;
	motor_t x4 = x3*x;
	motor_t	x5 = x4*x;

	//motor_t y = 4.4887*powf(10,-10)*x5 - 5.874*powf(10,-7)* x4 + 0.00030083*x3 - 0.057589 *x2 + 6.2994*x1 - 0.098165;
	motor_t y = 0.00000000044887f*x5 - 0.0000005874f*x4 + 0.00030083f*x3 - 0.057589f *x2 + 6.2994f*x1 - 0.098165f;
	return (uint32_t)y;
}



