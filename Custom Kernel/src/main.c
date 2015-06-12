/* Copyright (C) 2015 HAN Jae Seung, JAES Electronics. All rights reserved.
   |
   |    rolling spider 
   |     --Top view--
   |   .....pitch.......
   |   2     head      0
   |    \     0|0     /
   |      \   0|0   /
   |        \  |  /
   |          \|/______> roll
   |          /|\
   |        /     \
   |      /         \
   |    /             \
   |   3                1
 */

#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include <math.h>
#include <time.h>
#include <sched.h>
#include <stdint.h>
#include "HAL_motor_J.h"
#include "HAL_pid_J.h"
#include "HAL_sensor_J.h"
#include "gpio_J.h"


#include "ultra_snd.h"

/*CMD CMD*/
typedef enum {
EMERGENCY = 0, //system stop all motor init
TAKEOFF = 1, // start timer and motor and pid
SYSTEM_KONST = 2,// input = sys Kp Ki Kd 
THROTT_INPUT = 3,
TOTAL_SENSOR = 4,
MAHONY = 5,
BATT = 6,
}SYSTEM_CMD;

typedef enum{
S_ACCEL=0,
S_GYRO=1,
S_KALMAN = 2 ,// motor value 0 1 2 3 input
S_COMPLEMENTARY = 3,
S_NEW = 4,
S_QUAT = 5,
S_QUAT_TO_EULER = 6,
S_INTEGRAL = 7,
S_MOTOR = 8,
}SENSRO_CMD;



void DEBUG(char *s,...){
#if 0
	printf(s);
#endif
}

//pthread_mutex_t  mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t  gmutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

uint32_t mask_bit =0;
timer_t timer1;
int TIMER_EMER =1;
int  thrott[4];

sensor_struct_t angle;
pid_control_t pid[MAX_SYSTEM];
sensor_t dt=0.002;

void main_control_code  (int signum)
{
	GpioWrite(LED_RED_LEFT,1);
	if(TIMER_EMER==1){
		return ;
	}
	
	//	float gx = (gyroX*250/32768)*DEG_TO_RAD;
	//	float gy = (gyroY*250/32768)*DEG_TO_RAD;
	//	float gz = (gyroZ*250/32768)*DEG_TO_RAD;
	//	float ax = accX*GRAVITY*2/32768;
	//	float ay = accY*GRAVITY*2/32768;
	//	float az = accZ*GRAVITY*2/32768;
	//	MahonyAHRSupdateIMU(gx,gy,gz,ax,ay,az);	
	//	Yaw_m=atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*RAD_TO_DEG;
	//	Pitch_m=-1*asin(2*q1*q3+2*q0*q2)*RAD_TO_DEG;
	//	Roll_m=atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*RAD_TO_DEG;
	motor_t target_roll = 0;
	motor_t target_pitch = 0;

	int m_0,m_1,m_2,m_3;
	int rForce,pForce;	

	sensor_read(&angle);
	sensor_filtering(KALMAN, &angle,dt);

	pid_setErr(&pid[ROLL],angle.roll-target_roll);
	pid_setErrD(&pid[ROLL],angle.mpu.gyroX,dt);
	pid_setErrI(&pid[ROLL],dt);
	rForce=pid_control(&pid[ROLL]);
	
	pid_setErr(&pid[PITCH],angle.pitch-target_pitch);
	pid_setErrD(&pid[PITCH],angle.mpu.gyroY,dt);
	pid_setErrI(&pid[PITCH],dt);
	pForce = pid_control(&pid[PITCH]);

	m_0 = pid_feedforward((rForce+pForce)/2+thrott[0]);
	m_1 = pid_feedforward((rForce-pForce)/2+thrott[1]);
	m_2 = pid_feedforward((-rForce+pForce)/2+thrott[2]);
	m_3 = pid_feedforward((-rForce-pForce)/2+thrott[3]);

	motor_setDuty(0,(m_0));
	motor_setDuty(1,(m_1));
	motor_setDuty(2,(m_2));
	motor_setDuty(3,(m_3));


	GpioWrite(LED_RED_LEFT,0);
}



/**********************************************************

  timer working function    High resolution timer + high priority process
  if endtime == 0 then infinite  wait for timer
  nperiod = nanoperiod,  nano end time;

 ************************************************************/
sigset_t set;
int TimerWorking(long nperiod, long nendtime){//if endtime == 0 then infinite  wait for timer 
	timer_t timer2;
	struct itimerspec new_value, old_value;
	struct sigaction action;
	struct sigevent sevent;

	struct sched_param sp = { .sched_priority = 10 };
	int ret;
	/* process  priority high */
	ret = sched_setscheduler(0, SCHED_FIFO, &sp);
	if (ret == -1) {
		perror("sched_setscheduler");
		return 1;
	}

	/* SIGALRM for printing time */
	memset (&action, 0, sizeof (struct sigaction));
	action.sa_handler = main_control_code;
	if (sigaction (SIGALRM, &action, NULL) == -1)
		perror ("sigaction");

	/* for program completion */
	memset (&sevent, 0, sizeof (struct sigevent));
	sevent.sigev_notify = SIGEV_SIGNAL;
	sevent.sigev_signo = SIGRTMIN;

	if (timer_create (CLOCK_MONOTONIC, NULL, &timer1) == -1)
		//	if (timer_create (CLOCK_REALTIME, NULL, &timer1) == -1)
		perror ("timer_create");

	new_value.it_interval.tv_sec = 0;
	new_value.it_interval.tv_nsec = nperiod; /* 1ms */
	new_value.it_value.tv_sec = 0;
	new_value.it_value.tv_nsec = nperiod; /* 1ms */
	if (timer_settime (timer1, 0, &new_value, &old_value) == -1)
		perror ("timer_settime");

	if (sigemptyset (&set) == -1)
		perror ("sigemptyset");

	if (sigaddset (&set, SIGRTMIN) == -1)
		perror ("sigaddset");

	if (sigprocmask (SIG_BLOCK, &set, NULL) == -1)
		perror ("sigprocmask");

	if (timer_create (CLOCK_MONOTONIC, &sevent, &timer2) == -1)
		perror ("timer_create");
	if(nendtime!=0){
		new_value.it_interval.tv_sec = 0;
		new_value.it_interval.tv_nsec = 0; /* one time timer, no reset */
		new_value.it_value.tv_sec = 0;
		new_value.it_value.tv_nsec = nendtime; /* endtime */
		if (timer_settime (timer2, 0, &new_value, &old_value) == -1)
			perror ("timer_settime");
	}
}

int TimerWait(){
	int signum;
	/* wait for completion signal (1 ms) */
	if (sigwait (&set, &signum) == -1)
		perror ("sigwait");
	exit (EXIT_SUCCESS);
	return 0;
}


void *t_function(void *data){
	char buffer[50];
	while(1){	
		/*if(mask_bit&(1<<S_NEW)){
			sprintf(buffer,"_J_DATA: NEWX %8.5f NEWY %8.5f NEWZ %8.5f \n",new_roll,new_pitch,new_yaw);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_KALMAN)){
			sprintf(buffer,"_J_DATA: KROLL %f KPITCH %f \n",kalAngleX,kalAngleY);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_GYRO)){
			sprintf(buffer,"_J_DATA: GYROX %d GYROY %d GYROZ %d \n",gyroX,gyroY,gyroZ);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_ACCEL)){
			sprintf(buffer,"_J_DATA: ACCX %d ACCY %d ACCZ %d \n",accX,accY,accZ);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_COMPLEMENTARY)){
			sprintf(buffer,"_J_DATA: CROLL %8.5f CPITCH %8.5f \n",compAngleX,compAngleY);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_QUAT)){
			//sprintf(buffer,"_J_ %8.5f %8.5f %8.5f %8.5f \n %8.5f %8.5f %8.5f \n",q0,q1,q2,q3,new_roll,new_pitch,new_yaw);
			sprintf(buffer,"_J_ %8.5f %8.5f %8.5f %8.5f \n",q0,q1,q2,q3);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_QUAT_TO_EULER)){
			sprintf(buffer,"_J_ %8.5f %8.5f %8.5f \n",Roll_m,Pitch_m,Yaw_m);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}
		if(mask_bit&(1<<S_INTEGRAL)){
			sprintf(buffer,"_J_  %8.5f %8.5f %d \n",pid[ROLL].errI,pid[PITCH].errI,MV);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}if(mask_bit&(1<<S_MOTOR)){
			sprintf(buffer,"_J_  %d %d %d %d \n",m_0,m_1,m_2,m_3);
			write(STDOUT_FILENO,buffer,strlen(buffer));
		}*/

		usleep(5000);
	}

}






int fd_ultra;
int Batt_enable=0;
void *batt_adc(void *data){
	int bat_val=0;
	char buf[20];
	while(1){
		if(Batt_enable==0){
			sleep(1);
			continue;
		}
		if((ioctl(fd_ultra,BATTERY,&bat_val))<0){
			printf("ioctl err\n");
			exit(0);
		}
		sprintf(buf,"batt = %d \n",bat_val);
		write(1,buf,strlen(buf));
		sleep(1);
	}

}

void BattInit(){
	if((fd_ultra = open("/dev/ultra_snd",O_RDWR))<0){
		printf("ultra open err");
		exit(0);
	}
}

/********************************************************
 *
 *		MAIN function
 *
 *********************************************************/


int main(){

	system("killall dragon-prog");
	sleep(2);

	sensor_init(&angle);
	motor_init();
	pid_init(&pid[ROLL]);
	pid_init(&pid[PITCH]);
	
	BattInit();
	int i;
	//mconst[0][Kp]=0;
	//mconst[0][Ki]=0;
	//mconst[0][Kd]=0;
	//mconst[1][Kp]=0;
	//mconst[1][Ki]=0;
	//mconst[1][Kd]=0;

	/*GPIO init*/
	GpioDir(LED_RED_LEFT,GPIO_OUTPUT_LOW);
	GpioDir(LED_GREEN_LEFT,GPIO_OUTPUT_LOW);

	/*Pthread init*/
	pthread_t p_thread[2];
	if(pthread_create(&p_thread[0], NULL, t_function, (void *)NULL)<0){
		perror("thread create error : ");
		exit(0);
	}
	if(pthread_create(&p_thread[1], NULL, batt_adc, (void *)NULL)<0){
		perror("thread create error : ");
		exit(0);
	}
	/*Timer init*/
	TimerWorking((int)(dt*1000000000),0);
	while(1){
		int num=-1;
		scanf("%d",&num);
		if(num==TOTAL_SENSOR){
			int dd,tru;
			scanf("%d %d",&dd,&tru);
			if(tru==1){
				mask_bit|=(1<<dd);
			}else{
				mask_bit&=~(1<<dd);
			}
			printf("mask_bit %d\n",dd);
			num =-1;
		}else if(num==SYSTEM_KONST){
			int sys;
			scanf("%d",&sys);
			pid[ROLL].errI=0;
			pid[PITCH].errI=0;
			//mconst[0][errI]=0;
			//mconst[1][errI]=0;
			if(sys>=MAX_SYSTEM){
				perror("sys input over");

			}else{	
				scanf("%f %f %f",&pid[sys].Kp,&pid[sys].Ki,&pid[sys].Kd);
				printf("_J_DATA: Kp %f Ki %f Kd %f \n",pid[sys].Kp,pid[sys].Ki,pid[sys].Kd);
			}
			num=-1;
		}else if(num==EMERGENCY){
			TIMER_EMER=1;
			pid[ROLL].errI=0;
			pid[PITCH].errI=0;
			motor_setDuty(0,0);
			motor_setDuty(1,0);
			motor_setDuty(2,0);
			motor_setDuty(3,0);
			num =-1;
		}else if(num==TAKEOFF){

			pid[ROLL].errI=0;
			pid[PITCH].errI=0;
			TIMER_EMER=0;
			
			num=-1;
		}else if(num==THROTT_INPUT){
			scanf("%d %d %d %d",&thrott[0],&thrott[1],&thrott[2],&thrott[3]);
			num=-1; 

			 }else if(num==BATT){
				 int d;
				 scanf("%d",&d);
				 if(d==1){
					 Batt_enable=1;
				 }else{
					 Batt_enable=0;
				 }
			 }
	}

	TimerWait();

}
