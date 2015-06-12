#include "pwm_ioctl.h"

#include "pwm_J.h"
#include "gpio_J.h"
	int fd_pwm;

int pwm_open(){
	GpioOpen();
	if((fd_pwm = open(PWM_DEV, O_RDWR))<0)perror("open");
}
int pwm_powerOn(){
	GpioDir(39,GPIO_OUTPUT_LOW);
}
int pwm_powerOff(){
	GpioDir(39,GPIO_OUTPUT_HIGH);
}


int pwm_SetWidth( unsigned int pwm,unsigned int width ){
	ioctl(fd_pwm, PWM_REQUEST, &pwm);
	ioctl(fd_pwm, PWM_SET_WIDTH, &width);
	ioctl(fd_pwm, PWM_RELEASE,&pwm);
	return 0;
}

int pwm_init(){
	unsigned int pwm =0;
	unsigned int pwm_width = 0;
	unsigned int pwm_freq =  10000;
	//printf("input pwm num\n");
	//scanf("%d",&pwm);

	for(pwm=0;pwm<4;pwm++){
		//		printf("pwm %d \n",pwm);
		if((ioctl(fd_pwm, PWM_REQUEST, &pwm))<0)perror("request");
		if((ioctl(fd_pwm, PWM_SET_FREQ, &pwm_freq))<0)perror("set req");
		if((ioctl(fd_pwm, PWM_SET_WIDTH, &pwm_width))<0)perror("set wid");
		if((ioctl(fd_pwm, PWM_START))<0)perror("start");
		if((ioctl(fd_pwm, PWM_RELEASE,&pwm))<0)perror("release");

	}

}
