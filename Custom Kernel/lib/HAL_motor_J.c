#include "HAL_motor_J.h"

int motor_init(){
pwm_open();
pwm_powerOn();
pwm_init();

pwm_SetWidth(0,0);
pwm_SetWidth(1,0);
pwm_SetWidth(2,0);
pwm_SetWidth(3,0);
}


int motor_setDuty(unsigned int pwm,unsigned int width ){
	pwm_SetWidth(pwm,width);
}


