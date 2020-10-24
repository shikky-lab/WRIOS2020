/*
 * OmniOperator.cpp
 *
 *  Created on: Jul 18, 2019
 *      Author: mfukunaga
 */

#include "ArmOperator.hpp"
#include <math.h>
#include <pigpiod_if2.h>
#include <stdio.h>
#include <unistd.h>//sleep()用

ArmOperator::ArmOperator(uint8_t piId,uint8_t tiltPin,uint8_t panPin,uint8_t extruderPin, uint8_t limitSwPin):PI_ID(piId),TILT_PIN_ID(tiltPin),PAN_PIN_ID(panPin),EXTRUDER_PIN_ID(extruderPin),LIMIT_SW_PIN_ID(limitSwPin){
}

void ArmOperator::init(int freq,int range){
	/*押し出し部分モータ初期化*/
	pwm_middle=range/2;
    set_PWM_frequency(PI_ID,EXTRUDER_PIN_ID,freq);//300Hz
    set_PWM_range(PI_ID,EXTRUDER_PIN_ID,range);//resolution 1000
	set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_middle);

	pwm_extension_speed=980;
	pwm_shorten_speed=20;

	/*押し出し部分のリミットスイッチ初期化*/
    set_mode(PI_ID, LIMIT_SW_PIN_ID, PI_INPUT);
    set_pull_up_down(PI_ID, LIMIT_SW_PIN_ID, PI_PUD_UP);

//	tilt_middle=1500;//実測値を直接指定
	tilt_middle=1337;//実測値を直接指定
	pan_middle=1500;
	
	/*アーム根本のサーボ初期化*/
    set_PWM_frequency(PI_ID,PAN_PIN_ID,50);//50Hz
    set_PWM_range(PI_ID,PAN_PIN_ID,20000); 
	set_PWM_dutycycle(PI_ID,PAN_PIN_ID,pan_middle);
    set_PWM_frequency(PI_ID,TILT_PIN_ID,50);//50Hz
    set_PWM_range(PI_ID,TILT_PIN_ID,20000);
	set_PWM_dutycycle(PI_ID,TILT_PIN_ID,tilt_middle);

}

int32_t ArmOperator::calcPan(int32_t angle){
	if(angle>900){
		return getServoMax(angle);
	}
	if(angle<-900){
		return getServoMin(angle);
	}
	
	return (pan_middle + angle*5/9);
}

/*+-1800で入力*/
void ArmOperator::setPan(int32_t angle){
	
	set_PWM_dutycycle(PI_ID,PAN_PIN_ID,calcPan(angle));
}

int32_t ArmOperator::getServoMax(int32_t middleVal){
	if(middleVal+SERVO_RANGE>SERVO_MAX){
		return middleVal+SERVO_RANGE;
	}else{
		return SERVO_MAX;
	}
}

int32_t ArmOperator::getServoMin(int32_t middleVal){
	if(middleVal-SERVO_RANGE<SERVO_MIN){
		return middleVal-SERVO_RANGE;
	}else{
		return SERVO_MIN;
	}
}
int32_t ArmOperator::calcTilt(int32_t angle){
	if(angle>900){
		return getServoMin(angle);
	}
	if(angle<-900){
		return getServoMax(angle);
	}
	
	return (tilt_middle - angle*5/9);
}
/*+-1800で入力*/
void ArmOperator::setTilt(int32_t angle){
	
	set_PWM_dutycycle(PI_ID,TILT_PIN_ID,calcTilt(angle));
}

void ArmOperator::setTiltMiddle(int32_t angle){
	tilt_middle=calcTilt(angle);
}

int32_t ArmOperator::getTiltMiddle(){
	return tilt_middle;
}

void ArmOperator::extendArm(){
	moving_count=0;
	state=EXTENDING;
    button_pushed=1;
    button_released=0;
	//set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_extension_speed);

}

void ArmOperator::shortenArm(){
	moving_count=0;
	state=SHORTENING;   
    button_released=1;
    button_pushed=0;
	//set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_shorten_speed);
}

int ArmOperator::sw_pressed(){
	return gpio_read(PI_ID,LIMIT_SW_PIN_ID);

}
int ArmOperator::sw_released(){
	return !gpio_read(PI_ID,LIMIT_SW_PIN_ID);
}

void ArmOperator::stop_arm(){
	set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_middle);
	state=STOP;
	moving_count=0;//無くてもいいはずだが念のため
    button_released=0;
    button_pushed=0;
}

void ArmOperator::checkArmState(){
	
    if(button_pushed==1){
        set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_extension_speed);
    }
    if(button_released==1){
        set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,pwm_shorten_speed);
    }
    moving_count++;
	switch(state){
		case STOP:
			moving_count=0;//無くてもいいはずだが念のため
			return;
		case EXTENDING:
			if(moving_count>EXTENDING_COUNT_MAX){
				stop_arm();
			}
			return;
		case SHORTENING:
			if(sw_pressed()){
				stop_arm();
			}
			break;
	}

	if(moving_count>MOVING_COUNT_MAX){
		stop_arm();
		return;
	}

}