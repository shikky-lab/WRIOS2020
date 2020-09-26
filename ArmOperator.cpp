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

ArmOperator::ArmOperator(uint8_t piId,uint8_t tiltPin,uint8_t panPin,uint8_t extruderPin, uint8_t limitSwPin):PI_ID(piId),TILT_PIN_ID(tiltPin),PAN_PIN_ID(panPin),EXTRUDER_PIN_ID(extruderPin),LIMIT_SW_PIN_ID(limitSwPin){
}

void ArmOperator::init(int freq,int range){
	/*押し出し部分モータ初期化*/
    set_PWM_frequency(PI_ID,EXTRUDER_PIN_ID,300);//300Hz
    set_PWM_range(PI_ID,EXTRUDER_PIN_ID,1000);//resolution 1000
	set_PWM_dutycycle(PI_ID,EXTRUDER_PIN_ID,500);

	/*押し出し部分のリミットスイッチ初期化*/
    set_mode(PI_ID, LIMIT_SW_PIN_ID, PI_INPUT);
    set_pull_up_down(PI_ID, LIMIT_SW_PIN_ID, PI_PUD_UP);

	/*アーム根本のサーボ初期化*/
    set_PWM_frequency(PI_ID,PAN_PIN_ID,50);//50Hz
    set_PWM_range(PI_ID,PAN_PIN_ID,20000); 
	set_PWM_dutycycle(PI_ID,PAN_PIN_ID,1500);
    set_PWM_frequency(PI_ID,TILT_PIN_ID,50);//50Hz
    set_PWM_range(PI_ID,TILT_PIN_ID,20000);
	set_PWM_dutycycle(PI_ID,TILT_PIN_ID,1500);
}

