/*
 * ArmOperator.hpp
 *
 *  Created on: Jul 18, 2019
 *      Author: mfukunaga
 */

#ifndef ARMOPERATOR_HPP_
#define ARMOPERATOR_HPP_

#include <cstdint>

#define SERVO_MAX 2000 
#define SERVO_MIN 1000
#define SERVO_RANGE 300

enum ARM_STATE{
		STOP,
		EXTENDING,
		SHORTENING
	};

class ArmOperator{

private:
	float top_motor_temp,left_motor_temp,right_motor_temp;

	const uint8_t PI_ID;
	const uint8_t TILT_PIN_ID;
	const uint8_t PAN_PIN_ID;
	const uint8_t EXTRUDER_PIN_ID;
	const uint8_t LIMIT_SW_PIN_ID;
	int pwm_middle;
	int pwm_extension_speed;
	int pwm_shorten_speed;
	uint8_t state=STOP;
	const int EXTENDING_COUNT_MAX=6;
	const int MOVING_COUNT_MAX=20;
	int moving_count;
		
	int32_t tilt_middle;
	int32_t pan_middle;

	int32_t getServoMax(int32_t middleVal);
	int32_t getServoMin(int32_t middleVal);

	int32_t calcTilt(int32_t angle);
	int32_t calcPan(int32_t angle);

	int sw_pressed();
	int sw_released();

public:
        int button_pushed=0;
        int button_released=0;
	ArmOperator(uint8_t piId,uint8_t tiltPin,uint8_t panPin,uint8_t extruderPin, uint8_t limitSwPin);
	void init(int freq=300,int range=1000);
	void setPan(int32_t angle);
	void setTilt(int32_t angle);
	void setTiltMiddle(int32_t angle);
	int32_t getTiltMiddle();
	void extendArm();
	void shortenArm();
	void checkArmState();
	void stop_arm();
};

#endif /* ARMOPERATOR_HPP_ */
