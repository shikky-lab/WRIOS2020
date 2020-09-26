/*
 * ArmOperator.hpp
 *
 *  Created on: Jul 18, 2019
 *      Author: mfukunaga
 */

#ifndef ARMOPERATOR_HPP_
#define ARMOPERATOR_HPP_

#include <cstdint>


class ArmOperator{

private:
	float top_motor_temp,left_motor_temp,right_motor_temp;

	const uint8_t PI_ID;
	const uint8_t TILT_PIN_ID;
	const uint8_t PAN_PIN_ID;
	const uint8_t EXTRUDER_PIN_ID;
	const uint8_t LIMIT_SW_PIN_ID;

public:
	ArmOperator(uint8_t piId,uint8_t tiltPin,uint8_t panPin,uint8_t extruderPin, uint8_t limitSwPin);
	void init(int freq,int range);
};

#endif /* ARMOPERATOR_HPP_ */
