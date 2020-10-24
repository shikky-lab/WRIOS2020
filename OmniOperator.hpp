/*
 * OmniOperator.hpp
 *
 *  Created on: Jul 18, 2019
 *      Author: mfukunaga
 */

#ifndef OMNIOPERATOR_HPP_
#define OMNIOPERATOR_HPP_

#include <cstdint>


class OmniOperator{

private:
	float top_motor_temp,left_motor_temp,right_motor_temp;

        const uint8_t PI_ID;
        const uint8_t TOP_PIN_ID;
        const uint8_t LEFT_PIN_ID;
        const uint8_t RIGHT_PIN_ID;
	const float PI = 3.14159265359;
	const float TOP_MOTOR_RAD = 0;
	const float LEFT_MOTOR_RAD = 2*PI/3;
	const float RIGHT_MOTOR_RAD = -2*PI/3;
	uint32_t target_x,target_y,target_r;
        int max_count;
        int middle_count;
        int calc_max_count;
        

	void calc_translation(float y, float x);
	void calc_rotation(float r);
	void set_motor_count();
	void fastAtan(float y,float x);
public:        
	OmniOperator(uint8_t id,uint8_t top,uint8_t left,uint8_t right);
        void set_limit(int percent);
        void init(int freq,int range);
	void move(float x, float y, float r);
};

#endif /* OMNIOPERATOR_HPP_ */
