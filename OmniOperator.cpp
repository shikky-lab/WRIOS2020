/*
 * OmniOperator.cpp
 *
 *  Created on: Jul 18, 2019
 *      Author: mfukunaga
 */

#include "OmniOperator.hpp"
#include <math.h>
#include <pigpiod_if2.h>
#include <stdio.h>
#include <map>

void OmniOperator::fastAtan(float y,float x){
	int quadrant=0;
	
	if(x>0 && y>0){
		
	}
	
	

}

#define PI 3.14159265359
#define deg_to_rad(deg) (((deg)/360)*2*PI)

//多項式近似による実現．0.001radくらいのずれだったので，多分実用上問題なさそう．
float _atan2(int16_t _y, int16_t _x) {

	if(_x==0){
		if(_y>0){
			return PI/2;
		}else{
			return -PI/2;
		}
	}

  int16_t x = abs(_x);
  int16_t y = abs(_y);
  float   z;
  bool    c;

  c = y < x;
  if (c)z = (float)y / x;
  else  z = (float)x / y;

  int16_t a;
  //a = z * (-1556 * z + 6072);                     //2次曲線近似
  //a = z * (z * (-448 * z - 954) + 5894);          //3次曲線近似
  a = z * (z * (z * (829 * z - 2011) - 58) + 5741); //4次曲線近似

  if (c) {
    if (_x > 0) {
      if (_y < 0)a *= -1;
    }
    if (_x < 0) {
      if (_y > 0)a = 18000 - a;
      if (_y < 0)a = a - 18000;
    }
  }

  if (!c) {
    if (_x > 0) {
      if (_y > 0) a = 9000 - a;
      if (_y < 0) a = a - 9000;
    }
    if (_x < 0) {
      if (_y > 0) a = a + 9000;
      if (_y < 0) a = -a - 9000;
    }
  }

  return deg_to_rad((float)a/100);
}

//x,yはそれぞれ1以下，というか合計して1になる小数の想定．
//じゃなかった．nunchukの時は中央1,1でそれぞれ0~2の値
void OmniOperator::calc_translation(float x, float y){
	if(x==0 && y==0){
		top_motor_temp=0;
		left_motor_temp=0;
		right_motor_temp=0;
		return;
	}

//	float rad0 = x!=0?atan2f(y,x):y>0?PI/2:-PI/2;
	float rad = _atan2((int16_t)(y*1000),(int16_t)(x*1000));

//	printf("org=%.5f\tsimpl=%.5f\r\n",rad0,rad);

	float motor_power = sqrt(powf(x,2)+powf(y,2));


	top_motor_temp = (int32_t)(cosf(rad-TOP_MOTOR_RAD)*motor_power * calc_max_count);
	left_motor_temp = (int32_t)(cosf(rad-LEFT_MOTOR_RAD)*motor_power * calc_max_count);
	right_motor_temp = (int32_t)(cosf(rad-RIGHT_MOTOR_RAD)*motor_power * calc_max_count);
}

void OmniOperator::calc_rotation(float r){
	if(r==0.f){
		return;//return to reduce calculation.
	}
	float abs_r = r>0?r:-r;

	top_motor_temp = (int32_t)(top_motor_temp+r*(calc_max_count - top_motor_temp)) ;
	left_motor_temp =(int32_t)(left_motor_temp+r*(calc_max_count - left_motor_temp)) ;
	right_motor_temp =(int32_t)(right_motor_temp+r*(calc_max_count - right_motor_temp)) ;
}

void OmniOperator::set_motor_count(){
	top_motor_temp += middle_count;
	left_motor_temp += middle_count;
	right_motor_temp += middle_count;
//	printf("top:%.3f,left:%.3f,right%.3f\r\n",top_motor_temp,left_motor_temp,right_motor_temp);
	if(top_motor_temp>0 && top_motor_temp<middle_count*2){
            set_PWM_dutycycle(PI_ID,TOP_PIN_ID,top_motor_temp);
	}
	if(left_motor_temp>0 && left_motor_temp<middle_count*2){
            set_PWM_dutycycle(PI_ID,LEFT_PIN_ID,left_motor_temp);
	}
	if(right_motor_temp>0 && right_motor_temp<middle_count*2){
            set_PWM_dutycycle(PI_ID,RIGHT_PIN_ID,right_motor_temp);
	}
}

OmniOperator::OmniOperator(uint8_t id,uint8_t top,uint8_t left,uint8_t right):PI_ID(id),TOP_PIN_ID(top),LEFT_PIN_ID(left),RIGHT_PIN_ID(right){
}

void OmniOperator::init(int freq,int range){
    set_PWM_frequency(PI_ID,TOP_PIN_ID,freq);
    set_PWM_frequency(PI_ID,LEFT_PIN_ID,freq);
    set_PWM_frequency(PI_ID,RIGHT_PIN_ID,freq);
    
    set_PWM_range(PI_ID,TOP_PIN_ID,range);
    set_PWM_range(PI_ID,LEFT_PIN_ID,range);
    set_PWM_range(PI_ID,RIGHT_PIN_ID,range);
    
    max_count=range/2;
    calc_max_count = max_count;
    middle_count = range/2;
}

void OmniOperator::set_limit(int percent) {
    calc_max_count = max_count *percent/100;
}


//receive,x(-1,1),y(-1,1),r(-1,1)
void OmniOperator::move(float x,float y,float r){

	calc_translation(x,y);
	calc_rotation(r);
	set_motor_count();
}

