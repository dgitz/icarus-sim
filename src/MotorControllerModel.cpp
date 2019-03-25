/*
 * MotorControllerModel.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#include "MotorControllerModel.h"

MotorControllerModel::MotorControllerModel() {
	// TODO Auto-generated constructor stub

}

MotorControllerModel::~MotorControllerModel() {
	// TODO Auto-generated destructor stub
}

bool MotorControllerModel::init(std::string part_number)
{
	if(part_number == "362009")
	{
		type = MotorControllerType::VICTOR_SPX;
		pwm_in_max = 2000.0;
		pwm_in_neutral = 1500.0;
		pwm_in_min = 1000.0;
	}
	else
	{
		type = MotorControllerType::UNKNOWN;
		return false;
	}

	return true;
}
double MotorControllerModel::set_input(double v)
{
	//m=(y2-y1)/(x2-x1), y-y1=m(x-x1)
	double m = (battery_voltage-(-1.0*battery_voltage))/(pwm_in_max-pwm_in_min);
	double out = m*(v-pwm_in_min) + (-1.0*battery_voltage);
	return out;
}
