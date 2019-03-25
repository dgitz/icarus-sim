/*
 * MotorModel.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#include "MotorModel.h"

MotorModel::MotorModel() {
	// TODO Auto-generated constructor stub

}

MotorModel::~MotorModel() {
	// TODO Auto-generated destructor stub
}
bool MotorModel::init(std::string part_number,double _gearbox_ratio)
{
	gearbox_ratio = _gearbox_ratio;
	if(part_number == "361006")
	{
		type = MotorModelType::REDLINE775;
		rated_voltage = 12.0;
		max_rpm = 1637.0*rated_voltage/gearbox_ratio;
	}
	else
	{
		type = MotorModelType::UNKNOWN;
		return false;
	}


	return true;
}
double MotorModel::set_input(double v)
{
	//m=(y2-y1)/(x2-x1), y-y1=m(x-x1)
	double m = (max_rpm-(-1.0*max_rpm))/(rated_voltage-(-1.0*rated_voltage));
	double out = m*(v-(-1.0*rated_voltage))+(-1.0*max_rpm);

	return out*2*3.14159/60.0;
}
