/*
 * MotorModel.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#include "MotorModel.h"

MotorModel::MotorModel() {
	// TODO Auto-generated constructor stub
	set_voltage = 0.0;
	motor_resistance = 0.0;
}

MotorModel::~MotorModel() {
	// TODO Auto-generated destructor stub
}
double MotorModel::get_currentconsumed()
{
	return current_consumed;
}
bool MotorModel::init(std::string motor_part_number,std::vector<std::string> gearbox_partnumbers,double extra_gearbox,double t_circuitbreaker_size)
{
	circuitbreaker_size = t_circuitbreaker_size;
	gearbox_ratio = 1.0;
	for(std::size_t i = 0; i < gearbox_partnumbers.size(); ++i)
	{
		if(gearbox_partnumbers.at(i) == "542026")
		{
			gearbox_ratio = gearbox_ratio*20.0;
		}
		else
		{
			printf("[ERROR]: Gearbox PN: %s Not Supported.\n",gearbox_partnumbers.at(i).c_str());
		}
		
	}
	gearbox_ratio = gearbox_ratio * extra_gearbox;
	
	if(motor_part_number == "361006")
	{
		type = MotorModelType::REDLINE775;
		rated_voltage = 12.0;
		max_rpm = 21000.0/gearbox_ratio;
		motor_resistance = 0.112;
		max_rpm_withcircuitbreaker = (circuitbreaker_size/(rated_voltage/motor_resistance))*max_rpm;
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
	set_voltage = v;
	double set_rpm = (v/rated_voltage)*max_rpm;
	if(set_rpm > max_rpm_withcircuitbreaker)
	{
		set_rpm = max_rpm_withcircuitbreaker;
		current_consumed = circuitbreaker_size;
	}
	else if(set_rpm < -max_rpm_withcircuitbreaker)
	{
		set_rpm = -max_rpm_withcircuitbreaker;
		current_consumed = circuitbreaker_size;
	}
	else
	{
		current_consumed = set_voltage/motor_resistance;
	}
	
	return set_rpm*2*M_PI/60.0; //Convert rpm to rad/s
}
