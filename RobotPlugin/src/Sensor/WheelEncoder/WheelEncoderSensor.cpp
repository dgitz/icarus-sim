/*
 * WheelEncoderSensorWheelEncoderSensor.cpp
 *
 */

#include "WheelEncoderSensor.h"

WheelEncoderSensor::WheelEncoderSensor() {
	// TODO Auto-generated constructor stub
	initialized = false;

}

WheelEncoderSensor::~WheelEncoderSensor() {
	// TODO Auto-generated destructor stub
}
bool WheelEncoderSensor::init(std::string t_partnumber,std::string t_name)
{
	name = t_name;
	update_count = 0;
	partnumber = t_partnumber;
	odom.xodom.name = name;
	odom.xodom.type = SIGNALTYPE_ROTATION_RATE;
	odom.xodom.status = SIGNALSTATE_INITIALIZING;
	odom.xodom.value = 0.0;
	odom.yodom.status = SIGNALSTATE_INVALID;
	odom.zodom.status = SIGNALSTATE_INVALID;
	if(partnumber == PN_110003)
	{
		count_per_revolution = 360.0;
	}
	else
	{
		initialized = false;
		return false;
	}
	
	initialized = true;
	return true;
}
double WheelEncoderSensor::get_currentconsumed()
{
	if(partnumber == PN_110003)
	{
		return 0.025;
	}
	return 0.0;
}
eros::odom WheelEncoderSensor::update(double t_current_time,double wheel_velocity)
{
	current_time = t_current_time;
	rms = compute_rms(rms,wheel_velocity,update_count);
	odom.sequence_number = update_count;
	odom.tov = current_time;
	odom.xodom.tov = current_time;
	odom.xodom.value = wheel_velocity;
	odom.xodom.rms = rms.value;
	odom.xodom.status = SIGNALSTATE_UPDATED;
	update_count++;
	return odom;
}
WheelEncoderSensor::RMS WheelEncoderSensor::compute_rms(RMS rms,double value,uint64_t t_update_count)
{
	if(t_update_count == 0)
	{
		rms.mean = pow(value,2.0);
	}
	else
	{
		double a = (pow(value,2.0)/(double)((t_update_count+1)));
		double b = (rms.mean/(double)(t_update_count+1));
		rms.mean += a-b;
	}
	rms.value = pow(rms.mean,0.5);
	return rms;
}