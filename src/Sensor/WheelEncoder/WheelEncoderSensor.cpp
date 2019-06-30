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
bool WheelEncoderSensor::init(std::string partnumber,std::string t_name)
{
	name = t_name;
	update_count = 0;
	position_mod = 0.0;
	position = 0.0;
	tick_value = 0;
	signal.name = name;
	signal.type = SIGNALTYPE_TICKSPEED;
	signal.status = SIGNALSTATE_INITIALIZING;
	signal.value = 0.0;
	if(partnumber == "110003")
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
eros::signal WheelEncoderSensor::update(double t_current_time,double wheel_velocity)
{
	double tick_velocity = 0.0;
	if(update_count > 0)
	{
		double dt = t_current_time-current_time;
		position_mod+=(wheel_velocity*dt)*180.0/M_PI;
		position+=(wheel_velocity*dt)*180.0/M_PI;
		if(position_mod > 180.0)
		{
			position_mod = -180.0;
		}
		if(position_mod < -180.0)
		{
			position_mod  = 180.0;
		}
		int16_t last_tick = tick_value;
		tick_value = position*(count_per_revolution/2.0)/180.0;
		tick_velocity = (tick_value-last_tick)/dt;
	}
	
	current_time = t_current_time;
	rms = compute_rms(rms,tick_velocity,update_count);
	signal.value = tick_velocity;
	signal.rms = rms.value;
	signal.status = SIGNALSTATE_UPDATED;
	update_count++;
	return signal;
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