/*
 * SonarSensor.cpp
 *
 */

#include "SonarSensor.h"

SonarSensor::SonarSensor() {
	// TODO Auto-generated constructor stub
	initialized = false;

}

SonarSensor::~SonarSensor() {
	// TODO Auto-generated destructor stub
}
bool WheelEncoderSensor::init(std::string t_partnumber,std::string t_name)
{
	name = t_name;
	update_count = 0;
	partnumber = t_partnumber;
	signal.name = name;
	signal.type = SIGNALTYPE_TICKSPEED;
	signal.status = SIGNALSTATE_INITIALIZING;
	signal.value = 0.0;
	if(partnumber == PN_110001)
	{
	}
	else
	{
		initialized = false;
		return false;
	}
	
	initialized = true;
	return true;
}
double SonarSensor::get_currentconsumed()
{
	if(partnumber == PN_110001)
	{
		return 0.035;
	}
	return 0.0;
}
eros::signal SonarSensor::update(double t_current_time,double sensor_value)
{
	if(update_count > 0)
	{
		double dt = t_current_time-current_time;
	}
	
	current_time = t_current_time;
	rms = compute_rms(rms,sensor_value,update_count);
	signal.value = sensor_value;
	signal.rms = rms.value;
	signal.status = SIGNALSTATE_UPDATED;
	update_count++;
	return signal;
}
SonarSensor::RMS SonarSensor::compute_rms(RMS rms,double value,uint64_t t_update_count)
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