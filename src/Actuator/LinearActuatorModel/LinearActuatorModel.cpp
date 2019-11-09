/*
 * LinearActuatorModel.cpp
 *
 */

#include "LinearActuatorModel.h"

LinearActuatorModel::LinearActuatorModel() {
	// TODO Auto-generated constructor stub
	run_time = 0.0;
	current_voltage = 0.0;
	current_position = 0.0;
	target_velocity = 0.0;
	target_force = 0.0;
	upper_position_limit = 0.0;
	lower_position_limit = 0.0;
	peak_power_point_force = 0.0;
	peak_power_point_speed = 0.0;
	peak_efficiency_point_force = 0.0;
	peak_efficiency_point_speed = 0.0;
	max_speed_noload = 0.0;
	max_force = 0.0;
	backdrive_force = 0.0;
	rated_voltage = 0.0;
	stall_current = 0.0;
	max_staticforce = 0.0;
	max_dutycycle = 0.0;

	current_signal.name = "";
	current_signal.tov = -1.0;
	current_signal.type = SIGNALTYPE_DISTANCE;
	current_signal.status = SIGNALSTATE_INITIALIZING;
	current_signal.value = 0.0;
	current_signal.rms = 0.0;
}
LinearActuatorModel::~LinearActuatorModel() {
	// TODO Auto-generated destructor stub
}
bool LinearActuatorModel::update(double dt,double t_current_voltage,double t_current_position)
{
	run_time += dt;
	current_voltage = t_current_voltage;
	current_position = t_current_position;
	current_signal.tov = run_time;
	current_signal.status = SIGNALSTATE_UPDATED;
	current_signal.value = current_position;
	return true;
}
void LinearActuatorModel::set_targetpinvalue(eros::pin pin)
{
	//TODO: Convert pin value to target velocity
	double perc_value = convert_pwmvalue((double)pin.Value);
	target_velocity = perc_value*peak_power_point_speed*(current_voltage/rated_voltage);
	target_force = perc_value*peak_efficiency_point_force*(current_voltage/rated_voltage);
	//printf("joint: %s pin: %d perc: %f target vel: %f\n",joint_name.c_str(),pin.Value,perc_value,target_velocity);
}
void LinearActuatorModel::print_info()
{
	printf("[%4.2f INFO] Name: %s(%s) Position: %4.4fm Target Force: %4.4fN\n",
		run_time,joint_name.c_str(),real_jointname.c_str(),current_position,target_force);
}
bool LinearActuatorModel::init(std::string part_number,std::string t_joint_name,std::string t_realjoint_name)
{
	joint_name = t_joint_name;
	real_jointname = t_realjoint_name;
	if(part_number == "361008")
	{
		partnumber = SupportedPartNumber::PN_361008;	
		upper_position_limit = 0.147; // meters
		lower_position_limit = 0.97; // meters
		peak_power_point_force = 250; // Newtons
		peak_power_point_speed = 0.0025; // meters/second
		peak_efficiency_point_force = 150; // Newtons
		peak_efficiency_point_speed = 0.0034; // meters/second
		max_speed_noload = .0048; // meters/second
		max_force = 300; // Newtons
		backdrive_force = 500; // Newtons
		rated_voltage = 12.0; // Volts
		stall_current = 1.0; // Amps at Rated Voltage
		max_staticforce = 500; // Newtons
		max_dutycycle = 0.2; // Percentage, max of 1.0
		current_signal.name = t_joint_name + "CurrentPosition";

	}
	else
	{
		return false;
	}
	return true;
}
double LinearActuatorModel::convert_pwmvalue(double v)
{
	//Convert pwm value of 1000-2000 to percent from -1 to 1
	double m = .002;
	double v1 = m*(v-1000)-1.0;
	return v1;
}