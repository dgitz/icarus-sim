/*
 * BatteryModel.cpp
 *
 */

#include "BatteryModel.h"

BatteryModel::BatteryModel() {
	// TODO Auto-generated constructor stub
	run_time = 0.0;
	battery_depleted = false;
	current_consumed = 0.0;

}
double BatteryModel::get_voltage()
{
	if((model_mode == BatteryModelMode::IDEAL) and  (battery_depleted == false))
	{
		actual_voltage = voltage_rated;
	}
	else if((model_mode == BatteryModelMode::DISCHARGE_CURVE) and (battery_depleted == false))
	{
		actual_voltage = series_count*compute_voltage_from_stageofcharge(capacity_remaining_perc);
	}
	else
	{
		actual_voltage = 0.0;
	}
	return actual_voltage;
	
}
BatteryModel::~BatteryModel() {
	// TODO Auto-generated destructor stub
}
bool BatteryModel::update(double dt,double t_current_consumed)
{
	current_consumed = t_current_consumed;
	run_time += dt;
	capacity_remaining -= (current_consumed*dt)/(60.0*60.0);
	capacity_remaining_perc = 100.0*(capacity_remaining/capacity_rated);
	if(capacity_remaining_perc < DEPLETED_THRESHOLD)
	{
		battery_depleted = true;
		capacity_remaining_perc = DEPLETED_THRESHOLD;
		capacity_remaining = (capacity_remaining_perc/100.0)*capacity_rated;
	}
	if(battery_depleted == true)
	{
		return false;
	}
	return true;
}
void BatteryModel::print_info()
{
	printf("[%4.2f INFO] Voltage: %4.2f Current Draw: %f Capacity Remaining: %4.2f\n",
		run_time,actual_voltage,current_consumed,capacity_remaining_perc);
}
eros::battery BatteryModel::get_batteryinfo()
{
	eros::battery bat;
	bat.name = "MainBattery";
	bat.active = true;
	bat.voltage = actual_voltage;
	bat.level_perc = capacity_remaining_perc;
	return bat;
}
bool BatteryModel::init(std::string part_number)
{
	if(part_number == PN_555005)
	{
		series_count = 3;
		parallel_count = 1;
		chemistry = Chemistry::LIPO;
		voltage_rated = 3.7*series_count;
		actual_voltage = voltage_rated;
		capacity_rated = 4.0;
		discharge_rate_normal = 35.0;
		discharge_rate_peak = 70.0;
		model_mode = BatteryModelMode::DISCHARGE_CURVE;
		//Reference: https://batteryuniversity.com/learn/article/lithium_based_batteries
		discharge_curve.stage_of_charge_perc = {100.0,20.0,0.0};
		discharge_curve.voltage_output = {3.9,3.5,3.0};
		
	}
	else
	{
		return false;
	}
	recharge_complete();

	return true;
}
double BatteryModel::compute_voltage_from_stageofcharge(double v_perc)
{
	double voltage = 0.0;
	if(v_perc >= 100.0)
	{
		return discharge_curve.voltage_output.at(0);
	}
	if(v_perc <= 0.0)
	{
		return discharge_curve.voltage_output.at(discharge_curve.voltage_output.size()-1);
	}
	double x1,x2;
	double y1,y2;
	std::size_t mark = 0;
	for(std::size_t i = 1; i < discharge_curve.stage_of_charge_perc.size();++i)
	{
		if(v_perc >= discharge_curve.stage_of_charge_perc.at(i))
		{
			mark = i;
			break;
		}
	}
	if(mark == 0)
	{
		return 0.0;
	}
	x1 = discharge_curve.stage_of_charge_perc.at(mark-1);
	x2 = discharge_curve.stage_of_charge_perc.at(mark);
	y1 = discharge_curve.voltage_output.at(mark-1);
	y2 = discharge_curve.voltage_output.at(mark);
	double m = (y2-y1)/(x2-x1);
	//y-y1=m(x-x1), y = m(x-x1)+y1
	return m*(v_perc-x1)+y1;
}
