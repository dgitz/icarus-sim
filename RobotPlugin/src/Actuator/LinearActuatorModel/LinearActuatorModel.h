/*
 * LinearActuatorModel.h
 *
 */

#ifndef SRC_ICARUS_SIM_SRC_LINEARACTUATORMODEL_H_
#define SRC_ICARUS_SIM_SRC_LINEARACTUATORMODEL_H_
#include "string"
#include "vector"
#include <eros/signal.h>
#include <eros/pin.h>
#include "../../../../../eROS/include/eROS_Definitions.h"
class LinearActuatorModel {
public:
	LinearActuatorModel();
	bool init(std::string part_number,std::string t_joint_name,std::string t_realjoint_name);
	bool update(double dt,double t_current_voltage,double t_current_position);
	void print_info();
	virtual ~LinearActuatorModel();
	double get_currentposition_m() { return current_position; }
	eros::signal get_currentsignal() { return current_signal; }
	std::string get_commandname() { return joint_name + "Command"; }
	void set_targetpinvalue(eros::pin pin);
	double get_targetvelocity() { return target_velocity; }
	double get_targetforce() { return target_force; }
	void set_targetforce(double v) { target_force = v; }
private:
	double convert_pwmvalue(double v);
	enum class SupportedPartNumber
	{
		UNKNOWN=0,
		PN_361008=1
	};
	/*
	enum class BatteryModelMode
	{
		UNKNOWN=0,
		IDEAL=1,//Full Power available until battery depleted
		DISCHARGE_CURVE=2,
	};
	enum class Chemistry
	{
		UNKNOWN=0,
		LIPO=1,
	};
	struct DishargeCurve
	{
		std::vector<double> stage_of_charge_perc;
		std::vector<double> voltage_output;
	};
	*/
	SupportedPartNumber partnumber;
	std::string real_jointname;
	std::string joint_name;
	double current_voltage;
	double run_time;
	double current_position;
	eros::signal current_signal;
	double target_velocity;
	double target_force;

	//Datasheet Specs
	double upper_position_limit;
	double lower_position_limit;
	double peak_power_point_force;
	double peak_power_point_speed;
	double peak_efficiency_point_force;
	double peak_efficiency_point_speed;
	double max_speed_noload;
	double max_force;
	double backdrive_force;
	double rated_voltage;
	double stall_current;
	double max_staticforce;
	double max_dutycycle;
};

#endif /* SRC_ICARUS_SIM_SRC_LINEARACTUATORMODEL_H_ */
