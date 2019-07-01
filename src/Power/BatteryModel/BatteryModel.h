/*
 * BatteryModel.h
 *
 */

#ifndef SRC_ICARUS_SIM_SRC_BATTERYMODEL_H_
#define SRC_ICARUS_SIM_SRC_BATTERYMODEL_H_
#include "string"
#include "vector"
#include <eros/battery.h>
#define DEPLETED_THRESHOLD 5.0f
class BatteryModel {
public:
	BatteryModel();
	bool init(std::string part_number);
	bool recharge_complete()
	{
		capacity_remaining = capacity_rated;
		capacity_remaining_perc = 100.0;
		battery_depleted = false;
		actual_voltage = voltage_rated;
	}
	bool update(double dt,double current_consumed);
	double get_capacityremaining_perc() { return capacity_remaining_perc; }
	double get_voltage();
	void print_info();
	eros::battery get_batteryinfo();
	virtual ~BatteryModel();
private:
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
	double compute_voltage_from_stageofcharge(double v_perc);
	BatteryModelMode model_mode;
	Chemistry chemistry;
	DishargeCurve discharge_curve;
	double capacity_remaining_perc;
	double current_consumed;
	double run_time;
	double voltage_rated;
	double actual_voltage;
	double series_count;
	double parallel_count;
	double capacity_rated; //Amp-Hours
	double capacity_remaining;
	double discharge_rate_normal; //C-Rating
	double discharge_rate_peak;  //C-Rating
	bool battery_depleted;
};

#endif /* SRC_ICARUS_SIM_SRC_BATTERYMODEL_H_ */
