/*
 * MotorControllerModel.h
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_SIM_SRC_MOTORCONTROLLERMODEL_H_
#define SRC_ICARUS_SIM_SRC_MOTORCONTROLLERMODEL_H_
#include "string"
class MotorControllerModel {
public:
	enum class MotorControllerType
	{
		UNKNOWN=0,
		VICTOR_SPX=1,
	};
	MotorControllerModel();
	bool init(std::string part_number);
	void set_batteryvoltage(double v)
	{
		battery_voltage = v;

	}
	double set_input(double v);
	virtual ~MotorControllerModel();
private:
	double battery_voltage;
	MotorControllerType type;
	double pwm_in_max;
	double pwm_in_neutral;
	double pwm_in_min;
};

#endif /* SRC_ICARUS_SIM_SRC_MOTORCONTROLLERMODEL_H_ */
