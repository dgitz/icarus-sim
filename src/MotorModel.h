/*
 * MotorModel.h
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_SIM_SRC_MOTORMODEL_H_
#define SRC_ICARUS_SIM_SRC_MOTORMODEL_H_
#include "string"
class MotorModel {
public:
	enum class MotorModelType
	{
		UNKNOWN = 0,
		REDLINE775= 1,
	};
	MotorModel();
	bool init(std::string part_number,double _gearbox_ratio);
	virtual ~MotorModel();
	double set_input(double v);
private:
	MotorModelType type;
	double gearbox_ratio;
	double max_rpm;
	double rated_voltage;
};

#endif /* SRC_ICARUS_SIM_SRC_MOTORMODEL_H_ */
