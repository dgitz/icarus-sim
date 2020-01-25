/*
 * MotorModel.h
 *
 *  Created on: Mar 25, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_SIM_SRC_MOTORMODEL_H_
#define SRC_ICARUS_SIM_SRC_MOTORMODEL_H_
#include "string"
#include "vector"
#include "math.h"
#include "../../../../../eROS/include/Supported_PN.h"
class MotorModel {
public:
	enum class MotorModelType
	{
		UNKNOWN = 0,
		REDLINE775= 1,
	};
	MotorModel();
	bool init(std::string motor_part_number,std::vector<std::string> gearbox_partnumbers,double extra_gearbox,double t_circuitbreaker_size );
	virtual ~MotorModel();
	double set_input(double v);
	double get_currentconsumed();
private:
	MotorModelType type;
	double circuitbreaker_size;
	double current_consumed;
	double gearbox_ratio;
	double max_rpm;
	double max_rpm_withcircuitbreaker;
	double rated_voltage;
	double set_voltage;
	double motor_resistance;

};

#endif /* SRC_ICARUS_SIM_SRC_MOTORMODEL_H_ */
