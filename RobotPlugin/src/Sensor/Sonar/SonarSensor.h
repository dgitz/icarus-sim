/*
 * SonarSensor.h
 */

#ifndef SRC_ICARUS_SIM_SRC_SONARSENSOR_H_
#define SRC_ICARUS_SIM_SRC_SONARSENSOR_H_
#include "string"
#include <math.h> 
#include <eros/signal.h>
#include <eigen3/Eigen/Dense>
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>

#include "../../../../../eROS/include/eROS_Definitions.h"
#include "../../../../../eROS/include/Supported_PN.h"
class SonarSensor {
public:
	SonarSensor();
	bool init(std::string t_partnumber,std::string t_name);
	bool is_initialized() { return initialized; }
	double get_currentconsumed();
	eros::signal update(double t_current_time,double sensor_value);
	virtual ~SonarSensor();
private:
	struct RMS
	{
		double value;
		double mean;
	};
	std::string partnumber;
	double count_per_revolution;
	bool initialized;
	RMS compute_rms(RMS rms,double value,uint64_t t_update_count);
	std::string name;
	eros::signal signal;
	RMS rms;
	uint64_t update_count;
	double current_time;
};

#endif /* SRC_ICARUS_SIM_SRC_SONARSENSOR_H_ */
