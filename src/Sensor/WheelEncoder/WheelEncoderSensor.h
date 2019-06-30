/*
 * WheelEncoderSensor.h
 *
 */

#ifndef SRC_ICARUS_SIM_SRC_WHEELENCODERSENSOR_H_
#define SRC_ICARUS_SIM_SRC_WHEELENCODERSENSOR_H_
#include "string"
#include <math.h> 
#include <eros/signal.h>
#include <eigen3/Eigen/Dense>
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>

#include "../../../../eROS/include/eROS_Definitions.h"
class WheelEncoderSensor {
public:
	WheelEncoderSensor();
	bool init(std::string t_name);
	bool is_initialized() { return initialized; }
	eros::signal update(double t_current_time,double wheel_velocity);
	virtual ~WheelEncoderSensor();
private:
	struct RMS
	{
		double value;
		double mean;
	};
	bool initialized;
	RMS compute_rms(RMS rms,double value,uint64_t t_update_count);
	std::string name;
	eros::signal signal;
	RMS rms;
	uint64_t update_count;
	double position;
	double current_time;
	int16_t tick_value;
};

#endif /* SRC_ICARUS_SIM_SRC_WHEELENCODERSENSOR_H_ */
