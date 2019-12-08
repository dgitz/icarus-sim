/*
 * TruthSensor.h
 *
 */

#ifndef SRC_ICARUS_SIM_SRC_TRUTHSENSOR_H_
#define SRC_ICARUS_SIM_SRC_TRUTHSENSOR_H_
#include "string"
#include <math.h> 
#include <eros/signal.h>
#include <eros/pose.h>
#include <eigen3/Eigen/Dense>
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>

#include "../../../../../eROS/include/eROS_Definitions.h"
#define TRANSMIT_RATE 50.0 //Hz

class TruthSensor {
public:
	TruthSensor();
	virtual ~TruthSensor();
	bool update_worldpose(
		double t_current_time,
		ignition::math::Pose3d pose,
		ignition::math::Vector3d rate);
	eros::pose get_pose() { return eros_pose; }
private:
	struct RMS
	{
		double value;
		double mean;
	};
	double current_time;
	uint64_t update_count;
	RMS compute_rms(RMS rms,double value,uint64_t t_update_count);
	eros::pose eros_pose;
	
	ignition::math::Pose3d world_pose;
	ignition::math::Vector3d angular_rate;
	RMS rollrate_rms;
	RMS pitchrate_rms;
	RMS yawrate_rms;
	RMS roll_rms;
	RMS pitch_rms;
	RMS yaw_rms;
	RMS east_rms;
	RMS north_rms;
	RMS elev_rms;

	
};

#endif /* SRC_ICARUS_SIM_SRC_TRUTHSENSOR_H_ */
