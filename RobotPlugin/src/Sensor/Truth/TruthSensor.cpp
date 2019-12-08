/*
 * TruthSensor.cpp
 *
 */

#include "TruthSensor.h"

TruthSensor::TruthSensor() {
	// TODO Auto-generated constructor stub
	eros_pose.yawrate.type = SIGNALTYPE_ROTATION_RATE;
	eros_pose.yawrate.status = SIGNALSTATE_INITIALIZING;
	
	eros_pose.pitchrate.type = SIGNALTYPE_ROTATION_RATE;
	eros_pose.pitchrate.status = SIGNALSTATE_INITIALIZING;

	eros_pose.rollrate.type = SIGNALTYPE_ROTATION_RATE;
	eros_pose.rollrate.status = SIGNALSTATE_INITIALIZING;

	eros_pose.yawrate.type = SIGNALTYPE_ROTATION_RATE;
	eros_pose.yawrate.status = SIGNALSTATE_INITIALIZING;

	eros_pose.yaw.type = SIGNALTYPE_ANGLE;
	eros_pose.yaw.status = SIGNALSTATE_INITIALIZING;

	eros_pose.pitch.type = SIGNALTYPE_ANGLE;
	eros_pose.pitch.status = SIGNALSTATE_INITIALIZING;

	eros_pose.roll.type = SIGNALTYPE_ANGLE;
	eros_pose.roll.status = SIGNALSTATE_INITIALIZING;

	eros_pose.east.type = SIGNALTYPE_DISTANCE;
	eros_pose.east.status = SIGNALSTATE_INITIALIZING;

	eros_pose.north.type = SIGNALTYPE_DISTANCE;
	eros_pose.north.status = SIGNALSTATE_INITIALIZING;

	eros_pose.elev.type = SIGNALTYPE_DISTANCE;
	eros_pose.elev.status = SIGNALSTATE_INITIALIZING;

}
TruthSensor::~TruthSensor() {
	// TODO Auto-generated destructor stub
}
bool TruthSensor::update_worldpose(
	double t_current_time,
	ignition::math::Pose3d pose,
	ignition::math::Vector3d rate)
{
	current_time = t_current_time;
	world_pose = pose;
	angular_rate = rate;
	{	//Angular Rate
		eros_pose.rollrate.value = rate.X()*180.0/M_PI;
		eros_pose.pitchrate.value = rate.Y()*180.0/M_PI;
		eros_pose.yawrate.value = rate.Z()*180.0/M_PI;
		rollrate_rms = compute_rms(rollrate_rms,eros_pose.rollrate.value,update_count);
		pitchrate_rms = compute_rms(pitchrate_rms,eros_pose.pitchrate.value,update_count);
		yawrate_rms = compute_rms(yawrate_rms,eros_pose.yawrate.value,update_count);
		eros_pose.rollrate.rms = rollrate_rms.value;
		eros_pose.pitchrate.rms = pitchrate_rms.value;
		eros_pose.yawrate.rms = pitchrate_rms.value;
		eros_pose.rollrate.status = SIGNALSTATE_UPDATED;
		eros_pose.pitchrate.status = SIGNALSTATE_UPDATED;
		eros_pose.yawrate.status = SIGNALSTATE_UPDATED;
	}
	{	//Orientation
		eros_pose.roll.value = pose.Rot().Roll()*180.0/M_PI;
		eros_pose.pitch.value = pose.Rot().Pitch()*180.0/M_PI;
		eros_pose.yaw.value = pose.Rot().Yaw()*180.0/M_PI;
		roll_rms = compute_rms(roll_rms,eros_pose.roll.value,update_count);
		pitch_rms = compute_rms(pitch_rms,eros_pose.pitch.value,update_count);
		yaw_rms = compute_rms(yaw_rms,eros_pose.yaw.value,update_count);
		eros_pose.roll.rms = roll_rms.value;
		eros_pose.pitch.rms = pitch_rms.value;
		eros_pose.yaw.rms = yaw_rms.value;
		eros_pose.roll.status = SIGNALSTATE_UPDATED;
		eros_pose.pitch.status = SIGNALSTATE_UPDATED;
		eros_pose.yaw.status = SIGNALSTATE_UPDATED;
	}
	{	//Position
		eros_pose.east.value = pose.Pos().X();
		eros_pose.north.value = pose.Pos().Y();
		eros_pose.elev.value = pose.Pos().Y();
		east_rms = compute_rms(east_rms,eros_pose.east.value,update_count);
		north_rms = compute_rms(north_rms,eros_pose.north.value,update_count);
		elev_rms = compute_rms(elev_rms,eros_pose.elev.value,update_count);
		eros_pose.east.rms = east_rms.value;
		eros_pose.north.rms = north_rms.value;
		eros_pose.elev.rms = elev_rms.value;
		eros_pose.east.status = SIGNALSTATE_UPDATED;
		eros_pose.north.status = SIGNALSTATE_UPDATED;
		eros_pose.elev.status = SIGNALSTATE_UPDATED;
	}
	eros_pose.tov = current_time;
	eros_pose.sequence_number = update_count;
	update_count++;
	return true;
}
TruthSensor::RMS TruthSensor::compute_rms(RMS rms,double value,uint64_t t_update_count)
{
	if(t_update_count == 0)
	{
		rms.mean = pow(value,2.0);
	}
	else
	{
		double a = (pow(value,2.0)/(double)((t_update_count+1)));
		double b = (rms.mean/(double)(t_update_count+1));
		rms.mean += a-b;
	}
	rms.value = pow(rms.mean,0.5);
	return rms;
}