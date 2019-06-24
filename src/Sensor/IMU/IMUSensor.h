/*
 * IMUSensor.h
 *
 */

#ifndef SRC_ICARUS_SIM_SRC_IMUSENSOR_H_
#define SRC_ICARUS_SIM_SRC_IMUSENSOR_H_
#include "string"
#include <math.h> 
#include <eros/signal.h>
#include <eros/imu.h>
#include <eigen3/Eigen/Dense>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>

#include "../../../../eROS/include/eROS_Definitions.h"
#define MAGNETOMETER_MAGNITUDE_LOWERBOUND 0.5f
#define MAGNETOMETER_MAGNITUDE_UPPERBOUND 5.0f
class IMUSensor {
public:
	IMUSensor();
	bool init(std::string t_name);
	void set_pose(gazebo::math::Pose pose);
	virtual ~IMUSensor();
	eros::imu update_IMU(double current_time,double last_imu_update,double last_mag_update,
	ignition::math::Vector3d linear_acc,ignition::math::Vector3d angular_rate,ignition::math::Vector3d sim_magdata);
	Eigen::Matrix3f GetRotationMatrix_Acceleration() { return rotate_matrix.Rotation_Acc; }
	Eigen::Matrix3f GetRotationMatrix_AngularRate() { return rotate_matrix.Rotation_Gyro; }
	Eigen::Matrix3f GetRotationMatrix_MagneticField() { return rotate_matrix.Rotation_Mag; }
private:
	struct RotationMatrix
	{
		Eigen::Matrix3f Rotation_Acc_X;
		Eigen::Matrix3f Rotation_Acc_Y;
		Eigen::Matrix3f Rotation_Acc_Z;
		Eigen::Matrix3f Rotation_Acc;
		Eigen::Matrix3f Rotation_Gyro_X;
		Eigen::Matrix3f Rotation_Gyro_Y;
		Eigen::Matrix3f Rotation_Gyro_Z;
		Eigen::Matrix3f Rotation_Gyro;
		Eigen::Matrix3f Rotation_Mag_X;
		Eigen::Matrix3f Rotation_Mag_Y;
		Eigen::Matrix3f Rotation_Mag_Z;
		Eigen::Matrix3f Rotation_Mag;
	};
	gazebo::math::Pose sensor_pose;
	RotationMatrix generate_rotation_matrix(double mao_roll_rad,double mao_pitch_rad,double mao_yaw_rad);
	RotationMatrix rotate_matrix;
	std::string name;
	double time_imu_updated;
	double time_imumag_updated;
	uint64_t update_count;
	eros::imu imu_data;

	double xacc_rms_mean1;
	double yacc_rms_mean1;
	double zacc_rms_mean1;
	double xgyro_rms_mean1;
	double ygyro_rms_mean1;
	double zgyro_rms_mean1;
	double xmag_rms_mean1;
	double ymag_rms_mean1;
	double zmag_rms_mean1;
};

#endif /* SRC_ICARUS_SIM_SRC_MOTORMODEL_H_ */
