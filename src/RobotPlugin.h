/*
 * RobotPlugin.h
 *
 *  Created on: Mar 10, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_
#define SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_

//C System Files
//C++ System Files
#include <thread>
#include <vector>
#include <stdint.h>
#include "string"
//Gazebo Base Functionality
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
//#include <ignition/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>

#include <functional>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <ignition/math/Vector3.hh>
//ROS Base Functionality
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
//Gazebo Messages
#include <gazebo/msgs/msgs.hh>
//ROS Messages
#include <eros/signal.h>
#include <eros/imu.h>
#include <eros/pin.h>
#include <eros/pose.h>
#include <eros/battery.h>
//Project
#include "../include/SimpleTimer.h"
#include "../../eROS/include/eROS_Definitions.h"
#include "Power/MotorControllerModel/MotorControllerModel.h"
#include "Power/MotorModel/MotorModel.h"
#include "Power/BatteryModel/BatteryModel.h"
#include "Sensor/Truth/TruthSensor.h"
#include "Sensor/IMU/IMUSensor.h"
#include "Sensor/WheelEncoder/WheelEncoderSensor.h"

#define INITIALIZATION_TIME 5.0f
#define KEYCODE_UPARROW 16777235
#define KEYCODE_LEFTARROW 16777234
#define KEYCODE_DOWNARROW 16777237
#define KEYCODE_RIGHTARROW 16777236
#define KEYCODE_SPACE 32
#define KEYCODE_ENTER 13

namespace gazebo
{
/*! \class RobotPlugin RobotPlugin.h "RobotPlugin.h"
 *  \brief This is a RobotPlugin class.  This plugin can be used by Gazebo.*/
class GAZEBO_VISIBLE RobotPlugin: public ModelPlugin {
public:
	RobotPlugin();
	virtual ~RobotPlugin();
	//Constants
	//Structs
	//Initialization Functions
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	//Update Functions
	virtual void OnUpdate();
	
	//Utility Functions
	void print_model();
	//Message Functions
	void drivetrain_left_cmd(const eros::pin::ConstPtr& _msg);
	void drivetrain_right_cmd(const eros::pin::ConstPtr& _msg);
	void boom_rotate_cmd(const eros::pin::ConstPtr& _msg);
	void bucket_rotate_cmd(const eros::pin::ConstPtr& _msg);
	
	//Destructors

private:
	//Structs
	
	enum class JointType
	{
		UNKNOWN = 0,
		DRIVETRAIN_LEFT =1,
		DRIVETRAIN_RIGHT = 2,
		BOOM_ROTATE = 3,
		BUCKET_ROTATE = 4,
	};
	enum class LinkType
	{
		UNKNOWN = 0,
		DRIVETRAIN_LEFT =1,
		DRIVETRAIN_RIGHT = 2,
		BOOM_ROTATE = 3,
		BUCKET_ROTATE = 4,
	};
	struct joint
	{
		JointType joint_type;
		double poweron_setpoint;
		uint16_t id;
		std::string name;
	};
	struct link
	{
		LinkType link_type;
		uint16_t id;
		std::string name;
	};
	struct DrivePerc
	{
		double left;
		double right;
	};
	void KeyboardEventCallback(ConstAnyPtr &_msg);
	double scale_value(double input_perc,double y1,double neutral,double y2);
	DrivePerc arcade_mix(double throttle_perc,double steer_perc);
	struct TruthPoseStorage
	{
		TruthSensor sensor;
		eros::pose pose;
	};
	struct IMUStorage
	{
		bool initialized;
		IMUSensor sensor;
		sensors::MagnetometerSensorPtr m_gazebo_imu_mag;
		sensors::ImuSensorPtr m_gazebo_imu;
		eros::imu eros_imu;
	};
	struct WheelEncoderStorage
	{
		bool initialized;
		WheelEncoderSensor sensor;
	};
	struct PinStorage
	{
		eros::pin pin;
	};
	//Initialize Functions
	IMUStorage initialize_imu(std::string partnumber,std::string location);
	bool InitializePlugin();
	bool LoadModel();
	bool LoadSensors();
	bool InitializeSubscriptions();
	bool InitializePublications();
	bool readLinkPose(std::string shortname,math::Pose* pose);
	//Update Functions
	void QueueThread();
	double  compute_distance(gazebo::math::Pose a, gazebo::math::Pose b);
	//Utility Functions
	std::string map_jointtype_tostring(JointType joint_type);
	void print_loopstates(SimpleTimer timer);
	
	//Communication Variables
	bool kill_node;
	std::unique_ptr<ros::NodeHandle> rosNode;
	transport::NodePtr node;
	event::ConnectionPtr updateConnection;
	ros::CallbackQueue rosQueue;
	physics::ModelPtr m_model;
	std::thread rosQueueThread;
	ros::Subscriber sub_drivetrain_left_cmd;
	ros::Subscriber sub_drivetrain_right_cmd;
	ros::Subscriber sub_boomrotate_cmd;
	ros::Subscriber sub_bucketrotate_cmd;
	ros::Publisher pub_leftimu;
	ros::Publisher pub_rightimu;
	ros::Publisher pub_truthpose;
	ros::Publisher pub_leftwheelencoder;
	ros::Publisher pub_rightwheelencoder;
	ros::Publisher pub_drivetrain_left_cmd;
	ros::Publisher pub_drivetrain_right_cmd;
	ros::Publisher pub_batteryinfo;
	transport::SubscriberPtr sub_keyboardevent;

	//Timing Variables
	double run_time;
	SimpleTimer m_fastloop;
	SimpleTimer m_mediumloop;
	SimpleTimer m_slowloop;
	SimpleTimer m_veryslowloop;

	//State Variables
	gazebo::math::Pose initial_pose;
	bool pose_initialized;
	std::string base_link;
	bool robot_initialized;
	bool drivecommand_received;
	std::vector<joint> joints;
	
	std::vector<link> links;
	common::PID drivetrain_left_pid;
	common::PID drivetrain_right_pid;
	double drivetrain_left_actual_velocity;
	double drivetrain_right_actual_velocity;
	PinStorage drivetrain_left_motorcontroller_pin;
	PinStorage drivetrain_right_motorcontroller_pin;
	double left_cmd;
	double right_cmd;
	common::PID boomrotate_pid;
	common::PID bucketrotate_pid;
	double boomrotate_cmd;
	double bucketrotate_cmd;

	//Sensor Variables
	sensors::SensorManager *m_sensorManager;
	bool sensors_enabled;
	IMUStorage left_imu;
	IMUStorage right_imu;
	TruthPoseStorage truth_pose;
	eros::pose last_pose;
	
	//Robot Modelling
	BatteryModel battery;
	double cmd_throttle;
	double cmd_steer;
	MotorControllerModel left_motorcontroller;
	MotorControllerModel right_motorcontroller;

	MotorModel left_motor;
	MotorModel right_motor;

	WheelEncoderStorage left_wheelencoder;
	WheelEncoderStorage right_wheelencoder;











};
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif /* SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_ */
