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
#include <ignition/math/Vector3.hh>

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
#include "std_msgs/Float64.h"
#include <eros/signal.h>
#include <eros/imu.h>
#include <eros/pin.h>
#include <eros/pose.h>
#include <eros/heartbeat.h>
#include <eros/battery.h>
//Project
#include "../../include/SimpleTimer.h"
#include "../../../eROS/include/eROS_Definitions.h"
#include "../../../eROS/include/Supported_PN.h"
#include "../../../eROS/include/logger.h"
#include "../../../eROS/include/eros_math.h"
#include "Power/MotorControllerModel/MotorControllerModel.h"
#include "Power/MotorModel/MotorModel.h"
#include "Power/BatteryModel/BatteryModel.h"
#include "Sensor/Truth/TruthSensor.h"
#include "Sensor/IMU/IMUSensor.h"
#include "Sensor/Sonar/SonarSensor.h"
#include "Sensor/WheelEncoder/WheelEncoderSensor.h"
#include "Actuator/LinearActuatorModel/LinearActuatorModel.h"
#include "CameraPanTilt/CameraPanTilt.h"
#include "../../../eROS/include/DiagnosticClass.h"
#define ALLOW_INCOMPLETEMODEL_INITIALIZATION true
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
	void enable_diagnostics(std::vector<uint8_t> diagnostic_types)
	{
		root_diagnostic.Node_Name = "gazebo_client";
		root_diagnostic.DeviceName = "SimRover";
		root_diagnostic.System = SIMROVER;
		root_diagnostic.SubSystem = ENTIRE_SYSTEM;
		root_diagnostic.Component = ENTIRE_SUBSYSTEM;
		std::sort(diagnostic_types.begin(),diagnostic_types.end());
		for(std::size_t i = 0; i < diagnostic_types.size(); ++i)
		{
			eros::diagnostic diag = root_diagnostic;
			diag.Diagnostic_Type = diagnostic_types.at(i);
			if(diagnostic_types.at(i) == SYSTEM_RESOURCE) //This is special, so we don't throw a ton of warn messages when the system launches.
			{
				diag.Level = NOTICE;
				diag.Diagnostic_Message = INITIALIZING;
				diag.Description = "Initializing Resource Monitor.";
			}
			else
			{
				diag.Level = WARN;
				diag.Diagnostic_Message = INITIALIZING;
				diag.Description = "Initializing Diagnostic.";
			}	
			diagnostics.push_back(diag);
		}
	}
	//Update Functions
	virtual void OnUpdate();
	
	//Utility Functions
	void print_model();
	Logger* get_logger() { return logger; }
	//Message Functions
	void drivetrain_left_cmd(const eros::pin::ConstPtr& _msg);
	void drivetrain_right_cmd(const eros::pin::ConstPtr& _msg);
	void implement_cmd(const eros::pin::ConstPtr& _msg);
	void boom_rotate_cmd(const eros::pin::ConstPtr& _msg);
	void bucket_rotate_cmd(const eros::pin::ConstPtr& _msg);
	void panservo_cmd(const eros::pin::ConstPtr& _msg);
	void tiltservo_cmd(const eros::pin::ConstPtr& _msg);
	
	//Attributes
	std::vector<eros::diagnostic> get_diagnostics() { return diagnostics; }
	//Destructors

private:
	//Structs
	
	enum class JointType
	{
		UNKNOWN = 0,
		DRIVETRAIN_LEFT =1,
		DRIVETRAIN_RIGHT = 2,
		LINEAR_ACTUATOR = 3,
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
	Logger *logger;
	bool logger_initialized;
	double scale_value(double input_perc,double y1,double neutral,double y2);
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
	struct SonarStorage
	{
		bool initialized;
		SonarSensor sensor;
		sensors::SonarSensorPtr m_gazebo_sonar;
		ros::Publisher pub;
	};
	struct PinStorage
	{
		eros::pin pin;
	};
	struct LinearActuatorStorage
	{
		bool initialized;
		uint64_t actuator_id;
		uint64_t sensor_id;
		LinearActuatorModel linear_actuator;
		ros::Publisher current_pub;
		ros::Subscriber command_sub;
	};
	struct CameraPanTiltStorage
	{
		bool initialized;
		CameraPanTilt assy;
		uint16_t panjoint_id;
		uint16_t tiltjoint_id;

	};
	//Initialize Functions
	IMUStorage initialize_imu(std::string partnumber,std::string location);
	SonarStorage initialize_sonar(std::string partnumber,std::string location);
	CameraPanTiltStorage initialize_camerapantilt(int16_t panjoint_id,int16_t tiltjoint_id);
	bool InitializePlugin();
	bool LoadModel();
	bool LoadSensors();
	bool InitializeSubscriptions();
	bool InitializePublications();
	bool readLinkPose(std::string shortname,ignition::math::Pose3d* pose);
	//Update Functions
	void QueueThread();
	double  compute_distance(ignition::math::Pose3d a, ignition::math::Pose3d b);
	double compute_magnitude(ignition::math::Vector3d a);
	eros::diagnostic update_diagnostic(uint8_t diagnostic_type,uint8_t level,uint8_t message,std::string description);
	eros::diagnostic update_diagnostic(std::string device_name,uint8_t diagnostic_type,uint8_t level,uint8_t message,std::string description);
	eros::diagnostic update_diagnostic(eros::diagnostic diag);
	//Utility Functions
	std::string map_jointtype_tostring(JointType joint_type);
	void print_loopstates(SimpleTimer timer);
	void print_jointinfo(bool v);
	double convert_pwm_toangle_deg(int32_t v);
	
	//Model Variables
	Compute_Average compute_average;
	//Communication Variables
	bool kill_node;
	std::unique_ptr<ros::NodeHandle> rosNode;
	transport::NodePtr node;
	event::ConnectionPtr updateConnection;
	ros::CallbackQueue rosQueue;
	physics::ModelPtr m_model;
	std::thread rosQueueThread;
	ros::Publisher pub_heartbeat;
	ros::Publisher pub_diagnostic;
	ros::Subscriber sub_drivetrain_left_cmd;
	ros::Subscriber sub_drivetrain_right_cmd;
	ros::Subscriber sub_boomrotate_cmd;
	ros::Subscriber sub_bucketrotate_cmd;
	ros::Subscriber sub_panservo_cmd;
	ros::Subscriber sub_tiltservo_cmd;
	ros::Publisher pub_leftimu;
	ros::Publisher pub_rightimu;
	ros::Publisher pub_truthpose;
	ros::Publisher pub_leftwheelencoder;
	ros::Publisher pub_rightwheelencoder;
	ros::Publisher pub_batteryinfo;
	ros::Publisher pub_gazebofps;
	transport::SubscriberPtr sub_keyboardevent;

	//Diagnostic Variables
	eros::heartbeat heartbeat;
	DiagnosticClass diagnostic_helper;
	eros::diagnostic root_diagnostic;
	std::vector<eros::diagnostic> diagnostics;
	//Timing Variables
	double run_time;
	double last_simtime;
	uint64_t last_iterationcount;
	SimpleTimer m_fastloop;
	SimpleTimer m_mediumloop;
	SimpleTimer m_slowloop;
	SimpleTimer m_veryslowloop;
	SimpleTimer m_20hzloop;

	//State Variables
	ignition::math::Pose3d initial_pose;
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
	MotorControllerModel left_motorcontroller;
	MotorControllerModel right_motorcontroller;

	MotorModel left_motor;
	MotorModel right_motor;

	WheelEncoderStorage left_wheelencoder;
	WheelEncoderStorage right_wheelencoder;

	std::vector<LinearActuatorStorage> linear_actuators;
	std::vector<SonarStorage> sonar_sensors;

	CameraPanTiltStorage camera_pantilt;

};
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif /* SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_ */
