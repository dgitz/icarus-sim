/*
 * RobotPlugin.h
 *
 *  Created on: Mar 10, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_
#define SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <stdint.h>
#include "string"
#include <gazebo/physics/physics.hh>
#include <functional>
namespace gazebo
{
class RobotPlugin: public ModelPlugin {
public:
	RobotPlugin();
	virtual ~RobotPlugin();
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void OnUpdate();
	void print_model();

	//Messages
	void drivetrain_left_cmd(const std_msgs::Float32ConstPtr &_msg);
	void drivetrain_right_cmd(const std_msgs::Float32ConstPtr &_msg);
private:
	bool InitializePlugin();
	bool LoadModel();
	bool InitializeSubscriptions();
	void QueueThread();
	enum JointType
	{
		UNKNOWN = 0,
		DRIVETRAIN_LEFT =1,
		DRIVETRAIN_RIGHT = 2,
	};
	struct joint
	{
		JointType joint_type;
		double poweron_setpoint;
		uint16_t id;
		std::string name;
	};
	std::vector<joint> joints;
	std::unique_ptr<ros::NodeHandle> rosNode;

	transport::NodePtr node;
	event::ConnectionPtr updateConnection;
	ros::CallbackQueue rosQueue;
	physics::ModelPtr m_model;
	std::thread rosQueueThread;

	ros::Subscriber sub_drivetrain_left_cmd;
	common::PID drivetrain_left_pid;
	ros::Subscriber sub_drivetrain_right_cmd;
	common::PID drivetrain_right_pid;
	double left_cmd;
	double right_cmd;
};
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif /* SRC_ICARUS_SIM_SRC_ROBOTPLUGIN_H_ */
