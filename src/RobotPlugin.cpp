/*
 * RobotPlugin.cpp
 *
 *  Created on: Mar 10, 2019
 *      Author: robot
 */

#include "RobotPlugin.h"
using namespace gazebo;
using namespace std;
RobotPlugin::RobotPlugin(): ModelPlugin(),
		m_model(nullptr)
{
	left_cmd = 0.0;
	right_cmd = 0.0;
	// TODO Auto-generated constructor stub
	printf("Plugin opened\n");
}

RobotPlugin::~RobotPlugin() {
	// TODO Auto-generated destructor stub
}
void RobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	printf("Plugin load started\n");
	m_model = _model;

	InitializePlugin();

	return;

}
bool RobotPlugin::InitializePlugin()
{
	if(LoadModel() == false)
	{
		printf("Model Failed to Load. Exiting.\n");
		return false;
	}
	print_model();
	this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
	this->node->Init(m_model->GetWorld()->GetName());
#else
	this->node->Init(this->model->GetWorld()->Name());
#endif
	printf("Starting ros\n");
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
	}
	rosNode.reset(new ros::NodeHandle("gazebo_client"));
	if(InitializeSubscriptions() == false)
	{
		printf("Not able to initialize Subscriptions. Exiting.\n");
	}
	printf("Plugin Started.\n");
	return true;
}
bool RobotPlugin::LoadModel()
{
	printf("Initializing Model.\n");
	//joint_list = _joint_list;
	if(m_model->GetJointCount() == 0)
	{
		printf("Robot Model did not receive any info.  Exiting.\n");
		return false;
	}
	for(uint16_t i = 0; i < m_model->GetJointCount(); ++i)
	{
		if (m_model->GetJoints()[i]->GetScopedName().find("drivetrain") != std::string::npos)
		{
			if (m_model->GetJoints()[i]->GetScopedName().find("left") != std::string::npos)
			{
				joint newjoint;
				newjoint.joint_type = JointType::DRIVETRAIN_LEFT;
				newjoint.id = i;
				newjoint.poweron_setpoint = 0.0;
				newjoint.name = m_model->GetJoints()[i]->GetScopedName();
				joints.push_back(newjoint);
			}
			if (m_model->GetJoints()[i]->GetScopedName().find("right") != std::string::npos)
			{
				joint newjoint;
				newjoint.joint_type = JointType::DRIVETRAIN_RIGHT;
				newjoint.id = i;
				newjoint.poweron_setpoint = 0.0;
				newjoint.name = m_model->GetJoints()[i]->GetScopedName();
				joints.push_back(newjoint);
			}
		}
	}
	drivetrain_left_pid = common::PID(0.1, 0, 0);
	drivetrain_right_pid = common::PID(0.1, 0, 0);
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
		{
			m_model->GetJointController()->SetVelocityPID(
					m_model->GetJoints()[i]->GetScopedName(), drivetrain_left_pid);
			m_model->GetJointController()->SetVelocityTarget(
					m_model->GetJoints()[joints.at(i).id]->GetScopedName(), joints.at(i).poweron_setpoint);
		}
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
		{
			m_model->GetJointController()->SetVelocityPID(
					m_model->GetJoints()[i]->GetScopedName(), drivetrain_right_pid);
			m_model->GetJointController()->SetVelocityTarget(
					m_model->GetJoints()[joints.at(i).id]->GetScopedName(), joints.at(i).poweron_setpoint);
		}

	}
	return true;
}
void RobotPlugin::print_model()
{
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		printf("[%d] Type: %d Name: %s\n",
				joints.at(i).id,
				joints.at(i).joint_type,
				joints.at(i).name.c_str());
	}
}
bool RobotPlugin::InitializeSubscriptions()
{
	{
		ros::SubscribeOptions so =
				ros::SubscribeOptions::create<std_msgs::Float32>(
						"/" +  m_model->GetName() + "/drivetrain_left_cmd",
						1,
						boost::bind(&RobotPlugin::drivetrain_left_cmd, this, _1),
						ros::VoidPtr(), &this->rosQueue);
		sub_drivetrain_left_cmd = this->rosNode->subscribe(so);
	}
	{
		ros::SubscribeOptions so =
				ros::SubscribeOptions::create<std_msgs::Float32>(
						"/" +  m_model->GetName() + "/drivetrain_right_cmd",
						1,
						boost::bind(&RobotPlugin::drivetrain_right_cmd, this, _1),
						ros::VoidPtr(), &this->rosQueue);
		sub_drivetrain_right_cmd = this->rosNode->subscribe(so);
	}
	this->rosQueueThread =
			std::thread(std::bind(&RobotPlugin::QueueThread, this));

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&RobotPlugin::OnUpdate, this));
	return true;
}
void RobotPlugin::QueueThread()
{
	static const double timeout = 0.01;
	while (this->rosNode->ok())
	{
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}
void RobotPlugin::OnUpdate()
{
	/*
	printf("Left: %f Right: %f\n",left_cmd,right_cmd);
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0,left_cmd);
		}
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
		{
			m_model->GetJoint(joints.at(i).name)->SetParam("vel", 0, right_cmd);
		}
	}
	 */
}
void RobotPlugin::drivetrain_left_cmd(const std_msgs::Float32ConstPtr &_msg)
{
	left_cmd = _msg->data;
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
		{
			printf("Left: %f\n",left_cmd);
			m_model->GetJointController()->SetVelocityTarget(
					m_model->GetJoints()[joints.at(i).id]->GetScopedName(), _msg->data);
		}
	}

}
void RobotPlugin::drivetrain_right_cmd(const std_msgs::Float32ConstPtr &_msg)
{
	right_cmd = _msg->data;
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
		{
			printf("Right: %f\n",right_cmd);
			m_model->GetJointController()->SetVelocityTarget(
					m_model->GetJoints()[joints.at(i).id]->GetScopedName(), _msg->data);
		}
	}
}
