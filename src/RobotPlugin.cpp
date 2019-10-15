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
		m_model(nullptr),
		m_sensorManager(nullptr)
		//m_left_imu(nullptr),
		//m_right_imu(nullptr)
{
	left_cmd = 0.0;
	right_cmd = 0.0;
	drivetrain_left_actual_velocity = 0.0;
	drivetrain_right_actual_velocity = 0.0;
	kill_node = false;
	robot_initialized = false;
	run_time = 0.0;
	cmd_throttle = 0.0;
	cmd_steer = 0.0;
	// TODO Auto-generated constructor stub
	printf("Plugin opened\n");
}

RobotPlugin::~RobotPlugin() {
	// TODO Auto-generated destructor stub
}

//Initialize Functions
void RobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	printf("Plugin load started\n");
	m_model = _model;
	pose_initialized = false;
	drivecommand_received = false;
	InitializePlugin();

	return;

}
bool RobotPlugin::InitializePlugin()
{
	sensors_enabled = true;
	if(LoadModel() == false)
	{
		printf("Model Failed to Load. Exiting.\n");
		return false;
	}
	print_model();
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(m_model->GetName());
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
		return false;
	}
	if(InitializePublications() == false)
	{
		printf("Not able to initialize Publications. Exiting.\n");
		return false;
	}
	m_fastloop.set_name("FASTLOOP");
	m_fastloop.set_targetrate(50.0);
	m_mediumloop.set_name("MEDIUMLOOP");
	m_mediumloop.set_targetrate(10.0);
	m_slowloop.set_name("SLOWLOOP");
	m_slowloop.set_targetrate(1.0);
	m_veryslowloop.set_name("VERYSLOWLOOP");
	m_veryslowloop.set_targetrate(0.1);
	printf("Plugin Started.  Waiting %4.2f seconds from Start before Initialization is complete.\n",INITIALIZATION_TIME);
	return true;
}
bool RobotPlugin::LoadModel()
{
	printf("Initializing Model.\n");
	if(LoadSensors() == false)
	{
		printf("Could not load sensors. Exiting.\n");
		return false;
	}
	if(m_model->GetJointCount() == 0)
	{
		printf("Robot Model did not receive any info.  Exiting.\n");
		return false;
	}
	auto t_links = m_model->GetLinks();
	bool found_baselink = false;
	for(std::size_t i = 0; i < t_links.size(); ++i)
	{	
		link newlink;
		newlink.id = (uint16_t)i;
		newlink.name = m_model->GetLinks()[i]->GetScopedName();
		newlink.link_type = LinkType::UNKNOWN;
		if (newlink.name.find("wheel") != std::string::npos)
		{
			if (newlink.name.find("left") != std::string::npos)
			{
				newlink.link_type = LinkType::DRIVETRAIN_LEFT;
			}
			else if (newlink.name.find("right") != std::string::npos)
			{
				newlink.link_type = LinkType::DRIVETRAIN_RIGHT;
			}
		}
		links.push_back(newlink);
		if(newlink.name.find("base") != std::string::npos)
		{
			base_link = newlink.name;
			found_baselink = true;
		}

	}
	if(found_baselink == false)
	{
		printf("Could not find link: base. Exiting.\n");
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
				drivetrain_left_actual_velocity = newjoint.poweron_setpoint;
				newjoint.name = m_model->GetJoints()[i]->GetScopedName();
				if(left_wheelencoder.sensor.is_initialized() == false)
				{
					if(left_wheelencoder.sensor.init("110003","LeftEncoder") == false)
					{
						printf("Could not initialize LeftEncoder.  Exiting.\n");
						return false;
					}
				}
				joints.push_back(newjoint);
			}
			if (m_model->GetJoints()[i]->GetScopedName().find("right") != std::string::npos)
			{
				joint newjoint;
				newjoint.joint_type = JointType::DRIVETRAIN_RIGHT;
				newjoint.id = i;
				newjoint.poweron_setpoint = 0.0;
				drivetrain_right_actual_velocity = newjoint.poweron_setpoint;
				newjoint.name = m_model->GetJoints()[i]->GetScopedName();
				if(right_wheelencoder.sensor.is_initialized() == false)
				{
					if(right_wheelencoder.sensor.init("110003","RightEncoder") == false)
					{
						printf("Could not initialize LeftEncoder.  Exiting.\n");
						return false;
					}
				}
				joints.push_back(newjoint);
			}
			
		}
		if (m_model->GetJoints()[i]->GetScopedName().find("actuator") != std::string::npos)
		{
			joint newjoint;
			newjoint.joint_type = JointType::LINEAR_ACTUATOR;
			newjoint.id = i;
			newjoint.poweron_setpoint = 0.1;
			std::string joint_scopedname = m_model->GetJoints()[i]->GetScopedName();
			newjoint.name = "";
			LinearActuatorModel actuator;	
			if(joint_scopedname == "sim_scout::actuator_left_liftarm_base_joint")
			{
				newjoint.name = "LiftCylinder";
			}
			else if(joint_scopedname == "sim_scout::actuator_leftbellcrank_implementcylinder_joint")
			{
				newjoint.name = "ImplementCylinder";
			}
			else
			{
				printf("Unable to parse Joint: %s. Exiting.\n",joint_scopedname.c_str());
				return false;
			}
			bool status = actuator.init("361008",newjoint.name,joint_scopedname);
			if(status == false)
			{
				printf("Could not Initialize Linear Actuator: %s. Exiting.\n",newjoint.name.c_str());
				return false;
			}
			LinearActuatorStorage linear_actuator_storage;
			linear_actuator_storage.linear_actuator = actuator;
			linear_actuator_storage.initialized = false;
			linear_actuator_storage.actuator_id = newjoint.id;
			for(uint16_t j = 0; j < m_model->GetJointCount(); ++j)
			{
				if (m_model->GetJoints()[j]->GetScopedName().find("sensor") != std::string::npos)
				{
					std::string tempstr = m_model->GetJoints()[j]->GetScopedName();
					if((newjoint.name == "LiftCylinder") && (tempstr == "sim_scout::sensor_left_liftcylinder_prism"))
					{
						linear_actuator_storage.sensor_id = j;
					}
					if((newjoint.name == "ImplementCylinder") && (tempstr == "sim_scout::sensor_implementcylinder_implementbase_prism"))
					{
						linear_actuator_storage.sensor_id = j;
					}
				}
			}
			printf("Created new Joint: %s (Real Name: %s) Sensor ID: %ld Actuator ID: %ld\n",
				newjoint.name.c_str(),
				joint_scopedname.c_str(),
				linear_actuator_storage.sensor_id,
				linear_actuator_storage.actuator_id);
			linear_actuators.push_back(linear_actuator_storage);
			joints.push_back(newjoint);
		}
	}
	
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		for(int j = 0; j < 0; ++j)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0,joints.at(i).poweron_setpoint);
		}
	}
	//print_jointinfo();
	/*for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::LINEAR_ACTUATOR)
		{
			m_model->GetJoints()[joints.at(i).id]->SetPosition(1,.122);
		}
	}
	*/
	{
		bool status = battery.init("555005");
		if(status == false)
		{
			printf("Could not Initialize Battery. Exiting.\n");
			return false;
		}
	}
	double motorcontroller_circuitbreaker_size = 30.0;
	{
		bool status = left_motorcontroller.init("362009",motorcontroller_circuitbreaker_size);
		if(status == false)
		{
			printf("Could not Initialize Left Motor Controller.\n");
			return false;
		}

	}
	{
		std::vector<std::string> left_gearbox;
		left_gearbox.push_back("542026");
		bool status = left_motor.init("361006",left_gearbox,36.0/16.0,motorcontroller_circuitbreaker_size);
		if(status == false)
		{
			printf("Could not Initialize Left Motor.\n");
			return false;
		}
		drivetrain_left_motorcontroller_pin.pin.ParentDevice = "SimulatedRover";
		drivetrain_left_motorcontroller_pin.pin.Name = "LeftMotorController";
		drivetrain_left_motorcontroller_pin.pin.MinValue = 1000;
		drivetrain_left_motorcontroller_pin.pin.MaxValue = 2000;
		drivetrain_left_motorcontroller_pin.pin.DefaultValue = 1500;
		drivetrain_left_motorcontroller_pin.pin.Value = drivetrain_left_motorcontroller_pin.pin.DefaultValue;
	}
	{
		bool status = right_motorcontroller.init("362009",motorcontroller_circuitbreaker_size);
		if(status == false)
		{
			printf("Could not Initialize Right Motor Controller.\n");
			return false;
		}
		drivetrain_right_motorcontroller_pin.pin.ParentDevice = "SimulatedRover";
		drivetrain_right_motorcontroller_pin.pin.Name = "RightMotorController";
		drivetrain_right_motorcontroller_pin.pin.MinValue = 1000;
		drivetrain_right_motorcontroller_pin.pin.MaxValue = 2000;
		drivetrain_right_motorcontroller_pin.pin.DefaultValue = 1500;
		drivetrain_right_motorcontroller_pin.pin.Value = drivetrain_left_motorcontroller_pin.pin.DefaultValue;
	}
	{
		std::vector<std::string> right_gearbox;
		right_gearbox.push_back("542026");
		bool status = right_motor.init("361006",right_gearbox,36.0/16.0,motorcontroller_circuitbreaker_size);
		if(status == false)
		{
			printf("Could not Initialize Right Motor.\n");
			return false;
		}
	}
	if(linear_actuators.size() == 0)
	{
		printf("Did not find any Linear Actuators. Exiting.\n");
		return false;
	}
	return true;
}
bool RobotPlugin::InitializeSubscriptions()
{
	{
		this->sub_keyboardevent = node->Subscribe("~/keyboard/keypress",&RobotPlugin::KeyboardEventCallback,this,true);
	}
	{
		ros::SubscribeOptions so =
				ros::SubscribeOptions::create<eros::pin>(
						"/LeftMotorController",
						1,
						boost::bind(&RobotPlugin::drivetrain_left_cmd, this, _1),
						ros::VoidPtr(), &this->rosQueue);
		sub_drivetrain_left_cmd = this->rosNode->subscribe(so);
	}
	{
		ros::SubscribeOptions so =
				ros::SubscribeOptions::create<eros::pin>(
						"/RightMotorController",
						1,
						boost::bind(&RobotPlugin::drivetrain_right_cmd, this, _1),
						ros::VoidPtr(), &this->rosQueue);
		sub_drivetrain_right_cmd = this->rosNode->subscribe(so);
	}


	this->rosQueueThread =
			std::thread(std::bind(&RobotPlugin::QueueThread, this));

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&RobotPlugin::OnUpdate, this));
	
	for(std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		std::string command_name = linear_actuators.at(i).linear_actuator.get_commandname();
		printf("sub: %s\n",command_name.c_str());
		ros::SubscribeOptions so =
				ros::SubscribeOptions::create<eros::pin>(
						"/" + command_name,
						1,
						boost::bind(&RobotPlugin::implement_cmd, this, _1),
						ros::VoidPtr(), &this->rosQueue);
		ros::Subscriber sub = this->rosNode->subscribe(so);
		linear_actuators.at(i).command_sub = sub;
	}
	return true;
}
bool RobotPlugin::InitializePublications()
{
	pub_truthpose = this->rosNode->advertise<eros::pose>("/TruthPose_Simulated",1);
	pub_batteryinfo = this->rosNode->advertise<eros::battery>("/MainBattery",1);
	if(sensors_enabled == true)
	{
		pub_leftimu = this->rosNode->advertise<eros::imu>("/LeftIMU_Simulated",1);
		pub_rightimu = this->rosNode->advertise<eros::imu>("/RightIMU_Simulated",1);
		pub_leftwheelencoder = this->rosNode->advertise<eros::signal>("/LeftWheelEncoder_Simulated",1);
		pub_rightwheelencoder = this->rosNode->advertise<eros::signal>("/RightWheelEncoder_Simulated",1);
		pub_drivetrain_left_cmd = this->rosNode->advertise<eros::pin>("/LeftMotorController",1);
		pub_drivetrain_right_cmd = this->rosNode->advertise<eros::pin>("/RightMotorController",1);
	}
	for(std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		eros::signal current_signal = linear_actuators.at(i).linear_actuator.get_currentsignal();
		ros::Publisher current_pub = this->rosNode->advertise<eros::signal>("/" + current_signal.name,1);
		linear_actuators.at(i).current_pub = current_pub;
	}
	return true;
}
bool RobotPlugin::LoadSensors()
{
	if(sensors_enabled == true)
	{
	left_imu = initialize_imu("110015","left");
	if(left_imu.initialized == false)
	{
		return false;
	}
	right_imu = initialize_imu("110015","right");
	if(right_imu.initialized == false)
	{
		return false;
	}
	}
	return true;
}
RobotPlugin::IMUStorage RobotPlugin::initialize_imu(std::string partnumber,std::string location)
{
	IMUStorage m_imu;
	m_imu.initialized = false;
	m_sensorManager = sensors::SensorManager::Instance();
	if(m_sensorManager ==nullptr)
	{
		m_imu.initialized = false;
		return m_imu;
	}
	std::string main_imu_name;
	std::string main_magnetometer_name;
	std::string topic_name;
	if(location == "left")
	{
		main_imu_name = "left_imu";
		main_magnetometer_name = "left_imu_mag";
		topic_name = "/LeftIMU";
		ignition::math::Pose3d link_pose;
		bool v = readLinkPose("LeftIMU",&link_pose);
		if(v == false)
		{
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.sensor.init(partnumber,"LeftIMU");
		m_imu.sensor.set_pose(link_pose);
		std::cout << "LeftIMU ACC Rotation Matrix:" << std::endl << 
			m_imu.sensor.GetRotationMatrix_Acceleration() << std::endl;
	}
	else if(location == "right")
	{
		main_imu_name = "right_imu";
		main_magnetometer_name = "right_imu_mag";
		topic_name = "/RightIMU";
		ignition::math::Pose3d link_pose;
		bool v = readLinkPose("RightIMU",&link_pose);
		if(v == false)
		{
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.sensor.init(partnumber,"RightIMU");
		m_imu.sensor.set_pose(link_pose);
		std::cout << "RightIMU ACC Rotation Matrix:" << std::endl << 
			m_imu.sensor.GetRotationMatrix_Acceleration() << std::endl;
	}
	else
	{
		m_imu.initialized = false;
		return m_imu;
	}
	{
		sensors::SensorPtr _sensor = m_sensorManager->GetSensor(main_imu_name);
		if(_sensor == nullptr)
		{
			printf("ERROR: Could not load %s IMU.  Exiting.\n",main_imu_name.c_str());
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.m_gazebo_imu = dynamic_pointer_cast<sensors::ImuSensor,sensors::Sensor>(_sensor);
	}
	{
		sensors::SensorPtr _sensor = m_sensorManager->GetSensor(main_magnetometer_name);
		if(_sensor == nullptr)
		{
			printf("ERROR: Could not load %s IMU Magnetometer.  Exiting.\n",main_magnetometer_name.c_str());
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.m_gazebo_imu_mag = dynamic_pointer_cast<sensors::MagnetometerSensor,sensors::Sensor>(_sensor);
	}
	
	
	last_pose = truth_pose.sensor.get_pose();
	m_imu.initialized = true;
	return m_imu;
}
//Update Functions
void RobotPlugin::QueueThread()
{
	static const double timeout = 0.01;
	while ((this->rosNode->ok()) and (kill_node == false))
	{
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}
void RobotPlugin::OnUpdate()
{
	if(m_fastloop.run_loop())
	{
		for(std::size_t i = 0; i < linear_actuators.size(); ++i)
		{
			//linear_actuators.at(i).linear_actuator.set_targetforce(-30.0);  //KEEP FOR DEBUGGING
		}
		/*
		for(std::size_t i = 0; i < joints.size(); ++i)
		{
			if(joints.at(i).joint_type == JointType::LINEAR_ACTUATOR)
			{
				//printf("setting: %d\n",joints.at(i).id);
				//m_model->GetJoints()[joints.at(i).id]->SetForce(0,-20.0);
				//m_model->GetJoints()[joints.at(i).id]->SetForce(1,-30.0);
				//m_model->GetJoints()[joints.at(i).id]->SetForce(2,-30.0);
			}
		}
		*/
		
		battery.recharge_complete();
		run_time+=m_fastloop.get_timedelta();
		if(run_time > INITIALIZATION_TIME)
		{
			if(robot_initialized == false)
			{
				m_fastloop.enable_printing();
				m_mediumloop.enable_printing();
				m_slowloop.enable_printing();
				m_veryslowloop.enable_printing();
				printf("Robot Initialized.\n");
			}
			robot_initialized = true;
		}
		left_motorcontroller.set_batteryvoltage(battery.get_voltage());
		right_motorcontroller.set_batteryvoltage(battery.get_voltage());
		ignition::math::Pose3d pose = m_model->WorldPose();
		if(pose_initialized == false)
		{
			initial_pose = pose;
			pose_initialized = true;
		}
		if(drivecommand_received == false)
		{
			double d = compute_distance(pose,initial_pose);
			if(d > 1)
			{
				kill_node = true;
				printf("Model Pose has drifted with no input command.  Likely a model sdf problem.  Please adjust\n");
				return;
			}
			
		}
		
		for(std::size_t i = 0; i < joints.size(); ++i)
		{
			if(joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
			{
				if(std::string::npos != joints.at(i).name.find("front"))
				{
					drivetrain_left_actual_velocity = m_model->GetJoints()[joints.at(i).id]->GetVelocity(0);
				}
			}
			else if(joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
			{
				if(std::string::npos != joints.at(i).name.find("front"))
				{
					drivetrain_right_actual_velocity = m_model->GetJoints()[joints.at(i).id]->GetVelocity(0);
				}
				/*printf("%s %f %f %f\n",
					joints.at(i).name.c_str(),
					m_model->GetJoints()[joints.at(i).id]->GetVelocity(0),
					m_model->GetJoints()[joints.at(i).id]->GetVelocity(1),
					m_model->GetJoints()[joints.at(i).id]->GetVelocity(2));
					*/
			}
			
			
		}
		double left_torque = 0.0;
		double right_torque = 0.0;
		for(std::size_t i = 0; i < links.size(); ++i)
		{
			if(links.at(i).link_type == LinkType::DRIVETRAIN_LEFT)
			{
				left_torque+=m_model->GetLink(links.at(i).name)->RelativeTorque().X();
			}
			if(links.at(i).link_type == LinkType::DRIVETRAIN_RIGHT)
			{
				right_torque+=m_model->GetLink(links.at(i).name)->RelativeTorque().X();
			}
		}
		//printf("Left: %f/%f Right: %f/%f\n",left_torque,drivetrain_left_actual_velocity*60.0/(2*M_PI),right_torque,drivetrain_right_actual_velocity*60.0/(2*M_PI));
		if(robot_initialized == true)
		{
			bool status = truth_pose.sensor.update_worldpose(
				m_fastloop.get_currentTime(),
				m_model->GetLink(base_link)->WorldPose(),
				m_model->GetLink(base_link)->WorldAngularVel());
			if(status == false)
			{
				kill_node = false;
			}
			pub_truthpose.publish(truth_pose.sensor.get_pose());
			DrivePerc cmd = arcade_mix(cmd_throttle,cmd_steer);	
			double left_sv = scale_value(cmd.left,1000.0,1500.0,2000.0);
			drivetrain_left_motorcontroller_pin.pin.Value = (int32_t)left_sv;
			pub_drivetrain_left_cmd.publish(drivetrain_left_motorcontroller_pin.pin);
			double right_sv = scale_value(cmd.right,1000.0,1500.0,2000.0);
			drivetrain_right_motorcontroller_pin.pin.Value = (int32_t)right_sv;
			pub_drivetrain_right_cmd.publish(drivetrain_right_motorcontroller_pin.pin);
			

		}
		if((sensors_enabled == true) and (robot_initialized == true))
		{
			pub_leftimu.publish(left_imu.sensor.update_IMU(
				m_fastloop.get_currentTime(),
				left_imu.m_gazebo_imu->LastUpdateTime().Double(),
				left_imu.m_gazebo_imu_mag->LastUpdateTime().Double(),
				left_imu.m_gazebo_imu->LinearAcceleration(),
				left_imu.m_gazebo_imu->AngularVelocity(),
				left_imu.m_gazebo_imu_mag->MagneticField()));
			pub_rightimu.publish(right_imu.sensor.update_IMU(
				m_fastloop.get_currentTime(),
				right_imu.m_gazebo_imu->LastUpdateTime().Double(),
				right_imu.m_gazebo_imu_mag->LastUpdateTime().Double(),
				right_imu.m_gazebo_imu->LinearAcceleration(),
				right_imu.m_gazebo_imu->AngularVelocity(),
				right_imu.m_gazebo_imu_mag->MagneticField()));
			pub_leftwheelencoder.publish(left_wheelencoder.sensor.update(m_fastloop.get_currentTime(),drivetrain_left_actual_velocity));
			pub_rightwheelencoder.publish(right_wheelencoder.sensor.update(m_fastloop.get_currentTime(),drivetrain_right_actual_velocity));
		}
		if((robot_initialized == true))
		{
			for(std::size_t i = 0; i < linear_actuators.size(); ++i)
			{
				ignition::math::Pose3d parent_pose = m_model->GetJoints()[linear_actuators.at(i).sensor_id]->GetParent()->WorldPose();
				ignition::math::Pose3d child_pose = m_model->GetJoints()[linear_actuators.at(i).sensor_id]->GetChild()->WorldPose();
				double parent_child_distance = compute_distance(parent_pose,child_pose);
				linear_actuators.at(i).linear_actuator.update(m_fastloop.get_currentTime(),
					battery.get_voltage(),parent_child_distance);
				linear_actuators.at(i).current_pub.publish(linear_actuators.at(i).linear_actuator.get_currentsignal());
				m_model->GetJoints()[linear_actuators.at(i).actuator_id]->SetForce(0,linear_actuators.at(i).linear_actuator.get_targetforce());
				//printf("act: %s %d\n",linear_actuators.at(i).linear_actuator.get_commandname().c_str(),linear_actuators.at(i).joint_id);
				//m_model->GetJoints()[linear_actuators.at(i).joint_id]->SetForce(1,50.0);
			}
		}
	}
	if(m_mediumloop.run_loop())
	{
		double current_consumed = 0.0;
		current_consumed+= left_motorcontroller.get_currentconsumed();
		current_consumed+= right_motorcontroller.get_currentconsumed();
		current_consumed+= right_motorcontroller.get_currentconsumed();
		current_consumed+= left_imu.sensor.get_currentconsumed();
		current_consumed+= right_imu.sensor.get_currentconsumed();
		current_consumed+= left_wheelencoder.sensor.get_currentconsumed();
		current_consumed+= right_wheelencoder.sensor.get_currentconsumed();
		current_consumed+= left_motor.get_currentconsumed();
		current_consumed+= right_motor.get_currentconsumed();
		if(battery.update(m_mediumloop.get_timedelta(),current_consumed) == false)
		{
			printf("[WARN]: BATTERY DEPLETED!\n");
		}
		pub_batteryinfo.publish(battery.get_batteryinfo());
		m_fastloop.check_looprate();
		m_mediumloop.check_looprate();
		m_slowloop.check_looprate();
		m_veryslowloop.check_looprate();
	}
	if(m_slowloop.run_loop())
	{
		battery.print_info();
		
		for(std::size_t i = 0; i < linear_actuators.size(); ++i)
		{
			linear_actuators.at(i).linear_actuator.print_info();
		}
		
		//print_jointinfo(true);
	}
	if(m_veryslowloop.run_loop())
	{
		//print_loopstates(m_veryslowloop);
		//print_loopstates(m_slowloop);
		//print_loopstates(m_mediumloop);
		//print_loopstates(m_fastloop);
	}
}
void RobotPlugin::print_jointinfo(bool all_joints)
{
	printf("--- JOINT INFO ---\n");
	if(all_joints == false)
	{
		for(std::size_t i = 0; i < joints.size(); ++i)
		{
			ignition::math::Pose3d parent_pose = m_model->GetJoints()[joints.at(i).id]->GetParent()->WorldPose();
			ignition::math::Pose3d child_pose = m_model->GetJoints()[joints.at(i).id]->GetChild()->WorldPose();
			ignition::math::Vector3d parent_link_force = m_model->GetJoints()[joints.at(i).id]->GetParent()->RelativeForce();
			ignition::math::Vector3d child_link_force = m_model->GetJoints()[joints.at(i).id]->GetChild()->RelativeForce();
			double parent_child_distance = compute_distance(parent_pose,child_pose);
			printf("[%d/%d] Joint: (%d)%s\n"
					"\tVel X: %4.2f Y:%4.2f Z: %4.2f\n"
					"\tPos X: %4.2f Y: %4.2f Z: %4.2f\n"
					"\tDist: %4.4f\n"
					"\tParent Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n"
					"\tChild Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n",
				(int)i+1,(int)joints.size(),
				m_model->GetJoints()[joints.at(i).id]->GetType(),
				joints.at(i).name.c_str(),
				m_model->GetJoints()[joints.at(i).id]->GetVelocity(0),
				m_model->GetJoints()[joints.at(i).id]->GetVelocity(1),
				m_model->GetJoints()[joints.at(i).id]->GetVelocity(2),
				m_model->GetJoints()[joints.at(i).id]->Position(0),
				m_model->GetJoints()[joints.at(i).id]->Position(1),
				m_model->GetJoints()[joints.at(i).id]->Position(2),
				parent_child_distance,
				parent_link_force.X(),
				parent_link_force.Y(),
				parent_link_force.Z(),
				compute_magnitude(parent_link_force),
				child_link_force.X(),
				child_link_force.Y(),
				child_link_force.Z(),
				compute_magnitude(child_link_force));
				//m_model->GetJoints()[joints.at(i).id]->GetForce(0),
				//m_model->GetJoints()[joints.at(i).id]->GetForce(1),
				//m_model->GetJoints()[joints.at(i).id]->GetForce(2));//link_force.X(),link_force.Y(),link_force.Z());
		}
	}
	else
	{
		for(uint16_t i = 0; i < m_model->GetJointCount(); ++i)
		{
			ignition::math::Pose3d parent_pose = m_model->GetJoints()[i]->GetParent()->WorldPose();
			ignition::math::Pose3d child_pose = m_model->GetJoints()[i]->GetChild()->WorldPose();
			ignition::math::Vector3d parent_link_force = m_model->GetJoints()[i]->GetParent()->RelativeForce();
			ignition::math::Vector3d child_link_force = m_model->GetJoints()[i]->GetChild()->RelativeForce();
			double parent_child_distance = compute_distance(parent_pose,child_pose);
			printf("[%d/%d] Joint: (%d)%s\n"
					"\tVel X: %4.2f Y:%4.2f Z: %4.2f\n"
					"\tPos X: %4.2f Y: %4.2f Z: %4.2f\n"
					"\tDist: %4.4f\n"
					"\tParent Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n"
					"\tChild Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n",
				(int)i+1,m_model->GetJointCount(),
				m_model->GetJoints()[i]->GetType(),
				m_model->GetJoints()[i]->GetScopedName().c_str(),
				m_model->GetJoints()[i]->GetVelocity(0),
				m_model->GetJoints()[i]->GetVelocity(1),
				m_model->GetJoints()[i]->GetVelocity(2),
				m_model->GetJoints()[i]->Position(0),
				m_model->GetJoints()[i]->Position(1),
				m_model->GetJoints()[i]->Position(2),
				parent_child_distance,
				parent_link_force.X(),
				parent_link_force.Y(),
				parent_link_force.Z(),
				compute_magnitude(parent_link_force),
				child_link_force.X(),
				child_link_force.Y(),
				child_link_force.Z(),
				compute_magnitude(child_link_force));
		}
	}
}
//Communication Functions
void RobotPlugin::implement_cmd(const eros::pin::ConstPtr& _msg)
{
	bool found = false;
	for(std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		if(_msg->ConnectedDevice == linear_actuators.at(i).linear_actuator.get_commandname())
		{
			found = true;
			eros::pin pin;
			pin.Name = _msg->Name;
			pin.ParentDevice = _msg->ParentDevice;
			pin.Function = _msg->Function;
			pin.Value = _msg->Value;
			pin.DefaultValue = _msg->DefaultValue;
			pin.MaxValue = _msg->MaxValue;
			pin.MinValue = _msg->MinValue;
			pin.ConnectedDevice = _msg->ConnectedDevice;
			pin.ConnectedSensor = _msg->ConnectedSensor;
			pin.AuxTopic = _msg->AuxTopic;
			pin.ScaleFactor = _msg->ScaleFactor;
			linear_actuators.at(i).linear_actuator.set_targetpinvalue(pin);
		}
	}
	if(found == false)
	{
		printf("[WARN] Couldn't parse pin: %s\n",_msg->ConnectedDevice.c_str());
	}
}
void RobotPlugin::drivetrain_left_cmd(const eros::pin::ConstPtr& _msg)
{
	drivecommand_received = true;
	double left_voltage = left_motorcontroller.set_input(_msg->Value);
	left_cmd = left_motor.set_input(left_voltage);
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0,left_cmd);
		}
	}

}

void RobotPlugin::drivetrain_right_cmd(const eros::pin::ConstPtr& _msg)
{
	drivecommand_received = true;
	double right_voltage = right_motorcontroller.set_input(_msg->Value);
	right_cmd = right_motor.set_input(right_voltage);
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		if(joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0,right_cmd);
		}
	}
}




//Utility Functions
void RobotPlugin::print_loopstates(SimpleTimer timer)
{
	printf("%s: Error: %4.2f%% Target Rate: %4.2f Actual Rate: %4.2f Set Rate: %4.2f\n",timer.get_name().c_str(),fabs(timer.get_timingerrorperc()),
			timer.get_rate(),timer.get_actualrate(),timer.get_setrate());
}
void RobotPlugin::print_model()
{
	printf("--- JOINTS ---\n");
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		printf("[%d] Type: %s Name: %s\n",
				joints.at(i).id,
				map_jointtype_tostring(joints.at(i).joint_type).c_str(),
				joints.at(i).name.c_str());
	}
	printf("--- LINKS ---\n");
	for(std::size_t i = 0; i < links.size(); ++i)
	{
		printf("[%d] Name: %s\n",
			links.at(i).id,
			links.at(i).name.c_str());
		std::cout << "\tPose: " << m_model->GetLink(links.at(i).name)->RelativePose() << std::endl;
	}
}
std::string RobotPlugin::map_jointtype_tostring(JointType joint_type)
{
	switch(joint_type)
	{
	case JointType::LINEAR_ACTUATOR:
		return "LINEAR ACTUATOR";
		break;
	case JointType::DRIVETRAIN_LEFT:
		return "DRIVETRAIN LEFT";
		break;
	case JointType::DRIVETRAIN_RIGHT:
		return "DRIVETRAIN RIGHT";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}
//Debug
double RobotPlugin::scale_value(double input_perc,double y1,double neutral,double y2)
{
	double m_upper,m_lower=0.0;
	double out_upper,out_lower,out=0.0;
	m_upper = (y2-neutral)/100.0;
	m_lower = (neutral-y1)/100.0;
	out_upper = (m_upper*input_perc) + neutral;
	out_lower = (m_lower*input_perc) + neutral;
	if(input_perc >= 0.0)
	{
		out = out_upper;
	}
	else
	{
		out = out_lower;
	}
	return out;
}
double RobotPlugin::compute_magnitude(ignition::math::Vector3d a)
{
	double d = sqrt((a.X()*a.X()) + (a.Y()*a.Y()) + (a.Z()*a.Z()));
	return d;
}
double RobotPlugin::compute_distance(ignition::math::Pose3d a, ignition::math::Pose3d b)
{
	double dx = a.Pos().X()-b.Pos().X();
	double dy = a.Pos().Y()-b.Pos().Y();
	double dz = a.Pos().Z()-b.Pos().Z();
	double d = sqrt((dx*dx)+(dy*dy)+(dz*dz));
	return d;
}
bool RobotPlugin::readLinkPose(std::string shortname,ignition::math::Pose3d* pose)
{
	bool found = false;
	auto t_links = m_model->GetLinks();
	for(std::size_t i = 0; i < t_links.size(); ++i)
	{	
		std::string name = m_model->GetLinks()[i]->GetScopedName();
		if(std::string::npos != name.find(shortname))
		{
			found = true;
			*pose = m_model->GetLink(name)->RelativePose();
		}
	}
	return found;

	//m_model->GetLink(links.at(i).name)->GetRelativePose()
}
RobotPlugin::DrivePerc RobotPlugin::arcade_mix(double throttle_perc,double steer_perc)
{
	DrivePerc d;
	double v =(100.0-fabs(steer_perc)) * (throttle_perc/100.0) + throttle_perc;
	double w= (100.0-fabs(throttle_perc)) * (steer_perc/100.0) + steer_perc;
	d.left = (v+w)/2.0;
	d.right = (v-w)/2.0;
	if(d.left > 100.0) { d.left = 100.0; }
	if(d.left < -100.0) { d.right = -100.0; }
	if(d.right > 100.0) { d.right = 100.0; }
	if(d.right < -100.0) { d.right = -100.0; }
	return d;
}
void RobotPlugin::KeyboardEventCallback(ConstAnyPtr &_msg)
{
	switch(_msg->int_value())
	{
		case KEYCODE_DOWNARROW:
			drivecommand_received = true;
			cmd_throttle-=5.0;
			if(cmd_throttle < -100.0)
			{
				cmd_throttle = -100.0;
			}
			break;
		case KEYCODE_LEFTARROW:
			drivecommand_received = true;
			cmd_steer-=5.0;
			if(cmd_steer < -100.0)
			{
				cmd_steer = -100.0;
			}
			break;
		case KEYCODE_RIGHTARROW:
			drivecommand_received = true;
			cmd_steer+=5.0;
			if(cmd_steer > 100.0)
			{
				cmd_steer = 100.0;
			}
			break;
		case KEYCODE_UPARROW:
			drivecommand_received = true;
			cmd_throttle+=5.0;
			if(cmd_throttle > 100.0)
			{
				cmd_throttle = 100.0;
			}
			break;
		case KEYCODE_SPACE:
			
			break;
		case KEYCODE_ENTER:
			drivecommand_received = true;
			cmd_throttle = 0.0;
			cmd_steer = 0.0;
			break;
		default:
			break;
	}
}