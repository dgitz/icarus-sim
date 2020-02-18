/*
 * RobotPlugin.cpp
 *
 *  Created on: Mar 10, 2019
 *      Author: robot
 */

#include "RobotPlugin.h"
using namespace gazebo;
using namespace std;
RobotPlugin::RobotPlugin() : ModelPlugin(),
							 m_model(nullptr),
							 m_sensorManager(nullptr)
//m_left_imu(nullptr),
//m_right_imu(nullptr)
{

	last_simtime = 0.0;
	last_iterationcount = 0;
	left_cmd = 0.0;
	right_cmd = 0.0;
	drivetrain_left_actual_velocity = 0.0;
	drivetrain_right_actual_velocity = 0.0;
	kill_node = false;
	robot_initialized = false;
	run_time = 0.0;
	compute_average.init();
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SENSORS);
	diagnostic_types.push_back(ACTUATORS);
	diagnostic_types.push_back(POSE);
	enable_diagnostics(diagnostic_types);
	// TODO Auto-generated constructor stub
}

RobotPlugin::~RobotPlugin()
{
	// TODO Auto-generated destructor stub
}

//Initialize Functions
void RobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	printf("Starting Plugin\n");
	m_model = _model;
	pose_initialized = false;
	drivecommand_received = false;
	if (InitializePlugin() == false)
	{
	}

	return;
}
bool RobotPlugin::InitializePlugin()
{
	eros::diagnostic diag = root_diagnostic;
	std::string node_name = "gazebo_client";
	heartbeat.BaseNode_Name = "gazebo";
	heartbeat.Node_Name = node_name;
	logger = new Logger("DEBUG", "/" + node_name);
	logger_initialized = true;
	if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == true)
	{
		logger->log_warn(__FILE__, __LINE__, "ALLOWING MODEL TO INITIALIZE EVEN IF DESCRIPTION FILES ARE INCOMPLETE/INVALID.");
	}
	sensors_enabled = true;
	if (LoadModel() == false)
	{

		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			logger->log_error(__FILE__, __LINE__, "Model Failed to Load. Exiting.");
			return false;
		}
		diag = update_diagnostic(DATA_STORAGE, WARN, INITIALIZING_ERROR, "Simulation Failed to Load Correctly but Continuing anyways.");
		logger->log_diagnostic(diag);
	}
	print_model();
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(m_model->GetName());
	logger->log_debug(__FILE__, __LINE__, "Starting ros");
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
	}

	rosNode.reset(new ros::NodeHandle(node_name));
	if (InitializeSubscriptions() == false)
	{
		logger->log_error(__FILE__, __LINE__, "Not able to initialize Subscriptions. Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	if (InitializePublications() == false)
	{
		logger->log_error(__FILE__, __LINE__, "Not able to initialize Publications. Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	m_fastloop.set_name("FASTLOOP");
	m_fastloop.set_targetrate(50.0);
	m_20hzloop.set_name("20HzLOOP");
	m_20hzloop.set_targetrate(20.0);
	m_mediumloop.set_name("MEDIUMLOOP");
	m_mediumloop.set_targetrate(10.0);
	m_slowloop.set_name("SLOWLOOP");
	m_slowloop.set_targetrate(1.0);
	m_veryslowloop.set_name("VERYSLOWLOOP");
	m_veryslowloop.set_targetrate(0.1);

	logger->log_notice(__FILE__, __LINE__, "Plugin Started.  Waiting " + std::to_string(INITIALIZATION_TIME) + " seconds from Start before Initialization is complete.");
	return true;
}
bool RobotPlugin::LoadModel()
{
	eros::diagnostic diag = root_diagnostic;
	logger->log_notice(__FILE__, __LINE__, "Initializing Model.");
	if (LoadSensors() == false)
	{
		logger->log_error(__FILE__, __LINE__, "Could not load sensors. Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	if (m_model->GetJointCount() == 0)
	{
		logger->log_error(__FILE__, __LINE__, "Robot Model did not receive any info.  Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	auto t_links = m_model->GetLinks();
	bool found_baselink = false;
	for (std::size_t i = 0; i < t_links.size(); ++i)
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
		if (newlink.name.find("base") != std::string::npos)
		{
			base_link = newlink.name;
			found_baselink = true;
		}
	}
	if (found_baselink == false)
	{
		logger->log_error(__FILE__, __LINE__, "Could not find link: base. Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	int16_t camera_panjoint_id = -1;
	int16_t camera_tiltjoint_id = -1;
	for (uint16_t i = 0; i < m_model->GetJointCount(); ++i)
	{
		printf("[%d/%d] Joint Name: %s\n", i, m_model->GetJointCount() - 1, m_model->GetJoints()[i]->GetScopedName().c_str());
		if (m_model->GetJoints()[i]->GetScopedName().find("CameraBase_CameraPan") != std::string::npos)
		{
			camera_panjoint_id = i;
		}
		if (m_model->GetJoints()[i]->GetScopedName().find("CameraPan_CameraTilt") != std::string::npos)
		{
			camera_tiltjoint_id = i;
		}
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
				if (left_wheelencoder.sensor.is_initialized() == false)
				{
					if (left_wheelencoder.sensor.init("110003", "LeftEncoder") == false)
					{
						logger->log_error(__FILE__, __LINE__, "Could not initialize LeftEncoder.  Exiting.");
						if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
						{
							return false;
						}
					}
					else
					{
						logger->log_notice(__FILE__, __LINE__, "Loaded Wheel Encoder Sensor at Location: left.");
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
				if (right_wheelencoder.sensor.is_initialized() == false)
				{
					if (right_wheelencoder.sensor.init("110003", "RightEncoder") == false)
					{
						logger->log_error(__FILE__, __LINE__, "Could not initialize RightEncoder.  Exiting.");
						if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
						{
							return false;
						}
					}
					else
					{
						logger->log_notice(__FILE__, __LINE__, "Loaded Wheel Encoder Sensor at Location: right.");
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
			if (joint_scopedname == "sim_scout::actuator_bucket_implementlink_joint")
			{
				newjoint.name = "ImplementCylinder";
			}
			else if (joint_scopedname == "sim_scout::actuator_left_liftarm_base_joint")
			{
				newjoint.name = "LiftCylinder";
			}
			else
			{
				logger->log_error(__FILE__, __LINE__, "Unable to parse Joint: " + joint_scopedname + " Exiting.");
				if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
				{
					return false;
				}
			}
			bool status = actuator.init(PN_361008, newjoint.name, joint_scopedname);
			if (status == false)
			{
				logger->log_error(__FILE__, __LINE__, "Could not Initialize Linear Actuator: " + newjoint.name + " Exiting.");
				if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
				{
					return false;
				}
			}
			LinearActuatorStorage linear_actuator_storage;
			linear_actuator_storage.linear_actuator = actuator;
			linear_actuator_storage.initialized = false;
			linear_actuator_storage.actuator_id = newjoint.id;
			for (uint16_t j = 0; j < m_model->GetJointCount(); ++j)
			{
				if (m_model->GetJoints()[j]->GetScopedName().find("sensor") != std::string::npos)
				{
					std::string tempstr = m_model->GetJoints()[j]->GetScopedName();
					if ((newjoint.name == "LiftCylinder") && (tempstr == "sim_scout::sensor_left_liftcylinder_prism"))
					{
						linear_actuator_storage.sensor_id = j;
					}
					else if ((newjoint.name == "ImplementCylinder") && (tempstr == "sim_scout::sensor_implementcylinder_implementbase_prism"))
					{
						linear_actuator_storage.sensor_id = j;
					}
				}
			}
			char tempstr[512];

			sprintf(tempstr, "Created new Joint: %s (Real Name: %s) Sensor ID: %ld Actuator ID: %ld",
					newjoint.name.c_str(),
					joint_scopedname.c_str(),
					linear_actuator_storage.sensor_id,
					linear_actuator_storage.actuator_id);
			logger->log_notice(__FILE__, __LINE__, std::string(tempstr));
			linear_actuators.push_back(linear_actuator_storage);
			joints.push_back(newjoint);
		}
	}
	camera_pantilt = initialize_camerapantilt(camera_panjoint_id, camera_tiltjoint_id);
	if (camera_pantilt.initialized == false)
	{
		logger->log_error(__FILE__, __LINE__, "Could not Initialize Camera Pan Tilt Assembly. Exiting.\n");
		return false;
	}
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		for (int j = 0; j < 0; ++j)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0, joints.at(i).poweron_setpoint);
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
		bool status = battery.init(PN_555005);
		if (status == false)
		{
			logger->log_error(__FILE__, __LINE__, "Could not Initialize Battery. Exiting.");
			if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
			{
				return false;
			}
		}
	}
	double motorcontroller_circuitbreaker_size = 30.0;
	{
		bool status = left_motorcontroller.init(PN_362009, motorcontroller_circuitbreaker_size);
		if (status == false)
		{
			logger->log_error(__FILE__, __LINE__, "Could not Initialize Left Motor Controller.");
			if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
			{
				return false;
			}
		}
	}
	{
		std::vector<std::string> left_gearbox;
		left_gearbox.push_back(PN_542026);
		bool status = left_motor.init(PN_361006, left_gearbox, 36.0 / 16.0, motorcontroller_circuitbreaker_size);
		if (status == false)
		{
			logger->log_error(__FILE__, __LINE__, "Could not Initialize Left Motor.");
			if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
			{
				return false;
			}
		}
		drivetrain_left_motorcontroller_pin.pin.ParentDevice = "SimulatedRover";
		drivetrain_left_motorcontroller_pin.pin.Name = "LeftMotorController";
		drivetrain_left_motorcontroller_pin.pin.MinValue = 1000;
		drivetrain_left_motorcontroller_pin.pin.MaxValue = 2000;
		drivetrain_left_motorcontroller_pin.pin.DefaultValue = 1500;
		drivetrain_left_motorcontroller_pin.pin.Value = drivetrain_left_motorcontroller_pin.pin.DefaultValue;
	}
	{
		bool status = right_motorcontroller.init(PN_362009, motorcontroller_circuitbreaker_size);
		if (status == false)
		{
			logger->log_error(__FILE__, __LINE__, "Could not Initialize Right Motor Controller.");
			if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
			{
				return false;
			}
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
		right_gearbox.push_back(PN_542026);
		bool status = right_motor.init(PN_361006, right_gearbox, 36.0 / 16.0, motorcontroller_circuitbreaker_size);
		if (status == false)
		{
			logger->log_error(__FILE__, __LINE__, "Could not Initialize Right Motor.");
			if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
			{
				return false;
			}
		}
	}
	if (linear_actuators.size() == 0)
	{
		logger->log_error(__FILE__, __LINE__, "Did not find any Linear Actuators. Exiting.");
		if (ALLOW_INCOMPLETEMODEL_INITIALIZATION == false)
		{
			return false;
		}
	}
	diag = update_diagnostic(SENSORS, INFO, NOERROR, "Sensors Loaded.");
	diag = update_diagnostic(ACTUATORS, INFO, NOERROR, "Actuators Loaded.");
	diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "Simulation Loaded.");
	diag = update_diagnostic(POSE, INFO, NOERROR, "No Error.");
	return true;
}
bool RobotPlugin::InitializeSubscriptions()
{

	logger->log_notice(__FILE__,__LINE__,"Subscribing to Channels.");
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind(&RobotPlugin::OnUpdate, this));
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
	{
		ros::SubscribeOptions so =
			ros::SubscribeOptions::create<eros::pin>(
				"/PanServo",
				1,
				boost::bind(&RobotPlugin::panservo_cmd, this, _1),
				ros::VoidPtr(), &this->rosQueue);
		sub_panservo_cmd = this->rosNode->subscribe(so);
	}
	{
		ros::SubscribeOptions so =
			ros::SubscribeOptions::create<eros::pin>(
				"/TiltServo",
				1,
				boost::bind(&RobotPlugin::tiltservo_cmd, this, _1),
				ros::VoidPtr(), &this->rosQueue);
		sub_tiltservo_cmd = this->rosNode->subscribe(so);
	}

	for (std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		std::string command_name = linear_actuators.at(i).linear_actuator.get_commandname();
		logger->log_debug(__FILE__, __LINE__, "Subscribing to: " + command_name);
		ros::SubscribeOptions so =
			ros::SubscribeOptions::create<eros::pin>(
				"/" + command_name,
				1,
				boost::bind(&RobotPlugin::implement_cmd, this, _1),
				ros::VoidPtr(), &this->rosQueue);
		ros::Subscriber sub = this->rosNode->subscribe(so);
		linear_actuators.at(i).command_sub = sub;
	}
	this->rosQueueThread =
		std::thread(std::bind(&RobotPlugin::QueueThread, this));
	return true;
}
bool RobotPlugin::InitializePublications()
{
	pub_heartbeat = this->rosNode->advertise<eros::heartbeat>("/gazebo/heartbeat", 1);
	heartbeat.stamp = ros::Time::now();
	heartbeat.TaskState = TASKSTATE_INITIALIZING;
	pub_heartbeat.publish(heartbeat);
	pub_diagnostic = this->rosNode->advertise<eros::diagnostic>("/gazebo/diagnostic", 1);
	pub_gazebofps = this->rosNode->advertise<std_msgs::Float64>("/gazebo/update_rate", 1);
	pub_truthpose = this->rosNode->advertise<eros::pose>("/TruthPose_Simulated", 1);
	pub_batteryinfo = this->rosNode->advertise<eros::battery>("/MainBattery", 1);
	if (sensors_enabled == true)
	{
		pub_leftimu = this->rosNode->advertise<eros::imu>("/LeftIMU_Simulated", 1);
		pub_rightimu = this->rosNode->advertise<eros::imu>("/RightIMU_Simulated", 1);
		pub_leftwheelencoder = this->rosNode->advertise<eros::odom>("/LeftWheelEncoder_Simulated", 1);
		pub_rightwheelencoder = this->rosNode->advertise<eros::odom>("/RightWheelEncoder_Simulated", 1);
	}
	for (std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		eros::signal current_signal = linear_actuators.at(i).linear_actuator.get_currentsignal();
		ros::Publisher current_pub = this->rosNode->advertise<eros::signal>("/" + current_signal.name, 1);
		linear_actuators.at(i).current_pub = current_pub;
	}
	for (std::size_t i = 0; i < sonar_sensors.size(); ++i)
	{
		if(sonar_sensors.at(i).initialized == true)
		{
			eros::signal current_signal = sonar_sensors.at(i).sensor.get_currentsignal();
			ros::Publisher current_pub = this->rosNode->advertise<eros::signal>("/" + current_signal.name, 1);
			sonar_sensors.at(i).pub = current_pub;
		}
	}
	return true;
}
bool RobotPlugin::LoadSensors()
{
	if (sensors_enabled == true)
	{
		m_sensorManager = sensors::SensorManager::Instance();
		if (m_sensorManager == nullptr)
		{
			return false;
		}
		{	// Load Truth Sensors
			sensors::SensorPtr _sensor = m_sensorManager->GetSensor("truth_imu");
			if (_sensor == nullptr)
			{
				logger->log_error(__FILE__, __LINE__, "Could not load truth_imu.  Exiting.");
				return false;
			}
			truth_pose.m_gazebo_imu = dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(_sensor);

		}
		{
			std::string location = "left";
			left_imu = initialize_imu(PN_110015, location);
			if (left_imu.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded IMU Sensor at Location: " + location + ".");
			}
		}
		{
			std::string location = "right";
			right_imu = initialize_imu(PN_110015, location);
			if (right_imu.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded IMU Sensor at Location: " + location + ".");
			}
		}
		{
			std::string location = "frontleft";
			SonarStorage sonar = initialize_sonar(PN_110001, location);
			if (sonar.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded Sonar Sensor at Location: " + location + ".");
			}
			sonar_sensors.push_back(sonar);
		}
		{
			std::string location = "frontright";
			SonarStorage sonar = initialize_sonar(PN_110001, location);
			if (sonar.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded Sonar Sensor at Location: " + location);
			}
			sonar_sensors.push_back(sonar);
		}
		{
			std::string location = "backleft";
			SonarStorage sonar = initialize_sonar(PN_110001, location);
			if (sonar.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded Sonar Sensor at Location: " + location);
			}
			sonar_sensors.push_back(sonar);
		}
		{
			std::string location = "backright";
			SonarStorage sonar = initialize_sonar(PN_110001, location);
			if (sonar.initialized == false)
			{
				return false;
			}
			else
			{
				logger->log_notice(__FILE__, __LINE__, "Loaded Sonar Sensor at Location: " + location);
			}
			sonar_sensors.push_back(sonar);
		}
	}
	return true;
}
RobotPlugin::CameraPanTiltStorage RobotPlugin::initialize_camerapantilt(int16_t panjoint_id, int16_t tiltjoint_id)
{
	CameraPanTiltStorage storage;
	if ((panjoint_id < 0) || (tiltjoint_id < 0))
	{
		logger->log_error(__FILE__, __LINE__, "Could not Find Camera Pan/Tilt Joints. Exiting.\n");
		storage.initialized = false;
		return storage;
	}

	bool status = storage.assy.init(PN_361005, PN_361005);
	if (status == false)
	{
		storage.initialized = false;
		logger->log_error(__FILE__,__LINE__,"Failed to Initialize Camera Pan/Tilt Assembly.");

	}
	else
	{
		logger->log_notice(__FILE__,__LINE__,"Initialized Camera Pan/Tilt Assembly.");
	}
	storage.panjoint_id = (uint16_t)panjoint_id;
	storage.tiltjoint_id = (uint16_t)tiltjoint_id;
	storage.initialized = true;
	return storage;
}
RobotPlugin::SonarStorage RobotPlugin::initialize_sonar(std::string partnumber, std::string location)
{
	SonarStorage m_sonar;
	m_sonar.initialized = false;
	std::string sonar_name;
	std::string topic_name;
	if (location == "frontleft")
	{
		sonar_name = "flsonar";
		m_sonar.sensor.init(partnumber, "FLSonar");
	}
	else if (location == "frontright")
	{
		sonar_name = "frsonar";
		m_sonar.sensor.init(partnumber, "FRSonar");
	}
	else if (location == "backleft")
	{
		sonar_name = "blsonar";
		m_sonar.sensor.init(partnumber, "BLSonar");
	}
	else if (location == "backright")
	{
		sonar_name = "brsonar";
		m_sonar.sensor.init(partnumber, "BRSonar");
	}
	else
	{
		m_sonar.initialized = false;
		return m_sonar;
	}
	{
		sensors::SensorPtr _sensor = m_sensorManager->GetSensor(sonar_name);
		if (_sensor == nullptr)
		{
			logger->log_error(__FILE__, __LINE__, "Could not load " + sonar_name + " Sonar.  Exiting.");
			m_sonar.initialized = false;
			return m_sonar;
		}
		m_sonar.m_gazebo_sonar = dynamic_pointer_cast<sensors::SonarSensor, sensors::Sensor>(_sensor);
	}
	last_pose = truth_pose.sensor.get_pose();
	m_sonar.initialized = true;
	return m_sonar;
}
RobotPlugin::IMUStorage RobotPlugin::initialize_imu(std::string partnumber, std::string location)
{
	IMUStorage m_imu;
	m_imu.initialized = false;
	std::string main_imu_name;
	std::string main_magnetometer_name;
	std::string topic_name;
	if (location == "left")
	{
		main_imu_name = "left_imu";
		main_magnetometer_name = "left_imu_mag";
		topic_name = "/LeftIMU";
		ignition::math::Pose3d link_pose;
		bool v = readLinkPose("LeftIMU", &link_pose);
		if (v == false)
		{
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.sensor.init(partnumber, "LeftIMU");
		m_imu.sensor.set_pose(link_pose);
		std::cout << "LeftIMU ACC Rotation Matrix:" << std::endl
				  << m_imu.sensor.GetRotationMatrix_Acceleration() << std::endl;
	}
	else if (location == "right")
	{
		main_imu_name = "right_imu";
		main_magnetometer_name = "right_imu_mag";
		topic_name = "/RightIMU";
		ignition::math::Pose3d link_pose;
		bool v = readLinkPose("RightIMU", &link_pose);
		if (v == false)
		{
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.sensor.init(partnumber, "RightIMU");
		m_imu.sensor.set_pose(link_pose);
		std::cout << "RightIMU ACC Rotation Matrix:" << std::endl
				  << m_imu.sensor.GetRotationMatrix_Acceleration() << std::endl;
	}
	else
	{
		m_imu.initialized = false;
		return m_imu;
	}
	{
		sensors::SensorPtr _sensor = m_sensorManager->GetSensor(main_imu_name);
		if (_sensor == nullptr)
		{
			logger->log_error(__FILE__, __LINE__, "Could not load " + main_imu_name + " IMU.  Exiting.");
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.m_gazebo_imu = dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(_sensor);
	}
	{
		sensors::SensorPtr _sensor = m_sensorManager->GetSensor(main_magnetometer_name);
		if (_sensor == nullptr)
		{
			logger->log_error(__FILE__, __LINE__, "Could not load " + main_magnetometer_name + " IMU Magnetometer.  Exiting.");
			m_imu.initialized = false;
			return m_imu;
		}
		m_imu.m_gazebo_imu_mag = dynamic_pointer_cast<sensors::MagnetometerSensor, sensors::Sensor>(_sensor);
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
	eros::diagnostic diag = root_diagnostic;
	if (m_fastloop.run_loop())
	{

		for (std::size_t i = 0; i < linear_actuators.size(); ++i)
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
		run_time += m_fastloop.get_timedelta();
		if (run_time > INITIALIZATION_TIME)
		{
			if (robot_initialized == false)
			{
				m_fastloop.enable_printing();
				m_mediumloop.enable_printing();
				m_slowloop.enable_printing();
				m_veryslowloop.enable_printing();
				logger->log_notice(__FILE__, __LINE__, "Robot Initialized.");
			}
			robot_initialized = true;
		}
		left_motorcontroller.set_batteryvoltage(battery.get_voltage());
		right_motorcontroller.set_batteryvoltage(battery.get_voltage());
		ignition::math::Pose3d pose = m_model->WorldPose();
		if (pose_initialized == false)
		{
			initial_pose = pose;
			pose_initialized = true;
		}
		if (drivecommand_received == false)
		{
			double d = compute_distance(pose, initial_pose);
			/*if (d > 1)
			{
				diag = update_diagnostic(POSE, WARN, DIAGNOSTIC_FAILED, "Model Pose has drifted with no input command.  Likely a model sdf problem.  Please adjust.");
				logger->log_diagnostic(diag);
			}
			*/
		}
		for (std::size_t i = 0; i < joints.size(); ++i)
		{
			if (joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
			{
				if (std::string::npos != joints.at(i).name.find("front"))
				{
					drivetrain_left_actual_velocity = m_model->GetJoints()[joints.at(i).id]->GetVelocity(0);
				}
			}
			else if (joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
			{
				if (std::string::npos != joints.at(i).name.find("front"))
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
		for (std::size_t i = 0; i < links.size(); ++i)
		{
			if (links.at(i).link_type == LinkType::DRIVETRAIN_LEFT)
			{
				left_torque += m_model->GetLink(links.at(i).name)->RelativeTorque().X();
			}
			if (links.at(i).link_type == LinkType::DRIVETRAIN_RIGHT)
			{
				right_torque += m_model->GetLink(links.at(i).name)->RelativeTorque().X();
			}
		}
		//printf("Left: %f/%f Right: %f/%f\n",left_torque,drivetrain_left_actual_velocity*60.0/(2*M_PI),right_torque,drivetrain_right_actual_velocity*60.0/(2*M_PI));
		if (robot_initialized == true)
		{
			bool status = truth_pose.sensor.update_worldpose(
				m_fastloop.get_currentTime(),
				m_model->GetLink(base_link)->WorldPose(),
				truth_pose.m_gazebo_imu->LinearAcceleration(),
				m_model->GetLink(base_link)->RelativeLinearVel(),
				truth_pose.m_gazebo_imu->AngularVelocity());
			if (status == false)
			{
				kill_node = false;
			}
			pub_truthpose.publish(truth_pose.sensor.get_pose());
		}
		if ((sensors_enabled == true) and (robot_initialized == true))
		{
			if(left_imu.initialized == true)
			{
			pub_leftimu.publish(left_imu.sensor.update_IMU(
				m_fastloop.get_currentTime(),
				left_imu.m_gazebo_imu->LastUpdateTime().Double(),
				left_imu.m_gazebo_imu_mag->LastUpdateTime().Double(),
				left_imu.m_gazebo_imu->LinearAcceleration(),
				left_imu.m_gazebo_imu->AngularVelocity(),
				left_imu.m_gazebo_imu_mag->MagneticField()));
			}
			if(right_imu.initialized == true)
			{
			pub_rightimu.publish(right_imu.sensor.update_IMU(
				m_fastloop.get_currentTime(),
				right_imu.m_gazebo_imu->LastUpdateTime().Double(),
				right_imu.m_gazebo_imu_mag->LastUpdateTime().Double(),
				right_imu.m_gazebo_imu->LinearAcceleration(),
				right_imu.m_gazebo_imu->AngularVelocity(),
				right_imu.m_gazebo_imu_mag->MagneticField()));
			}
			pub_leftwheelencoder.publish(left_wheelencoder.sensor.update(m_fastloop.get_currentTime(), drivetrain_left_actual_velocity));
			pub_rightwheelencoder.publish(right_wheelencoder.sensor.update(m_fastloop.get_currentTime(), drivetrain_right_actual_velocity));
		}
		if ((robot_initialized == true))
		{
			for (std::size_t i = 0; i < linear_actuators.size(); ++i)
			{
				ignition::math::Pose3d parent_pose = m_model->GetJoints()[linear_actuators.at(i).sensor_id]->GetParent()->WorldPose();
				ignition::math::Pose3d child_pose = m_model->GetJoints()[linear_actuators.at(i).sensor_id]->GetChild()->WorldPose();
				double parent_child_distance = compute_distance(parent_pose, child_pose);
				linear_actuators.at(i).linear_actuator.update(m_fastloop.get_currentTime(),
															  battery.get_voltage(), parent_child_distance);
				linear_actuators.at(i).current_pub.publish(linear_actuators.at(i).linear_actuator.get_currentsignal());
				m_model->GetJoints()[linear_actuators.at(i).actuator_id]->SetForce(0, linear_actuators.at(i).linear_actuator.get_targetforce());
				//printf("act: %s %d\n",linear_actuators.at(i).linear_actuator.get_commandname().c_str(),linear_actuators.at(i).joint_id);
				//m_model->GetJoints()[linear_actuators.at(i).joint_id]->SetForce(1,50.0);
			}
		}
	}
	if (m_mediumloop.run_loop())
	{

		
		heartbeat.TaskState = TASKSTATE_RUNNING;
		heartbeat.stamp = ros::Time::now();
		pub_heartbeat.publish(heartbeat);
		double current_consumed = 0.0;
		current_consumed += left_motorcontroller.get_currentconsumed();
		current_consumed += right_motorcontroller.get_currentconsumed();
		current_consumed += right_motorcontroller.get_currentconsumed();
		if(left_imu.initialized == true)
		{
			current_consumed += left_imu.sensor.get_currentconsumed();
		}
		if(right_imu.initialized == true)
		{
			current_consumed += right_imu.sensor.get_currentconsumed();
		}
		current_consumed += left_wheelencoder.sensor.get_currentconsumed();
		current_consumed += right_wheelencoder.sensor.get_currentconsumed();
		current_consumed += left_motor.get_currentconsumed();
		current_consumed += right_motor.get_currentconsumed();
		for (std::size_t i = 0; i < sonar_sensors.size(); ++i)
		{
			if(sonar_sensors.at(i).initialized == true)
			{
				current_consumed += sonar_sensors.at(i).sensor.get_currentconsumed();
			}
		}

		if (battery.update(m_mediumloop.get_timedelta(), current_consumed) == false)
		{
			logger->log_warn(__FILE__, __LINE__, "BATTERY DEPLETED");
		}
		if(0)
		{  
			if(camera_pantilt.initialized == true)
			{
				// Camera Pan/Tilt Updates
				std::vector<CameraPanTilt::Joint> cam_joints = camera_pantilt.assy.update(m_mediumloop.get_timedelta(),
						m_model->GetJoints()[camera_pantilt.panjoint_id]->Position(0)*180.0/M_PI,
						m_model->GetJoints()[camera_pantilt.tiltjoint_id]->Position(0)*180.0/M_PI);
					m_model->GetJoints()[camera_pantilt.panjoint_id]->SetForce(0, 
						cam_joints.at((std::size_t)CameraPanTilt::JointIndex::JOINT_PAN_INDEX).output_value);
					m_model->GetJoints()[camera_pantilt.tiltjoint_id]->SetForce(0, 
						cam_joints.at((std::size_t)CameraPanTilt::JointIndex::JOINT_TILT_INDEX).output_value);
			}
		}
		pub_batteryinfo.publish(battery.get_batteryinfo());
		m_fastloop.check_looprate();
		m_mediumloop.check_looprate();
		m_20hzloop.check_looprate();
		m_slowloop.check_looprate();
		m_veryslowloop.check_looprate();
	}
	if (m_20hzloop.run_loop())
	{
		for (std::size_t i = 0; i < sonar_sensors.size(); ++i)
		{
			if(sonar_sensors.at(i).initialized == true)
			{
				sonar_sensors.at(i).pub.publish(sonar_sensors.at(i).sensor.update(m_20hzloop.get_currentTime(),
																			  sonar_sensors.at(i).m_gazebo_sonar->Range()));
			}
		}
	}
	if (m_slowloop.run_loop())
	{
		printf("%s\n",pose_helper.get_simplepose_string(truth_pose.sensor.get_pose()).c_str());
		double sim_time = m_model->GetWorld()->RealTime().Double();
		uint32_t count = m_model->GetWorld()->Iterations();
		double update_rate = 0.0;
		if (last_iterationcount > 0)
		{
			double new_value = (double)(count - last_iterationcount) / (sim_time - last_simtime);
			if (new_value > 500.0)
			{
				new_value = 500.0;
			}
			if (new_value < 1.0)
			{
				new_value = 1.0;
			}
			update_rate = compute_average.compute(new_value);
			//printf("xxx1: %f %f %f %d(%d) %f(%f)\n",new_value,update_rate,(double)count/sim_time,count,last_iterationcount,sim_time,last_simtime);
		}
		std_msgs::Float64 v;
		v.data = update_rate;
		pub_gazebofps.publish(v);
		last_simtime = sim_time;
		last_iterationcount = count;

		//printf("count: %d simtime: %f fps: %f\n",count,sim_time,update_rate);
		//battery.print_info();

		for (std::size_t i = 0; i < linear_actuators.size(); ++i)
		{
			linear_actuators.at(i).linear_actuator.print_info();
		}
	}
	if (m_veryslowloop.run_loop())
	{
		std::vector<eros::diagnostic> diaglist = get_diagnostics();
		for (std::size_t i = 0; i < diaglist.size(); ++i)
		{
			logger->log_diagnostic(diaglist.at(i));
			pub_diagnostic.publish(diaglist.at(i));
		}
		//print_loopstates(m_veryslowloop);
		//print_loopstates(m_slowloop);
		//print_loopstates(m_mediumloop);
		//print_loopstates(m_fastloop);
	}
}
void RobotPlugin::print_jointinfo(bool all_joints)
{
	char tempstr[2048 * (uint16_t)joints.size()];
	sprintf(tempstr, "--- JOINT INFO ---\n");
	if (all_joints == false)
	{
		for (std::size_t i = 0; i < joints.size(); ++i)
		{
			ignition::math::Pose3d parent_pose = m_model->GetJoints()[joints.at(i).id]->GetParent()->WorldPose();
			ignition::math::Pose3d child_pose = m_model->GetJoints()[joints.at(i).id]->GetChild()->WorldPose();
			ignition::math::Vector3d parent_link_force = m_model->GetJoints()[joints.at(i).id]->GetParent()->RelativeForce();
			ignition::math::Vector3d child_link_force = m_model->GetJoints()[joints.at(i).id]->GetChild()->RelativeForce();
			double parent_child_distance = compute_distance(parent_pose, child_pose);
			sprintf(tempstr, "%s[%d/%d] Joint: (%d)%s\n"
							 "\tVel X: %4.2f Y:%4.2f Z: %4.2f\n"
							 "\tPos X: %4.2f Y: %4.2f Z: %4.2f\n"
							 "\tDist: %4.4f\n"
							 "\tParent Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n"
							 "\tChild Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n",
					tempstr,
					(int)i + 1, (int)joints.size(),
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
		for (uint16_t i = 0; i < m_model->GetJointCount(); ++i)
		{
			ignition::math::Pose3d parent_pose = m_model->GetJoints()[i]->GetParent()->WorldPose();
			ignition::math::Pose3d child_pose = m_model->GetJoints()[i]->GetChild()->WorldPose();
			ignition::math::Vector3d parent_link_force = m_model->GetJoints()[i]->GetParent()->RelativeForce();
			ignition::math::Vector3d child_link_force = m_model->GetJoints()[i]->GetChild()->RelativeForce();
			double parent_child_distance = compute_distance(parent_pose, child_pose);
			sprintf(tempstr, "%s[%d/%d] Joint: (%d)%s\n"
							 "\tVel X: %4.2f Y:%4.2f Z: %4.2f\n"
							 "\tPos X: %4.2f Y: %4.2f Z: %4.2f\n"
							 "\tDist: %4.4f\n"
							 "\tParent Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n"
							 "\tChild Force: X: %4.4f Y: %4.4f Z: %4.4f Mag: %4.4f\n",
					tempstr,
					(int)i + 1, m_model->GetJointCount(),
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
	logger->log_info(__FILE__, __LINE__, std::string(tempstr));
}
//Communication Functions
void RobotPlugin::implement_cmd(const eros::pin::ConstPtr &_msg)
{
	bool found = false;
	for (std::size_t i = 0; i < linear_actuators.size(); ++i)
	{
		if (_msg->ConnectedDevice == linear_actuators.at(i).linear_actuator.get_commandname())
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
	if (found == false)
	{
		logger->log_error(__FILE__, __LINE__, "Couldn't parse pin: " + _msg->ConnectedDevice);
	}
}
void RobotPlugin::drivetrain_left_cmd(const eros::pin::ConstPtr &_msg)
{
	drivecommand_received = true;
	double left_voltage = left_motorcontroller.set_input(_msg->Value);
	left_cmd = left_motor.set_input(left_voltage);
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		if (joints.at(i).joint_type == JointType::DRIVETRAIN_LEFT)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0, left_cmd);
		}
	}
}

void RobotPlugin::drivetrain_right_cmd(const eros::pin::ConstPtr &_msg)
{
	drivecommand_received = true;
	double right_voltage = right_motorcontroller.set_input(_msg->Value);
	right_cmd = right_motor.set_input(right_voltage);
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		if (joints.at(i).joint_type == JointType::DRIVETRAIN_RIGHT)
		{
			m_model->GetJoints()[joints.at(i).id]->SetVelocity(0, right_cmd);
		}
	}
}
void RobotPlugin::panservo_cmd(const eros::pin::ConstPtr& _msg)
{
	if(camera_pantilt.initialized == true)
	{
		camera_pantilt.assy.set_jointcommand((uint8_t)CameraPanTilt::JointIndex::JOINT_PAN_INDEX,convert_pwm_toangle_deg(_msg->Value));
	}
}
void RobotPlugin::tiltservo_cmd(const eros::pin::ConstPtr& _msg)
{
	if(camera_pantilt.initialized == true)
	{
		camera_pantilt.assy.set_jointcommand((uint8_t)CameraPanTilt::JointIndex::JOINT_TILT_INDEX,convert_pwm_toangle_deg(_msg->Value));
	}
}
//Utility Functions
void RobotPlugin::print_loopstates(SimpleTimer timer)
{
	char tempstr[1024];
	sprintf(tempstr, "%s: Error: %4.2f%% Target Rate: %4.2f Actual Rate: %4.2f Set Rate: %4.2f", timer.get_name().c_str(), fabs(timer.get_timingerrorperc()),
			timer.get_rate(), timer.get_actualrate(), timer.get_setrate());
	logger->log_info(__FILE__, __LINE__, std::string(tempstr));
}
void RobotPlugin::print_model()
{
	printf("--- JOINTS ---\n");
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		printf("[%d] Type: %s Name: %s\n",
			   joints.at(i).id,
			   map_jointtype_tostring(joints.at(i).joint_type).c_str(),
			   joints.at(i).name.c_str());
	}
	printf("--- LINKS ---\n");
	for (std::size_t i = 0; i < links.size(); ++i)
	{
		printf("[%d] Name: %s\n",
			   links.at(i).id,
			   links.at(i).name.c_str());
		std::cout << "\tPose: " << m_model->GetLink(links.at(i).name)->RelativePose() << std::endl;
	}
}
std::string RobotPlugin::map_jointtype_tostring(JointType joint_type)
{
	switch (joint_type)
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
double RobotPlugin::scale_value(double input_perc, double y1, double neutral, double y2)
{
	double m_upper, m_lower = 0.0;
	double out_upper, out_lower, out = 0.0;
	m_upper = (y2 - neutral) / 100.0;
	m_lower = (neutral - y1) / 100.0;
	out_upper = (m_upper * input_perc) + neutral;
	out_lower = (m_lower * input_perc) + neutral;
	if (input_perc >= 0.0)
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
	double d = sqrt((a.X() * a.X()) + (a.Y() * a.Y()) + (a.Z() * a.Z()));
	return d;
}
double RobotPlugin::compute_distance(ignition::math::Pose3d a, ignition::math::Pose3d b)
{
	double dx = a.Pos().X() - b.Pos().X();
	double dy = a.Pos().Y() - b.Pos().Y();
	double dz = a.Pos().Z() - b.Pos().Z();
	double d = sqrt((dx * dx) + (dy * dy) + (dz * dz));
	return d;
}
double RobotPlugin::convert_pwm_toangle_deg(int32_t v)
{
	//v: range from 1000 to 2000 with center of 1500
	double v1 = v-1500; //Range from -500 to 500
	double angle_deg = v1*180.0/500.0; //Range from -180 deg to 180 deg
	return angle_deg;
}
bool RobotPlugin::readLinkPose(std::string shortname, ignition::math::Pose3d *pose)
{
	bool found = false;
	auto t_links = m_model->GetLinks();
	for (std::size_t i = 0; i < t_links.size(); ++i)
	{
		std::string name = m_model->GetLinks()[i]->GetScopedName();
		if (std::string::npos != name.find(shortname))
		{
			found = true;
			*pose = m_model->GetLink(name)->RelativePose();
		}
	}
	return found;

	//m_model->GetLink(links.at(i).name)->GetRelativePose()
}
eros::diagnostic RobotPlugin::update_diagnostic(uint8_t diagnostic_type, uint8_t level, uint8_t message, std::string description)
{
	return update_diagnostic(root_diagnostic.DeviceName, diagnostic_type, level, message, description);
}
eros::diagnostic RobotPlugin::update_diagnostic(eros::diagnostic diag)
{
	return update_diagnostic(diag.DeviceName, diag.Diagnostic_Type, diag.Level, diag.Diagnostic_Message, diag.Description);
}
eros::diagnostic RobotPlugin::update_diagnostic(std::string device_name, uint8_t diagnostic_type, uint8_t level, uint8_t message, std::string description)
{
	bool devicetype_found = false;
	bool devicename_found = false;
	eros::diagnostic diag;
	uint8_t insert_index = -1;
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		if (diagnostic_type == diagnostics.at(i).Diagnostic_Type)
		{
			devicetype_found = true;
			insert_index = i;
			if (diagnostics.at(i).DeviceName == device_name)
			{
				devicename_found = true;
				diag = diagnostics.at(i);
				diag.Level = level;
				diag.Diagnostic_Message = message;
				diag.Description = description;
				diagnostics.at(i) = diag;
			}
		}
	}
	if ((devicetype_found == true) and (devicename_found == false))
	{
		diag = root_diagnostic;
		diag.Diagnostic_Type = diagnostic_type;
		diag.DeviceName = device_name;
		diag.Level = level;
		diag.Diagnostic_Message = message;
		diag.Description = description;
		std::vector<eros::diagnostic>::iterator it;
		it = diagnostics.begin();
		diagnostics.insert(it + insert_index, diag);
	}
	if (devicetype_found == true)
	{
		return diag;
	}
	else
	{
		diag = root_diagnostic;
		diag.Diagnostic_Type = diagnostic_type;
		diag.Level = ERROR;
		diag.Diagnostic_Message = UNKNOWN_MESSAGE;
		char tempstr[512];
		sprintf(tempstr, "Unsupported Diagnostic Type: %s(%d).  Did you forget to enable it?",
				diagnostic_helper.get_DiagTypeString(diagnostic_type).c_str(), diagnostic_type);
		diag.Description = std::string(tempstr);
		return diag;
	}
}