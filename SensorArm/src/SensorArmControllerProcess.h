#include "BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class SensorArmControllerProcess SensorArmControllerProcess.h "SensorArmControllerProcess.h"
 *  \brief This is a SensorArmControllerProcess class.  Used for the sample_node node.
 *
 */
#define JOINT_COUNT 3
class SensorArmControllerProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
	enum class SampleEnum 
	{
		UNKNOWN=0,
		ENUMTYPEA=1,
		ENUMTYPEB=2,
	};
    //Structs
	struct Joint
	{
		double command_position;
		std::string topic_name;
		double start_angle;
		double stop_angle;
		double step_angle;
	};
	//Sub-Classes
	class SampleSubClass
	{
		public:
		bool initialize(std::string t_name)
		{
			name = t_name;
			return true;
		}
		std::string getName()
		{
			return name;
		}
		private:

			std::string name;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic set_config_filepaths(std::string filepath);
	eros::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_holdsteptime(double v) { holdtime_step = v; }
	bool set_jointproperties(uint8_t index,double start,double stop, double step)
	{
		if(index > (JOINT_COUNT-1))
		{
			return false;
		}
		if(index < 0)
		{
			return false;
		}
		joints.at(index).start_angle = start;
		joints.at(index).stop_angle = stop;
		joints.at(index).step_angle = step;
		return true;
	}
	std::vector<Joint> get_joints() { return joints; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions
	uint8_t map_sampledatatype_ToInt(std::string data);
	std::string map_sampledatatype_ToString(uint8_t data);

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::string config_filepath;

	std::map<uint8_t,std::string> sample_map;
	std::vector<Joint> joints;
	double holdtime_step;
	double lasttime_jointsupdated;
	int8_t jointindex_toupdate;

};
