/*
 * CameraPanTilt.h
 */

#ifndef SRC_ICARUS_SIM_SRC_CAMERAPANTILT_H_
#define SRC_ICARUS_SIM_SRC_CAMERAPANTILT_H_
#include "string"
#include <math.h> 
#include <eros/signal.h>
#include <eigen3/Eigen/Dense>
#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>

#include "../../../../eROS/include/eROS_Definitions.h"
#include "../../../../eROS/include/Supported_PN.h"
#include "../../../../eROS/include/controlgroup.h"
class CameraPanTilt {
public:
    enum class JointIndex
	{
		JOINT_PAN_INDEX = 0,
		JOINT_TILT_INDEX =1,
	};
    struct Joint
	{
		std::string name;
        double command_value;
        double current_value;
        double output_value;
        std::string partnumber;
        ControlGroup cg;
	};
	CameraPanTilt();
	bool init(std::string panjoint_partnumber,std::string tiltjoint_partnumber);
	bool is_initialized() { return initialized; }
	double get_currentconsumed();
	std::vector<Joint> update(double dt,double pan_joint_current_deg,double tilt_joint_current_deg);
	virtual ~CameraPanTilt();
    bool set_jointcommand(uint8_t index,double value)
    {
        if(index < (joints.size()))
        {
            if(index == (uint8_t)JointIndex::JOINT_PAN_INDEX)
            {
                value = -1.0*value;
            }
            joints.at(index).command_value = value;
            joints.at(index).cg.new_input("Command",joints.at(index).command_value);
            return true;
        }
        else
        {
            return false;
        }
        
    }
    std::vector<Joint> get_joints() { return joints; }
    void print_diagnostic(std::string filename,int linenumber,int level,std::string description)
    {
        printf("[%s %d] Level: %d Desc: %s\n",filename.c_str(),linenumber,level,description.c_str());
    }
private:
	bool initialized;
	uint64_t update_count;
	double current_time;
    std::vector<Joint> joints;
};

#endif /* SRC_ICARUS_SIM_SRC_SONARSENSOR_H_ */
