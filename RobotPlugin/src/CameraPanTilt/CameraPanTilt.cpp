/*
 * SonarSensor.cpp
 *
 */

#include "CameraPanTilt.h"

CameraPanTilt::CameraPanTilt()
{
    // TODO Auto-generated constructor stub
    initialized = false;
}

CameraPanTilt::~CameraPanTilt()
{
    // TODO Auto-generated destructor stub
}
bool CameraPanTilt::init(std::string panjoint_partnumber, std::string tiltjoint_partnumber)
{
    update_count = 0;
    if (panjoint_partnumber == PN_361005)
    {
        Joint joint;
        joint.name = "CameraPanJoint";
        joint.partnumber = panjoint_partnumber;
        eros::diagnostic diag;
        diag = joint.cg.init(diag, ControlGroup::Mode::PID, "CameraPanJointControlGroup");
        if (diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        joint.cg.set_printtuninginfo(false);
        joint.cg.set_PIDGains(5.0,0.0,2.0);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::INPUT, "Current", SIGNALTYPE_ANGLE);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::INPUT, "Command", SIGNALTYPE_ANGLE);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::OUTPUT, "Output", SIGNALTYPE_FORCE);
        joint.cg.set_outputlimits(-50.0,0.0,50.0);
        joint.cg.set_max_deltaoutput(100.0);
        diag = joint.cg.finish_initialization();
        if (diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        diag = joint.cg.new_input("Command",0.0);
        if(diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        joints.push_back(joint);
    }
    else
    {
        printf("[ERROR] Camera Pan Joint Part Number: %s Not Supported.\n", panjoint_partnumber.c_str());
    }
    if (panjoint_partnumber == PN_361005)
    {
        Joint joint;
        joint.name = "CameraTiltJoint";
        joint.partnumber = tiltjoint_partnumber;
        eros::diagnostic diag;
        diag = joint.cg.init(diag, ControlGroup::Mode::PID, "CameraTiltJointControlGroup");
        if (diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        joint.cg.set_printtuninginfo(false);
        joint.cg.set_PIDGains(10.0, 3.0,0.0);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::INPUT, "Current", SIGNALTYPE_ANGLE);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::INPUT, "Command", SIGNALTYPE_ANGLE);
        joint.cg.initialize_signal(ControlGroup::SignalDirection::OUTPUT, "Output", SIGNALTYPE_FORCE);
        joint.cg.set_outputlimits(-50.0,0.0,50.0);
        joint.cg.set_max_deltaoutput(100.0);
        diag = joint.cg.finish_initialization();
        if (diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        diag = joint.cg.new_input("Command",0.0);
        if(diag.Level > NOTICE)
        {
            print_diagnostic(__FILE__,__LINE__,diag.Level,diag.Description);
            return false;
        }
        joints.push_back(joint);
    }
    else
    {
        printf("[ERROR] Camera Tilt Joint Part Number: %s Not Supported.\n", tiltjoint_partnumber.c_str());
    }
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        joints.at(i).command_value = 0.0;
        joints.at(i).current_value = 0.0;
        joints.at(i).output_value = 0.0;
    }
    initialized = true;
    return true;
}
double CameraPanTilt::get_currentconsumed()
{
    return 0.0;
}
std::vector<CameraPanTilt::Joint> CameraPanTilt::update(double dt, double pan_joint_current_deg, double tilt_joint_current_deg)
{
    current_time += dt;
    joints.at((std::size_t)JointIndex::JOINT_PAN_INDEX).current_value = pan_joint_current_deg;
    joints.at((std::size_t)JointIndex::JOINT_TILT_INDEX).current_value = tilt_joint_current_deg;
    for (std::size_t i = 0; i < joints.size(); ++i)
    {

        eros::diagnostic diag = joints.at(i).cg.new_input("Current",joints.at(i).current_value);
        if(diag.Level > NOTICE)
        {
            printf("[%s %d] Error: %d %s\n", __FILE__, __LINE__, diag.Level, diag.Description.c_str());
        }
        diag = joints.at(i).cg.update(dt);
        if(diag.Level > NOTICE)
        {
            printf("[%s %d] Error: %d %s\n", __FILE__, __LINE__, diag.Level, diag.Description.c_str());
        }
        std::vector<eros::signal> outs = joints.at(i).cg.get_outputsignals();
        joints.at(i).output_value = outs.at(0).value;
    }
    update_count++;
    return joints;
}