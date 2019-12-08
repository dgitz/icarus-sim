#include "SensorArmControllerProcess.h"
eros::diagnostic SensorArmControllerProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	config_filepath = t_filepath;
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config File Paths Set");
	return diag;
}
eros::diagnostic SensorArmControllerProcess::load_configfile(std::string path)
{
	eros::diagnostic diag = root_diagnostic;
	bool all_items_loaded = true;
	if(all_items_loaded == false)
	{
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Not all items loaded.");
		return diag;
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config Loaded.");
		return diag;
	}


}
eros::diagnostic  SensorArmControllerProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	jointindex_toupdate = -1;
	for(int i = 0; i < JOINT_COUNT; ++i)
	{
		Joint joint;
		joint.command_position = 0.0;
		joint.start_angle = 0.0;
		joint.stop_angle = 0.0;
		joint.step_angle = 0.0;
		joint.topic_name = "/rrbot/joint" + std::to_string(i) + "_position_controller/command";
		joints.push_back(joint);
	}
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
    return diag;
}
eros::diagnostic SensorArmControllerProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		if(ready == false)
		{
			
		}
		ready = true;

	}
	bool update_joints = false;
	if(jointindex_toupdate < 0)
	{
		lasttime_jointsupdated = t_ros_time;
		jointindex_toupdate = 0;
		update_joints = true;
	}
	else if((t_ros_time-lasttime_jointsupdated) >= holdtime_step)
	{
		update_joints = true;
	}
	if(update_joints == true)
	{
		lasttime_jointsupdated = t_ros_time;
		for(std::size_t i = 0; i < joints.size(); ++i)
		{
			double new_pos = joints.at(i).command_position + joints.at(i).step_angle;
			if(new_pos > joints.at(i).stop_angle)
			{
				joints.at(i).command_position = joints.at(i).start_angle;
			}
			else
			{
				joints.at(i).command_position = new_pos;
				break;
			}
		}
		for(std::size_t i = 0; i < joints.size(); ++i)
		{
			printf("[%d]:%f",i,joints.at(i).command_position);
		}
		printf("\n");
		/*double new_pos = joints.at(jointindex_toupdate).command_position + joints.at(jointindex_toupdate).step_angle;
		if(new_pos > joints.at(jointindex_toupdate).stop_angle)
		{
			joints.at(jointindex_toupdate).command_position = joints.at(jointindex_toupdate).start_angle;
			printf("1j: %d v: %f\n",jointindex_toupdate,joints.at(jointindex_toupdate).command_position);
			jointindex_toupdate++;
		}
		else
		{
			joints.at(jointindex_toupdate).command_position = new_pos;
			printf("2j: %d v: %f\n",jointindex_toupdate,joints.at(jointindex_toupdate).command_position);
		}
		if(jointindex_toupdate > (JOINT_COUNT-1))
		{
			jointindex_toupdate = 0;
		}
		*/

	}
	eros::diagnostic diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running");
		
	}
	return diag;
}
eros::diagnostic SensorArmControllerProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	return diag;
}
std::vector<eros::diagnostic> SensorArmControllerProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> SensorArmControllerProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
uint8_t SensorArmControllerProcess::map_sampledatatype_ToInt(std::string data)
{
	std::map<std::string,uint8_t> reverse_map;
	std::map<std::string,uint8_t>::iterator it = reverse_map.begin();

	for (auto& x: sample_map)
	{
		reverse_map.insert (it, std::pair<std::string,uint8_t>(x.second,x.first));
	}
	it = reverse_map.find(data);
	if (it != reverse_map.end())
	{
		return it->second;
	}
	return (uint8_t)SampleEnum::UNKNOWN;
}
std::string SensorArmControllerProcess::map_sampledatatype_ToString(uint8_t data)
{
	std::map<uint8_t,std::string>::iterator it;
	it = sample_map.find(data);
	if (it != sample_map.end())
	{
		return it->second;
	}
	return "UNDEFINED";
}