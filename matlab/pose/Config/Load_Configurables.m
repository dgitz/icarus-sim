%LoadConfigurables
Scenario = '/home/robot/Documents/Data/2017-12-23-08-43-46';
global Config;
Config.TimeCompensation_Method = TIMINGCOMPENSATION_METHOD.LinearExtrapolate;
Config.InitializationTime = 5.0; %seconds
Config.CalibrationThreshold_Jerk = .1;
Config.CalibrationTime = 2.0;
Config.Pose_UpdateRate = 20;
Config.PoseModelMode = MODELEXECUTION_MODE.DummyData_SensorPostProcess;
SaveResults = 0;
Analyze_Pose = 0;
Plot_Figures = 1;
Truth_Available = 0;

global VEHICLE;
VEHICLE.TIRE_DIAMETER_M = 0;
VEHICLE.WHEELBASE_M = 0;