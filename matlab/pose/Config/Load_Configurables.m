%LoadConfigurables
Scenario = '/home/robot/Documents/Data/DriveStraightTurnLeft/2020-02-18-06-06-53_0_CSV';
global Config;
%Config.TimeCompensation_Method = TIMINGCOMPENSATION_METHOD.LinearExtrapolate;
%Config.InitializationTime = 5.0; %seconds
%Config.CalibrationThreshold_Jerk = .1;
%Config.CalibrationTime = 2.0;
%Config.Pose_UpdateRate = 20;
%Config.PoseModelMode = MODELEXECUTION_MODE.Standard_PoseModel;
SaveResults = 0;
Analyze_Pose = 0;
%Plot_Figures = 1;
Truth_Available = 1;

%global VEHICLE;
%VEHICLE.TIRE_DIAMETER_M = 0;
%VEHICLE.WHEELBASE_M = 0;