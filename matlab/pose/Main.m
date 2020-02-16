%Main
close all
clc
more off

addpath(genpath('Config'));
addpath(genpath('Development'));
addpath(genpath('Graphing'));
addpath(genpath('Helper'));
addpath(genpath('SensorModel'));
addpath(genpath('PoseModel'));
addpath(genpath('Initializers'));
addpath(genpath('Analysis'));
addpath(genpath('Simulink'));

if(bdIsLoaded('PoseExecutor') == 0)
    pause(3);
    disp('Opening PoseExecutor Model');
    open_system('PoseExecutor');
    pause(3);
    set_param(gcs,'EnableLBRepository','on');
end

Initialize;
% if(Config.PoseModelMode != MODELEXECUTION_MODE.Standard_PoseModel)
%   TestClass;
% elseif(Config.PoseModelMode == MODELEXECUTION_MODE.Standard_PoseModel)
%   RunPoseModel;
%   if(Analyze_Pose == 1)
%     AnalyzeData;
%   end
%   if(Plot_Figures == 1)
%     DrawGraphs
%   end
% end




