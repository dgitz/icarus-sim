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
Initialize;
if(Config.PoseModelMode != MODELEXECUTION_MODE.Standard_PoseModel)
  TestClass;
elseif(Config.PoseModelMode == MODELEXECUTION_MODE.Standard_PoseModel)
  RunPoseModel;
  if(Analyze_Pose == 1)
    AnalyzeData;
  end
  if(Plot_Figures == 1)
    DrawGraphs
  end
end




