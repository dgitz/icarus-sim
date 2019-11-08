%Initialize
Load_Data = 0; %Probably shouldn't have to change this.
Load_GlobalDefines;
Load_Configurables;
Load_SensorConfig;
Load_FilterConfig;
if(exist('Sensor_Data_Loaded') == 0)
    Load_Data = 1;
elseif(Sensor_Data_Loaded == 0)
  Load_Data = 1;
end
if(Config.PoseModelMode == MODELEXECUTION_MODE.Standard_PoseModel)
  disp(['Using Scenario: ' Scenario]);
else
  Load_Data = 0;
end
if(Load_Data == 1)
  disp(['Loading Data...']);
else
  disp(['Data already loaded.']);
end
if(Load_Data == 1)
  [Sensor_Data_Loaded,Sensor_Signals,IMU_Count] = Load_SensorSignals(Scenario);
end