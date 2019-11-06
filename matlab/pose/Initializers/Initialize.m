%Initialize
Load_Data = 0; %Probably shouldn't have to change this.
Load_GlobalDefines;
Load_Configurables;
Load_SensorConfig;
Load_FilterConfig;
disp(['Using Scenario: ' Scenario]);
if(exist('Sensor_Data_Loaded') == 0)
    Load_Data = 1;
elseif(Sensor_Data_Loaded == 0)
  Load_Data = 1;
end


if(Config.PoseModelMode == MODELEXECUTION_MODE.DummyData_TimeCompensate)
  [signal_vector,time_compensation_method_enum,signalstate_enum,name,time_compensation_method] = create_dummydata_timecompensatorblock;
  tc = TimeCompensate(signal_vector,time_compensation_method_enum,SignalState,SignalClass,name,time_compensation_method);

  elap_time = 0;
  time_to_run = 12.0;
  dt = 0.001;
  while(elap_time < time_to_run)
    if(tc.current_index >= 100)
      a = 1;
    end
    tc = tc.new_input(elap_time);
    out = tc.output;
    elap_time = elap_time+dt;
  end
  
  Load_Data=0;
end
if(Load_Data == 1)
  disp(['Loading Data...']);
else
  disp(['Data already loaded.']);
end
if(Load_Data == 1)
  [Sensor_Data_Loaded,Sensor_Signals,IMU_Count] = Load_SensorSignals(Scenario);
end