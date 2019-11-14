%RunPoseModel
%Calculate time to run: This should be fixed in US: 85
global Config
Timed_Signals = [];
Processed_Signals = [];
Input_Signals = [];
start_times_vector=[];
end_times_vector=[];
for i = 1:length(Sensor_Signals)
  start_times_vector(i) = Sensor_Signals{i}(1).tov;
  end_times_vector(i) = Sensor_Signals{i}(end).tov;
end
log_start_time = min(start_times_vector);
log_end_time = max(end_times_vector);
start_time = 0;
end_time = log_end_time-log_start_time;
%% Initialize all Pose Model Signals, Objects
time_compensators = [];
sensor_postprocessors = [];
for i = 1:length(Sensor_Signals)
  %% Time Compensator Objects
  tc = TimeCompensate(Sensor_Signals{i}(:),TIMINGCOMPENSATION_METHOD,SignalState,SignalClass,Sensor_Signals{i}(1).name,Config.TimeCompensation_Method);
  time_compensators{i} = tc;
  sp = SensorPostProcess;
  sp = sp.init(SignalClass);
  sensor_postprocessors{i} = sp;
end
%% Run Pose Model
disp(['Log Start Time: ' num2str(log_start_time) '(s) Log End Time: ' num2str(log_end_time) '(s)']);
disp(["Running Pose Model for: " num2str(end_time) " seconds (simulation time)"]);
counter = 0;
sim_time = start_time;
sensor_postprocess = SensorPostProcess;
signal_linker = MasterLinker;
signal_linker = signal_linker.init(SignalClass,SignalType,SignalState);
while(sim_time < end_time)
  fixed_dt = 1.0/Config.Pose_UpdateRate;
  dt = fixed_dt;
  
  %% Time Compensator Calls
  signal_vector = [];
  for i = 1:length(Sensor_Signals)
    time_compensators{i} = time_compensators{i}.new_input(log_start_time,sim_time);
    if(counter == 0)
      Timed_Signals{i} = time_compensators{i}.output;
    else
      signal_vector = [Timed_Signals{i} time_compensators{i}.output];
      Timed_Signals{i} = signal_vector;
    end
  end
  
  %% Sensor Post-Processor Calls
  signal_vector = [];
  for i = 1:length(Timed_Signals)
    sensor_postprocessors{i} = sensor_postprocessors{i}.new_input(Timed_Signals{i}(counter+1));
    if(counter == 0)
      Processed_Signals{i} = sensor_postprocessors{i}.output;
    else
      signal_vector = [Processed_Signals{i} sensor_postprocessors{i}.output];
      Processed_Signals{i} = signal_vector;
    end
  end
  
  %% Signal Linker Calls
  signal_vector = [];
  for i = 1:length(Processed_Signals)
    signal_vector = [signal_vector Processed_Signals{i}(counter+1)];
  end
  signal_linker = signal_linker.new_input(signal_vector);
  linked.linear_accelerations = signal_linker.linearacceleration_linker.linear_accelerations;
  linked.rotation_rates = signal_linker.rotationrate_linker.rotation_rates;
  linked.orientations = signal_linker.orientation_linker.orientations;
  Input_Signals = [Input_Signals linked];
  sim_time += dt;
  if((mod(counter,100) == 0) && (sim_time > 0.0))
    disp(["Still Running Simulation... at time: " num2str(sim_time) "/" num2str(end_time)]);
  end  
  
  
  counter=counter+1;
end
if(0)
  %Initialize Signals
  %Raw = IMU_Raw;
  %for i = 1:length(IMU_Raw)
  %  IMU_Signals{length(IMU_Signals)+1} = InitializeSignal(IMU_Raw(i).name,IMU_Raw(1).units);
  %end
  %Sensor_Signals = InitializeSensorSignals(IMU_Signals);
  Pose_Signals = InitializePoseSignals;
  Filt_Signals = InitializeFilterSignals;
  SelfCalibrate_Signals = InitializeCalibrationSignals;
  [LinearAccelerationFilter,AngleRateFilter] = InitializeFilters;
  System_Signals = InitializeSystemSignals;
  simulation_ended = 0;
  sim_time = 0;
  disp("Running Pose Model");
  time = tic;
  counter = 0;
  v = [];
  for i = 1:length(IMU_Raw)
    if(i == 10)
      a = 1;
    end
    s = convert_rawdata_tosignal(IMU_Raw(i));
    Sensor_Signals{i} = s;
  end
  while(simulation_ended == 0)

    for i = 1:length(IMU_Raw)
      %[Raw(i),IMU_Signals{i}] = TimeCompensate(Config.TimeCompensation_Method,Raw(i),IMU_Signals{i},sim_time);  
    end

    
    %Model is now fit to consume sensor data
    %Sensor_Signals = SensorPostProcess(Sensor_Signals,IMU_Signals);

    [Pose_Signals] = SensorLinkage(Pose_Signals,Sensor_Signals,sim_time);
    
    %[LinearAccelerationFilter,Filt_Signals] = RunLinearAccelerationFilter(LinearAccelerationFilter,Filt_Signals,Pose_Signals);

    %[AngleRateFilter,Filt_Signals] = RunAngleRateFilter(AngleRateFilter,Filt_Signals,Pose_Signals);
    
    %[System_Signals] = ComputeSystemState(Filt_Signals,Pose_Signals,System_Signals);
    
    %[SelfCalibrate_Signals] = SelfCalibrate(SelfCalibrate_Signals,Filt_Signals,Pose_Signals,System_Signals,sim_time);
    %v(length(v)+1) = SelfCalibrate_Signals.mag_acc;
    
    %if(Analyze_Pose == 1)
    %  Analysis_Signals = saveSignals(Analysis_Signals,Sensor_Signals,Pose_Signals,Filt_Signals,System_Signals);
    %end
    if((mod(counter,1000) == 0) && (sim_time > 0.0))
      disp(["Still Running Simulation... at time: " num2str(sim_time) "/" num2str(end_time)]);
    end
    
    sim_time = sim_time + 1/(RATES.POSE_RATE);
    if(sim_time >= end_time) %Should get stop condition from time compensation 
      simulation_ended = 1;
      break;
    end
    counter = counter + 1;
  end
  etime = toc(time);
  disp(["Time to Run Pose Model: " num2str(etime) " (actual time)"])
end
