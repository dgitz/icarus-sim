
if(Config.PoseModelMode == MODELEXECUTION_MODE.DummyData_TimeCompensate)
  [signal_vector,name] = create_dummydata_timecompensatorblock;
  tc = TimeCompensate(signal_vector,TIMINGCOMPENSATION_METHOD,SignalState,SignalClass,name,Config.TimeCompensation_Method);

  elap_time = 0;
  time_to_run = 100;
  dt = .05;
  out_vector = [];
  while(elap_time <= time_to_run)
    if(tc.current_index >= 100)
      a = 1;
    end
    if(length(out_vector) > 1)
      if(out_vector(end).status == SignalState.SIGNALSTATE_EXTRAPOLATED)
        a = 1;
      end
    end
    tc = tc.new_input(0,elap_time);
    time_signal = tc.output;
    sensor_postprocess = sensor_postprocess.new_input(time_signal);
    out = sensor_postprocess.output;
    out_vector = [out_vector out];
    elap_time = elap_time+dt;
  end
  subplot(3,1,1) % Input Vector and Output Vector
  hold on
  plot([signal_vector.tov],[signal_vector.value],'b');
  hold off
  subplot(3,1,2)
  hold on
  plot([out_vector.tov],[out_vector.value],'r');
  hold off
  subplot(3,1,3) %Input vector state and output vector state
  hold on
  plot([signal_vector.tov],[signal_vector.status],'b');
  plot([out_vector.tov],[out_vector.status],'r');
  hold off
  Load_Data=0;
elseif(Config.PoseModelMode == MODELEXECUTION_MODE.DummyData_SensorPostProcess)
  [signal_vector,name] = create_dummydata_timecompensatorblock;
  tc = TimeCompensate(signal_vector,TIMINGCOMPENSATION_METHOD,SignalState,SignalClass,name,Config.TimeCompensation_Method);
  sensor_postprocess = SensorPostProcess(SignalClass);
  elap_time = 0;
  time_to_run = 100;
  dt = .05;
  out_vector = [];
  while(elap_time <= time_to_run)
    if(tc.current_index >= 100)
      a = 1;
    end
    if(length(out_vector) > 1)
      if(out_vector(end).status == SignalState.SIGNALSTATE_EXTRAPOLATED)
        a = 1;
      end
    end
    tc = tc.new_input(0,elap_time);
    out = tc.output;
    out_vector = [out_vector out];
    elap_time = elap_time+dt;
  end
  subplot(3,1,1) % Input Vector and Output Vector
  hold on
  plot([signal_vector.tov],[signal_vector.value],'b');
  hold off
  subplot(3,1,2)
  hold on
  plot([out_vector.tov],[out_vector.value],'r');
  hold off
  subplot(3,1,3) %Input vector state and output vector state
  hold on
  plot([signal_vector.tov],[signal_vector.status],'b');
  plot([out_vector.tov],[out_vector.status],'r');
  hold off
  Load_Data=0;
elseif(Config.PoseModelMode == MODELEXECUTION_MODE.DummyData_SensorLinker)
  Sensor_Signals = [];
  Timed_Signals = [];
  Processed_Signals = [];
  Input_Signals = [];
  [Processed_Signals] = create_dummydata_signallinkerblock;
  signal_linker = MasterLinker;
  signal_linker = signal_linker.init(SignalClass,SignalType,SignalState);
  new_input_vector = [];
  for i = 1:length(Processed_Signals)
    new_input_vector = [new_input_vector Processed_Signals{i}(1)];
  end
  signal_linker = signal_linker.new_input(new_input_vector);
  log_start_time = 0;
  elap_time = 0;
  time_to_run = 100;
  dt = .05;
  out_vector = [];
  for t = 1:length(Processed_Signals{1})
    new_input_vector = [];
    for i = 1:length(Processed_Signals)
      new_input_vector = [new_input_vector Processed_Signals{i}(t)];
    end
    signal_linker = signal_linker.new_input(new_input_vector);
    out.linear_accelerations = signal_linker.linearacceleration_linker.linear_accelerations;
    out.rotation_rates = signal_linker.rotationrate_linker.rotation_rates;
    out.orientations = signal_linker.orientation_linker.orientations;
    Input_Signals = [Input_Signals out];
    %
    %Input_Signals{t} = out;
    elap_time = elap_time+dt;
  end
  %DrawGraphs
  Load_Data=0;
end