close all;
fig_list = [];
%Plot_Jerk = 1;
Plot_Acceleration = 1;
Plot_AngularRate = 0;
%Plot_MagneticField = 1;
Plot_Orientation = 1;

Plot_SensorSignals = 1;
Plot_TimedSignals = 0;
Plot_ProcessedSignals = 0;
Plot_InputSignals = 1;

linear_acc_signals = [];
rotation_rate_signals = [];
magnetic_field_signals = [];
orientation_signals = [];
if(Plot_SensorSignals == 1)
  for i = 1:length(Sensor_Signals)
    if(Sensor_Signals{i}(1).type == SignalType.SIGNALTYPE_ACCELERATION)
      linear_acc_signals{length(linear_acc_signals)+1} = Sensor_Signals{i};
    end
    if(Sensor_Signals{i}(1).type == SignalType.SIGNALTYPE_ROTATION_RATE)
      rotation_rate_signals{length(rotation_rate_signals)+1} = Sensor_Signals{i};
    end
    if(Sensor_Signals{i}(1).type == SignalType.SIGNALTYPE_MAGNETIC_FIELD)
      magnetic_field_signals{length(magnetic_field_signals)+1} = Sensor_Signals{i};
    end
  end
end
if(Plot_TimedSignals == 1)
  for i = 1:length(Timed_Signals)
    if(Timed_Signals{i}(1).type == SignalType.SIGNALTYPE_ACCELERATION)
      linear_acc_signals{length(linear_acc_signals)+1} = Timed_Signals{i};
    end
    if(Timed_Signals{i}(1).type == SignalType.SIGNALTYPE_ROTATION_RATE)
      rotation_rate_signals{length(rotation_rate_signals)+1} = Timed_Signals{i};
    end
    if(Timed_Signals{i}(1).type == SignalType.SIGNALTYPE_MAGNETIC_FIELD)
      magnetic_field_signals{length(magnetic_field_signals)+1} = Timed_Signals{i};
    end
  end
end
if(Plot_ProcessedSignals == 1)
  for i = 1:length(Processed_Signals)
    if(Processed_Signals{i}(1).type == SignalType.SIGNALTYPE_ACCELERATION)
      linear_acc_signals{length(linear_acc_signals)+1} = Processed_Signals{i};
    end
    if(Processed_Signals{i}(1).type == SignalType.SIGNALTYPE_ROTATION_RATE)
      rotation_rate_signals{length(rotation_rate_signals)+1} = Processed_Signals{i};
    end
    if(Processed_Signals{i}(1).type == SignalType.SIGNALTYPE_MAGNETIC_FIELD)
      magnetic_field_signals{length(magnetic_field_signals)+1} = Processed_Signals{i};
    end
  end
end
if(Plot_InputSignals == 1)
  if(length(Input_Signals) > 0)
    for i = 1:length(Input_Signals(1).linear_accelerations)
      vector_x = [];
      vector_y = [];
      vector_z = [];
      for t = 1:length(Input_Signals)
        vector_x = [vector_x Input_Signals(t).linear_accelerations{i}.x];
        vector_y = [vector_y Input_Signals(t).linear_accelerations{i}.y];
        vector_z = [vector_z Input_Signals(t).linear_accelerations{i}.z];
      end
      linear_acc_signals{length(linear_acc_signals)+1} = vector_x;
      linear_acc_signals{length(linear_acc_signals)+1} = vector_y;
      linear_acc_signals{length(linear_acc_signals)+1} = vector_z;
    end

    for i = 1:length(Input_Signals(1).rotation_rates)
      vector_x = [];
      vector_y = [];
      vector_z = [];
      for t = 1:length(Input_Signals)
        vector_x = [vector_x Input_Signals(t).rotation_rates{i}.x];
        vector_y = [vector_y Input_Signals(t).rotation_rates{i}.y];
        vector_z = [vector_z Input_Signals(t).rotation_rates{i}.z];
      end
      rotation_rate_signals{length(rotation_rate_signals)+1} = vector_x;
      rotation_rate_signals{length(rotation_rate_signals)+1} = vector_y;
      rotation_rate_signals{length(rotation_rate_signals)+1} = vector_z;
    end

    for i = 1:length(Input_Signals(1).orientations)
      vector_x = [];
      vector_y = [];
      vector_z = [];
      for t = 1:length(Input_Signals)
        vector_x = [vector_x Input_Signals(t).orientations{i}.x];
        vector_y = [vector_y Input_Signals(t).orientations{i}.y];
        vector_z = [vector_z Input_Signals(t).orientations{i}.z];
      end
      orientation_signals{length(orientation_signals)+1} = vector_x;
      orientation_signals{length(orientation_signals)+1} = vector_y;
      orientation_signals{length(orientation_signals)+1} = vector_z;
    end
  end
end
if(Plot_Acceleration == 1)
  fig_list = draw_linearacceleration_graphs(log_start_time,linear_acc_signals);
end
if(Plot_AngularRate == 1)
  fig_list = draw_rotationrate_graphs(log_start_time,rotation_rate_signals);
end
if(Plot_Orientation == 1)
  fig_list = draw_orientation_graphs(log_start_time,orientation_signals);
end
