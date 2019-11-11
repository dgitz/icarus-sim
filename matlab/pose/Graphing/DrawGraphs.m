close all;
fig_list = [];
%Plot_Jerk = 1;
Plot_Acceleration = 1;
Plot_AngularRate = 1;
%Plot_MagneticField = 1;
%Plot_Angle = 1;

linear_acc_signals = [];
rotation_rate_signals = [];
magnetic_field_signals = [];
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

if(Plot_Acceleration == 1)
  fig_list = draw_linearacceleration_graphs(log_start_time,linear_acc_signals);
end
if(Plot_AngularRate == 1)
  fig_list = draw_rotationrate_graphs(log_start_time,rotation_rate_signals);
end

