close all;
fig_list = [];
%Plot_Jerk = 1;
Plot_Acceleration = 1;
%Plot_AngularRate = 1;
%Plot_MagneticField = 1;
%Plot_Angle = 1;

linear_acc_signals = [];
rotation_rate_signals = [];

for i = 1:length(Sensor_Signals)
  if(Sensor_Signals{i}(1).type == SignalType.SIGNALTYPE_ACCELERATION)
    linear_acc_signals{length(linear_acc_signals)+1} = Sensor_Signals{i};
  end
end
for i = 1:length(Timed_Signals)
  if(Timed_Signals{i}(1).type == SignalType.SIGNALTYPE_ACCELERATION)
    linear_acc_signals{length(linear_acc_signals)+1} = Timed_Signals{i};
  end
end

if(Plot_Acceleration == 1)
  fig_list = draw_linearacceleration_graphs(log_start_time,linear_acc_signals);
end

