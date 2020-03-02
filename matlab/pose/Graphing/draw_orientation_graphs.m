function [figs] = draw_orientation_graphs (log_start_time,signals)
  figs = [];
  %Draw X-Axis
  x_signals = subset_signals(signals,'roll');
  [~,sz] = size(x_signals);
  cm = jet(sz);
  fig_title = 'Orientation-Roll';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig1_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(x_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([x_signals{i}.tov]-offset_time,[x_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(x_signals{i}(1).class) ': ' x_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Orientation (deg)")
  hold off  
  fig1_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(x_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([x_signals{i}.tov]-offset_time,[x_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(x_signals{i}(1).class) ': ' x_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg)")
  hold off
  fig1_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(x_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([x_signals{i}.tov]-offset_time,[x_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(x_signals{i}(1).class) ': ' x_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Y-Axis
  y_signals = subset_signals(signals,'pitch');
  [~,sz] = size(y_signals);
  cm = jet(sz);
  fig_title = 'Orientation-Pitch';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig2_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(y_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([y_signals{i}.tov]-offset_time,[y_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(y_signals{i}(1).class) ': ' y_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Orientation (deg)")
  hold off  
  fig2_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(y_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([y_signals{i}.tov]-offset_time,[y_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(y_signals{i}(1).class) ': ' y_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg/s)")
  hold off
  fig2_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(y_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([y_signals{i}.tov]-offset_time,[y_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(y_signals{i}(1).class) ': ' y_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Z-Axis
  z_signals = subset_signals(signals,'yaw');
  [~,sz] = size(z_signals);
  cm = jet(sz);
  fig_title = 'Orientation-Yaw';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig3_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(z_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([z_signals{i}.tov]-offset_time,[z_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(z_signals{i}(1).class) ': ' z_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Orientation (deg)")
  hold off  
  fig3_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(z_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([z_signals{i}.tov]-offset_time,[z_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(z_signals{i}(1).class) ': ' z_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg/s)")
  hold off
  fig3_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(z_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
      offset_time=log_start_time;
    end
    plot([z_signals{i}.tov]-offset_time,[z_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(z_signals{i}(1).class) ': ' z_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
