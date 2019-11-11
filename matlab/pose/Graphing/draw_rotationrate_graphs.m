function [figs] = draw_rotationrate_graphs (log_start_time,signals)
  global SignalClass;
  figs = [];
  %Draw X-Axis
  xgyro_signals = subset_signals(signals,'xgyro');
  [~,sz] = size(xgyro_signals);
  cm = jet(sz);
  fig_title = 'Rotation Rate-X';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig1_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xgyro_signals{i}.tov]-offset_time,[xgyro_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xgyro_signals{i}(1).class) ': ' xgyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Rotation Rate (deg/s)")
  hold off  
  fig1_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xgyro_signals{i}.tov]-offset_time,[xgyro_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xgyro_signals{i}(1).class) ': ' xgyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg/s)")
  hold off
  fig1_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xgyro_signals{i}.tov]-offset_time,[xgyro_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xgyro_signals{i}(1).class) ': ' xgyro_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Y-Axis
  ygyro_signals = subset_signals(signals,'ygyro');
  [~,sz] = size(xgyro_signals);
  cm = jet(sz);
  fig_title = 'Rotation Rate-Y';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig2_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(ygyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([ygyro_signals{i}.tov]-offset_time,[ygyro_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(ygyro_signals{i}(1).class) ': ' ygyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Rotation Rate (deg/s)")
  hold off  
  fig2_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(ygyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([ygyro_signals{i}.tov]-offset_time,[ygyro_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(ygyro_signals{i}(1).class) ': ' ygyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg/s)")
  hold off
  fig2_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(ygyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([ygyro_signals{i}.tov]-offset_time,[ygyro_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(ygyro_signals{i}(1).class) ': ' ygyro_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Z-Axis
  zgyro_signals = subset_signals(signals,'zgyro');
  [~,sz] = size(xgyro_signals);
  cm = jet(sz);
  fig_title = 'Rotation Rate-Z';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig3_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zgyro_signals{i}.tov]-offset_time,[zgyro_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zgyro_signals{i}(1).class) ': ' zgyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Rotation Rate (deg/s)")
  hold off  
  fig3_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zgyro_signals{i}.tov]-offset_time,[zgyro_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zgyro_signals{i}(1).class) ': ' zgyro_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (deg/s)")
  hold off
  fig3_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zgyro_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zgyro_signals{i}.tov]-offset_time,[zgyro_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zgyro_signals{i}(1).class) ': ' zgyro_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  

endfunction
