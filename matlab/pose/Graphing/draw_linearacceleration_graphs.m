function [figs] = draw_linearacceleration_graphs (log_start_time,signals)
  global SignalClass;
  figs = [];
  %Draw X-Axis
  xacc_signals = subset_signals(signals,'xacc');
  [~,sz] = size(xacc_signals);
  cm = jet(sz);
  fig_title = 'Acceleration-X';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig1_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xacc_signals{i}.tov]-offset_time,[xacc_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xacc_signals{i}(1).class) ': ' xacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Acceleration (m/s^2)")
  hold off  
  fig1_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xacc_signals{i}.tov]-offset_time,[xacc_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xacc_signals{i}(1).class) ': ' xacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (m/s^2)")
  hold off
  fig1_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(xacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([xacc_signals{i}.tov]-offset_time,[xacc_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(xacc_signals{i}(1).class) ': ' xacc_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Y-Axis
  yacc_signals = subset_signals(signals,'yacc');
  [~,sz] = size(xacc_signals);
  cm = jet(sz);
  fig_title = 'Acceleration-Y';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig2_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(yacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([yacc_signals{i}.tov]-offset_time,[yacc_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(yacc_signals{i}(1).class) ': ' yacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Acceleration (m/s^2)")
  hold off  
  fig2_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(yacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([yacc_signals{i}.tov]-offset_time,[yacc_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(yacc_signals{i}(1).class) ': ' yacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (m/s^2)")
  hold off
  fig2_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(yacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([yacc_signals{i}.tov]-offset_time,[yacc_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(yacc_signals{i}(1).class) ': ' yacc_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  
%Draw Z-Axis
  zacc_signals = subset_signals(signals,'zacc');
  [~,sz] = size(xacc_signals);
  cm = jet(sz);
  fig_title = 'Acceleration-Z';
  figure('Name',fig_title,'NumberTitle','off')
  title(fig_title);
  fig3_ax1 = subplot(3,1,1);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zacc_signals{i}.tov]-offset_time,[zacc_signals{i}.value],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zacc_signals{i}(1).class) ': ' zacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("Acceleration (m/s^2)")
  hold off  
  fig3_ax2 = subplot(3,1,2);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zacc_signals{i}.tov]-offset_time,[zacc_signals{i}.rms],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zacc_signals{i}(1).class) ': ' zacc_signals{i}(1).name];
  end  
  legend(leg)
  ylabel("RMS (m/s^2)")
  hold off
  fig3_ax3 = subplot(3,1,3);
  hold on
  leg = [];
  for i = 1:sz
    offset_time = 0;
    if(zacc_signals{i}(1).class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
      offset_time=log_start_time;
    end
    plot([zacc_signals{i}.tov]-offset_time,[zacc_signals{i}.status],'color',cm(i,:));
    leg{i} = [map_signalclass_tostring(zacc_signals{i}(1).class) ': ' zacc_signals{i}(1).name];
  end  
  legend(leg)
  xlabel("Time (s)")
  ylabel("Status")
  hold off  

endfunction
