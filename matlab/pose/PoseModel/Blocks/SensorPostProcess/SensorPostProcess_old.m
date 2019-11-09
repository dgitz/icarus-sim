function [signals] = SensorPostProcess(signals,imu)
global SensorConfig;
global RINGBUFFER_SIZE;
global SignalState;
imu = IMUSensorPostProcess(imu);

s = [imu];
for i = 1:length(SensorConfig)
  if(length(SensorConfig{i}.input) > 1)
    disp(["ERROR: Sensor Input size > 1 NOT SUPPORTED"]);
  end
  for j = 1:length(s)
      if(strcmp(s{j}.name,SensorConfig{i}.input{1}.name) == 1)
        output = 0;
        status = s{j}.status;
        for op = 1:length(SensorConfig{i}.operator)
          if(strcmp(SensorConfig{i}.operator{op}.name,"diff") == 1)
            if(length(signals{j}.value_buffer) < SensorConfig{i}.operator{op}.param_1)
              output = 0;
              status = SignalState.SIGNALSTATE_INITIALIZING;
            else
              v = diff(signals{j}.value_buffer,SensorConfig{i}.operator{op}.param_1);
              output = v(1);
              status = s{j}.status;
            end
          elseif(strcmp(SensorConfig{i}.operator{op}.name,"scale") == 1)
            output = output * SensorConfig{i}.operator{op}.param_1;
          else
            disp(["ERROR: Operator: " SensorConfig{i}.operator{op}.name " NOT SUPPORTED."]);
          end
        end
        SensorConfig{i}.output{1}.value = output;
        SensorConfig{i}.output{1}.status = status;
      end
  end
end
for i = 1:length(SensorConfig)
  for j = 1:length(s)
      if(strcmp(s{j}.name,SensorConfig{i}.output{1}.name) == 1)
        if(j == 13)
          a = 1;
        end
        s{j}.value = SensorConfig{i}.output{1}.value;
        s{j}.value_buffer(length(s{j}.value_buffer)+1) = s{j}.value;
        s{j}.status = SensorConfig{i}.output{1}.status;
      end
  end
end

%Final Post Processing
if(length(s) ~= length(signals))
  disp(["ERROR: SIGNAL SIZE MISMATCH!"]);
end
for i = 1:length(signals)
  input = [];
  output = [];
  input = s{i};
  output = signals{i};
  if(output.initialized == 0)
    if(input.rms < 0)
      output.compute_rms = 1;
    else
      output.compute_rms = input.rms;
    end
    output.initialized = 1;
  end

  %% RMS Calculations
  if(output.compute_rms == 1)
    output.rms_temp1 = output.rms_temp1 + input.value^2.0;
  else
    output.rms_temp1 = 0;
  end
  output.value_buffer(length(output.value_buffer)+1) = input.value;
  if(length(output.value_buffer) > RINGBUFFER_SIZE)
    output.value_buffer(1) = [];
    output.rms_temp1 = output.rms_temp1 - output.value_buffer(1)^2.0;
  end

  if(output.compute_rms == 1)
    if(length(output.value_buffer) > 2)
      output.rms = ((1/(length(output.value_buffer)))*(output.rms_temp1))^0.5;
    else
      output.rms = -1.0;
    end
  end
 
  %% Range/Dynamics Checks
  if(strcmp(input.sensorsource,'110012') == 1) % Razor IMUSensorPostProcess
    output.value = input.value;
  else
  end

  output.status = input.status;
  output.value = input.value;
  output.timestamp = input.timestamp;
  output.computed_signal = input.computed_signal;
  
  signals{i} = output;
end
a = 1;
end
a = 1;

