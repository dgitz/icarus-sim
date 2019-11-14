%LoadSensorData
function [loaded,Sensor_Signals,IMU_Count] = Load_SensorSignals(scenario)
global OPERATION_MODE;
global SIGNAL_STATUS;
global VARIANCE_BUFFER_SIZE;
Sensor_Signals = [];
IMU_Raw = [];
IMU_SEQUENCE_COLUMN = 2; % MAY BE CHANGED WITH EROS MSG UPDATES
IMU_XACC_COLUMN = 7;
IMU_YACC_COLUMN = 11;
IMU_ZACC_COLUMN = 15;
IMU_XGYRO_COLUMN = 19;
IMU_YGYRO_COLUMN = 23;
IMU_ZGYRO_COLUMN = 27;
IMU_XMAG_COLUMN = 31;
IMU_YMAG_COLUMN = 35;
IMU_ZMAG_COLUMN = 39;

listing = dir(scenario);
if(length(listing) == 0)
  disp(['Scenario: ' scenario ' Contains no Data Files. Exiting.']);
  return
end
imuindex = 1;
IMU_Count = 0;
for i = 3:length(listing)
  is_lockfile = strfind(listing(i).name,'lock');
  if(is_lockfile > 0)
    continue;
  end
  v = strfind(listing(i).name,'IMU');
  if(v > 0)
    IMU_Count = IMU_Count + 1;
    index = str2num(listing(i).name(4:strfind(listing(i).name,'.')-1));
    disp(['Reading IMU' num2str(index) ' Data']);
    fid = fopen([scenario '/' listing(i).name]);
    line = fgetl(fid);
    fclose(fid);
    cols = strsplit(line,',');
    data = csvread([scenario '/' listing(i).name],1,0);
    for j = 1:length(cols)
      v = strfind(cols{j},'field.');
      if(v > 0)
        cols{j} = cols{j}(7:end);
      end
    end
    timestamp = data(:,1)/1000000000;    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_XACC_COLUMN}(1:strfind(cols{IMU_XACC_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'meter/s^2';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_XACC_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_XACC_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_XACC_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Linear Acceleration";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_YACC_COLUMN}(1:strfind(cols{IMU_YACC_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'meter/s^2';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_YACC_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_YACC_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_YACC_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Linear Acceleration";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_ZACC_COLUMN}(1:strfind(cols{IMU_ZACC_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'meter/s^2';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_ZACC_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_ZACC_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_ZACC_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Linear Acceleration";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_XGYRO_COLUMN}(1:strfind(cols{IMU_XGYRO_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'deg/s';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_XGYRO_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_XGYRO_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_XGYRO_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_YGYRO_COLUMN}(1:strfind(cols{IMU_YGYRO_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'deg/s';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_YGYRO_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_YGYRO_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_YGYRO_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_ZGYRO_COLUMN}(1:strfind(cols{IMU_ZGYRO_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'deg/s';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_ZGYRO_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_ZGYRO_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_ZGYRO_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_XMAG_COLUMN}(1:strfind(cols{IMU_XMAG_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'uTesla';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_XMAG_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_XMAG_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_XMAG_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Magnetic Field";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_YMAG_COLUMN}(1:strfind(cols{IMU_YMAG_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'uTesla';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_YMAG_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_YMAG_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_YMAG_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Magnetic Field";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    
    IMU_Raw(length(IMU_Raw)+1).name = [cols{IMU_ZMAG_COLUMN}(1:strfind(cols{IMU_ZMAG_COLUMN},'.')-1) num2str(index)];
    IMU_Raw(length(IMU_Raw)).sequence_number = data(:,IMU_SEQUENCE_COLUMN);
    IMU_Raw(length(IMU_Raw)).units = 'uTesla';
    IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
    IMU_Raw(length(IMU_Raw)).value = data(:,IMU_ZMAG_COLUMN);
    IMU_Raw(length(IMU_Raw)).status = data(:,IMU_ZMAG_COLUMN+1);
    IMU_Raw(length(IMU_Raw)).rms = data(:,IMU_ZMAG_COLUMN+2);
    IMU_Raw(length(IMU_Raw)).type = "Magnetic Field";
    IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
    IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
    IMU_Raw(length(IMU_Raw)).sensorindex = index;
    IMU_Raw(length(IMU_Raw)).computed_signal = 0;
    if(0)
      %Add Extra IMU Signals as Appropriate
      IMU_Raw(length(IMU_Raw)+1).name = ["mag_yaw" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'degree';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
      
      IMU_Raw(length(IMU_Raw)+1).name = ["acc_roll" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'degree';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
      
      IMU_Raw(length(IMU_Raw)+1).name = ["acc_pitch" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'degree';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
      
      
      
      IMU_Raw(length(IMU_Raw)+1).name = ["d_acc_roll" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'deg/s';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
      
      IMU_Raw(length(IMU_Raw)+1).name = ["d_acc_pitch" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'deg/s';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
      
      IMU_Raw(length(IMU_Raw)+1).name = ["d_mag_yaw" num2str(index)];
      IMU_Raw(length(IMU_Raw)).units = 'deg/s';
      IMU_Raw(length(IMU_Raw)).timestamp = timestamp;
      IMU_Raw(length(IMU_Raw)).rms = -1;
      IMU_Raw(length(IMU_Raw)).type = "Angle Rate";
      IMU_Raw(length(IMU_Raw)).sensorsource = "110012";
      IMU_Raw(length(IMU_Raw)).sensorname = ["IMU" num2str(index)]; 
      IMU_Raw(length(IMU_Raw)).sensorindex = index;
      IMU_Raw(length(IMU_Raw)).computed_signal = 1;
    end
    
 
  end
end
for i = 1:length(IMU_Raw)
  sensor_signal_obj = Initialize_SensorSignal;
  sensor_signal_obj.name = IMU_Raw(i).name;
  [conversion_factor,signal_type] = convert_signaltype(IMU_Raw(i).units);
  sensor_signal_obj.type = signal_type;
  signal_vector = [];
  for j = 1:length(IMU_Raw(i).timestamp)
    sig = sensor_signal_obj;
    sig.sequence_number = IMU_Raw(i).sequence_number(j);
    sig.value = conversion_factor*IMU_Raw(i).value(j);
    sig.tov = IMU_Raw(i).timestamp(j);
    sig.status = IMU_Raw(i).status(j);
    sig.rms = IMU_Raw(i).rms(j)*conversion_factor;
    signal_vector = [signal_vector sig];
  end
  if(i == 10)
    a = 1;
  end
  Sensor_Signals{i} = signal_vector;
end
loaded = 1;