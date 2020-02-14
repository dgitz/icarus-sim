%LoadSensorData
function [loaded, datatime_end,...
            accel1x_in,accel1y_in,accel1z_in, ...
            accel2x_in,accel2y_in,accel2z_in, ...
            accel3x_in,accel3y_in,accel3z_in, ...
            accel4x_in,accel4y_in,accel4z_in, ...
            rotationrate1x_in,rotationrate1y_in,rotationrate1z_in, ...
            rotationrate2x_in,rotationrate2y_in,rotationrate2z_in, ...
            rotationrate3x_in,rotationrate3y_in,rotationrate3z_in, ...
            rotationrate4x_in,rotationrate4y_in,rotationrate4z_in, ...
            mag1x_in,mag1y_in,mag1z_in, ...
            mag2x_in,mag2y_in,mag2z_in, ...
            mag3x_in,mag3y_in,mag3z_in, ...
            mag4x_in,mag4y_in,mag4z_in] = Load_SensorSignals(scenario)
global OPERATION_MODE;
global SIGNAL_STATUS;
global VARIANCE_BUFFER_SIZE;
global SignalType
Sensor_Signals = [];
accel1x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel2x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel3x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel4x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel1y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel2y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel3y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel4y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel1z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel2z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel3z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
accel4z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION);
rotationrate1x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate2x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate3x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate4x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate1y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate2y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate3y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate4y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate1z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate2z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate3z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
rotationrate4z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE);
mag1x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag2x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag3x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag4x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag1y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag2y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag3y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag4y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag1z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag2z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag3z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
mag4z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD);
IMU_Raw = [];
IMU_SEQUENCE_COLUMN = 3; % MAY BE CHANGED WITH EROS MSG UPDATES
IMU_XACC_COLUMN = 8;
IMU_YACC_COLUMN = 14;
IMU_ZACC_COLUMN = 20;
IMU_XGYRO_COLUMN = 26;
IMU_YGYRO_COLUMN = 32;
IMU_ZGYRO_COLUMN = 38;
IMU_XMAG_COLUMN = 44;
IMU_YMAG_COLUMN = 50;
IMU_ZMAG_COLUMN = 56;
if(isempty(scenario) == 1)
  disp(['WARN: Scenario Not Defined. Not loading any data. ']);
  loaded = 0;
  return;
end
listing = dir(scenario);
if(length(listing) == 0)
  disp(['Scenario: ' scenario ' Contains no Data Files. Exiting.']);
  return
end
imuindex = 0;
IMU_Count = 0;
for i = 3:length(listing)
  is_lockfile = strfind(listing(i).name,'lock');
  if(is_lockfile > 0)
    continue;
  end
  v = strfind(listing(i).name,'IMU');
  if(v > 0)
    IMU_Count = IMU_Count + 1;
    imuindex = imuindex + 1;
    imu_name = listing(i).name(1:end-4);
    disp(['Reading IMU' imu_name ' Data']);
    data = readtable([scenario '/' listing(i).name]);
    timestamp = table2array(data(:,4));
    timestamp = timestamp(:)-timestamp(1);
    datatime_end = timestamp(end);
    
    xacc_value = table2array(data(:,IMU_XACC_COLUMN));
    xacc_status = table2array(data(:,IMU_XACC_COLUMN+1));
    xacc_rms = table2array(data(:,IMU_XACC_COLUMN+2));
    xacc_type = SignalType.SIGNALTYPE_ACCELERATION;
    xacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    yacc_value = table2array(data(:,IMU_YACC_COLUMN));
    yacc_status = table2array(data(:,IMU_YACC_COLUMN+1));
    yacc_rms = table2array(data(:,IMU_YACC_COLUMN+2));
    yacc_type = SignalType.SIGNALTYPE_ACCELERATION;
    yacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zacc_value = table2array(data(:,IMU_ZACC_COLUMN));
    zacc_status = table2array(data(:,IMU_ZACC_COLUMN+1));
    zacc_rms = table2array(data(:,IMU_ZACC_COLUMN+2));
    zacc_type = SignalType.SIGNALTYPE_ACCELERATION;
    zacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    xgyro_value = table2array(data(:,IMU_XGYRO_COLUMN));
    xgyro_status = table2array(data(:,IMU_XGYRO_COLUMN+1));
    xgyro_rms = table2array(data(:,IMU_XGYRO_COLUMN+2));
    xgyro_type = SignalType.SIGNALTYPE_ROTATION_RATE;
    xgyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    ygyro_value = table2array(data(:,IMU_YGYRO_COLUMN));
    ygyro_status = table2array(data(:,IMU_YGYRO_COLUMN+1));
    ygyro_rms = table2array(data(:,IMU_YGYRO_COLUMN+2));
    ygyro_type = SignalType.SIGNALTYPE_ROTATION_RATE;
    ygyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zgyro_value = table2array(data(:,IMU_ZGYRO_COLUMN));
    zgyro_status = table2array(data(:,IMU_ZGYRO_COLUMN+1));
    zgyro_rms = table2array(data(:,IMU_ZGYRO_COLUMN+2));
    zgyro_type = SignalType.SIGNALTYPE_ROTATION_RATE;
    zgyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    xmag_value = table2array(data(:,IMU_XMAG_COLUMN));
    xmag_status = table2array(data(:,IMU_XMAG_COLUMN+1));
    xmag_rms = table2array(data(:,IMU_XMAG_COLUMN+2));
    xmag_type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
    xmag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    ymag_value = table2array(data(:,IMU_YMAG_COLUMN));
    ymag_status = table2array(data(:,IMU_YMAG_COLUMN+1));
    ymag_rms = table2array(data(:,IMU_YMAG_COLUMN+2));
    ymag_type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
    ymag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zmag_value = table2array(data(:,IMU_ZMAG_COLUMN));
    zmag_status = table2array(data(:,IMU_ZMAG_COLUMN+1));
    zmag_rms = table2array(data(:,IMU_ZMAG_COLUMN+2));
    zmag_type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
    zmag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    if(imuindex == 1)
        accel1x_in.available = 1;
        accel1x_in.time = timestamp;
        accel1x_in.signals.values = [xacc_type*ones(length(timestamp),1)';xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel1y_in.available = 1;
        accel1y_in.time = timestamp;
        accel1y_in.signals.values = [yacc_type*ones(length(timestamp),1)';yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel1z_in.available = 1;
        accel1z_in.time = timestamp;
        accel1z_in.signals.values = [zacc_type*ones(length(timestamp),1)';zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate1x_in.available = 1;
        rotationrate1x_in.time = timestamp;
        rotationrate1x_in.signals.values = [xgyro_type*ones(length(timestamp),1)';xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate1x_in.available = 1;
        rotationrate1y_in.time = timestamp;
        rotationrate1y_in.signals.values = [ygyro_type*ones(length(timestamp),1)';ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate1z_in.available = 1;
        rotationrate1z_in.time = timestamp;
        rotationrate1z_in.signals.values = [zgyro_type*ones(length(timestamp),1)';zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag1x_in.available = 1;
        mag1x_in.time = timestamp;
        mag1x_in.signals.values = [xmag_type*ones(length(timestamp),1)';xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag1x_in.available = 1;
        mag1y_in.time = timestamp;
        mag1y_in.signals.values = [ymag_type*ones(length(timestamp),1)';ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag1z_in.available = 1;
        mag1z_in.time = timestamp;
        mag1z_in.signals.values = [zmag_type*ones(length(timestamp),1)';zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
        
    end
    if(imuindex == 2)
        accel2x_in.available = 1;
        accel2x_in.time = timestamp;
        accel2x_in.signals.values = [xacc_type*ones(length(timestamp),1)';xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel2y_in.available = 1;
        accel2y_in.time = timestamp;
        accel2y_in.signals.values = [yacc_type*ones(length(timestamp),1)';yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel2z_in.available = 1;
        accel2z_in.time = timestamp;
        accel2z_in.signals.values = [zacc_type*ones(length(timestamp),1)';zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate2x_in.available = 1;
        rotationrate2x_in.time = timestamp;
        rotationrate2x_in.signals.values = [xgyro_type*ones(length(timestamp),1)';xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate2x_in.available = 1;
        rotationrate2y_in.time = timestamp;
        rotationrate2y_in.signals.values = [ygyro_type*ones(length(timestamp),1)';ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate2z_in.available = 1;
        rotationrate2z_in.time = timestamp;
        rotationrate2z_in.signals.values = [zgyro_type*ones(length(timestamp),1)';zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag2x_in.available = 1;
        mag2x_in.time = timestamp;
        mag2x_in.signals.values = [xmag_type*ones(length(timestamp),1)';xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag2x_in.available = 1;
        mag2y_in.time = timestamp;
        mag2y_in.signals.values = [ymag_type*ones(length(timestamp),1)';ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag2z_in.available = 1;
        mag2z_in.time = timestamp;
        mag2z_in.signals.values = [zmag_type*ones(length(timestamp),1)';zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
    if(imuindex == 3)
       accel3x_in.available = 1;
        accel3x_in.time = timestamp;
        accel3x_in.signals.values = [xacc_type*ones(length(timestamp),1)';xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel3y_in.available = 1;
        accel3y_in.time = timestamp;
        accel3y_in.signals.values = [yacc_type*ones(length(timestamp),1)';yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel3z_in.available = 1;
        accel3z_in.time = timestamp;
        accel3z_in.signals.values = [zacc_type*ones(length(timestamp),1)';zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate3x_in.available = 1;
        rotationrate3x_in.time = timestamp;
        rotationrate3x_in.signals.values = [xgyro_type*ones(length(timestamp),1)';xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate3x_in.available = 1;
        rotationrate3y_in.time = timestamp;
        rotationrate3y_in.signals.values = [ygyro_type*ones(length(timestamp),1)';ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate3z_in.available = 1;
        rotationrate3z_in.time = timestamp;
        rotationrate3z_in.signals.values = [zgyro_type*ones(length(timestamp),1)';zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag3x_in.available = 1;
        mag3x_in.time = timestamp;
        mag3x_in.signals.values = [xmag_type*ones(length(timestamp),1)';xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag3x_in.available = 1;
        mag3y_in.time = timestamp;
        mag3y_in.signals.values = [ymag_type*ones(length(timestamp),1)';ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag3z_in.available = 1;
        mag3z_in.time = timestamp;
        mag3z_in.signals.values = [zmag_type*ones(length(timestamp),1)';zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
    if(imuindex == 4)
        accel4x_in.available = 1;
        accel4x_in.time = timestamp;
        accel4x_in.signals.values = [xacc_type*ones(length(timestamp),1)';xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel4y_in.available = 1;
        accel4y_in.time = timestamp;
        accel4y_in.signals.values = [yacc_type*ones(length(timestamp),1)';yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel4z_in.available = 1;
        accel4z_in.time = timestamp;
        accel4z_in.signals.values = [zacc_type*ones(length(timestamp),1)';zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate4x_in.available = 1;
        rotationrate4x_in.time = timestamp;
        rotationrate4x_in.signals.values = [xgyro_type*ones(length(timestamp),1)';xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate4x_in.available = 1;
        rotationrate4y_in.time = timestamp;
        rotationrate4y_in.signals.values = [ygyro_type*ones(length(timestamp),1)';ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate4z_in.available = 1;
        rotationrate4z_in.time = timestamp;
        rotationrate4z_in.signals.values = [zgyro_type*ones(length(timestamp),1)';zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag4x_in.available = 1;
        mag4x_in.time = timestamp;
        mag4x_in.signals.values = [xmag_type*ones(length(timestamp),1)';xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag4x_in.available = 1;
        mag4y_in.time = timestamp;
        mag4y_in.signals.values = [ymag_type*ones(length(timestamp),1)';ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag4z_in.available = 1;
        mag4z_in.time = timestamp;
        mag4z_in.signals.values = [zmag_type*ones(length(timestamp),1)';zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
  end
end
    
   
disp('Sensor Data Load Complete.');
loaded = 1;