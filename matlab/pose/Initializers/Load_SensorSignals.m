%LoadSensorData
function [loaded, max_time,...
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
            mag4x_in,mag4y_in,mag4z_in, ...
            odom1x_in,odom1y_in,odom1z_in, ...
            odom2x_in,odom2y_in,odom2z_in, ...
            odom3x_in,odom3y_in,odom3z_in, ...
            odom4x_in,odom4y_in,odom4z_in] = Load_SensorSignals(scenario)
odom1x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom1y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom1z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom2x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom2y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom2z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom3x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom3y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom3z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom4x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom4y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
odom4z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
accel1x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel2x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel3x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel4x_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel1y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel2y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel3y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel4y_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel1z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel2z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel3z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
accel4z_in = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
rotationrate1x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate2x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate3x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate4x_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate1y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate2y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate3y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate4y_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate1z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate2z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate3z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rotationrate4z_in = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
mag1x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag2x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag3x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag4x_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag1y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag2y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag3y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag4y_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag1z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag2z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag3z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
mag4z_in = Initialize_Signal(SignalType.SIGNALTYPE_MAGNETIC_FIELD_);
IMU_SEQUENCE_COLUMN = 2; % MAY BE CHANGED WITH EROS MSG UPDATES
IMU_TIMESTAMP_COLUMN = 3;
IMU_XACC_COLUMN = 7;
IMU_YACC_COLUMN = 13;
IMU_ZACC_COLUMN = 19;
IMU_XGYRO_COLUMN = 25;
IMU_YGYRO_COLUMN = 31;
IMU_ZGYRO_COLUMN = 37;
IMU_XMAG_COLUMN = 43;
IMU_YMAG_COLUMN = 49;
IMU_ZMAG_COLUMN = 55;

ODOM_SEQUENCE_COLUMN = 2;
ODOM_TIMESTAMP_COLUMN = 3;
ODOM_XODOM_COLUMN = 7;
ODOM_YODOM_COLUMN = 13;
ODOM_ZODOM_COLUMN = 19;

max_time = 0;
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
odomindex = 0;
ODOM_Count = 0;
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
    disp(['Reading IMU: ' imu_name ' Data']);
    data = readtable([scenario '/' listing(i).name]);
    timestamp = table2array(data(:,IMU_TIMESTAMP_COLUMN));
    timestamp = timestamp(:)-timestamp(1);
    max_dt = max(diff(timestamp));
    if(max_dt > TimeCompensateConfig.TOVDELTA_ERROR)
        disp(['[WARN]: File: ' listing(i).name ' Has a Max Delta Timestamp: ' num2str(max_dt) ' > ' num2str(TimeCompensateConfig.TOVDELTA_ERROR)]);
    end
    log_end = timestamp(end);
    if(log_end > max_time)
        max_time = log_end;
    end
    
    xacc_value = table2array(data(:,IMU_XACC_COLUMN));
    xacc_status = table2array(data(:,IMU_XACC_COLUMN+1));
    xacc_rms = table2array(data(:,IMU_XACC_COLUMN+2));
    xacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    yacc_value = table2array(data(:,IMU_YACC_COLUMN));
    yacc_status = table2array(data(:,IMU_YACC_COLUMN+1));
    yacc_rms = table2array(data(:,IMU_YACC_COLUMN+2));
    yacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zacc_value = table2array(data(:,IMU_ZACC_COLUMN));
    zacc_status = table2array(data(:,IMU_ZACC_COLUMN+1));
    zacc_rms = table2array(data(:,IMU_ZACC_COLUMN+2));
    zacc_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    xgyro_value = table2array(data(:,IMU_XGYRO_COLUMN));
    xgyro_status = table2array(data(:,IMU_XGYRO_COLUMN+1));
    xgyro_rms = table2array(data(:,IMU_XGYRO_COLUMN+2));
    xgyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    ygyro_value = table2array(data(:,IMU_YGYRO_COLUMN));
    ygyro_status = table2array(data(:,IMU_YGYRO_COLUMN+1));
    ygyro_rms = table2array(data(:,IMU_YGYRO_COLUMN+2));
    ygyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zgyro_value = table2array(data(:,IMU_ZGYRO_COLUMN));
    zgyro_status = table2array(data(:,IMU_ZGYRO_COLUMN+1));
    zgyro_rms = table2array(data(:,IMU_ZGYRO_COLUMN+2));
    zgyro_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    xmag_value = table2array(data(:,IMU_XMAG_COLUMN));
    xmag_status = table2array(data(:,IMU_XMAG_COLUMN+1));
    xmag_rms = table2array(data(:,IMU_XMAG_COLUMN+2));
    xmag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    ymag_value = table2array(data(:,IMU_YMAG_COLUMN));
    ymag_status = table2array(data(:,IMU_YMAG_COLUMN+1));
    ymag_rms = table2array(data(:,IMU_YMAG_COLUMN+2));
    ymag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    zmag_value = table2array(data(:,IMU_ZMAG_COLUMN));
    zmag_status = table2array(data(:,IMU_ZMAG_COLUMN+1));
    zmag_rms = table2array(data(:,IMU_ZMAG_COLUMN+2));
    zmag_sequence_number = table2array(data(:,IMU_SEQUENCE_COLUMN));
    
    if(imuindex == 1)
        accel1x_in.available = 1;
        accel1x_in.time = timestamp;
        accel1x_in.signals.values = [xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel1y_in.available = 1;
        accel1y_in.time = timestamp;
        accel1y_in.signals.values = [yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel1z_in.available = 1;
        accel1z_in.time = timestamp;
        accel1z_in.signals.values = [zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate1x_in.available = 1;
        rotationrate1x_in.time = timestamp;
        rotationrate1x_in.signals.values = [xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate1x_in.available = 1;
        rotationrate1y_in.time = timestamp;
        rotationrate1y_in.signals.values = [ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate1z_in.available = 1;
        rotationrate1z_in.time = timestamp;
        rotationrate1z_in.signals.values = [zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag1x_in.available = 1;
        mag1x_in.time = timestamp;
        mag1x_in.signals.values = [xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag1x_in.available = 1;
        mag1y_in.time = timestamp;
        mag1y_in.signals.values = [ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag1z_in.available = 1;
        mag1z_in.time = timestamp;
        mag1z_in.signals.values = [zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
        
    end
    if(imuindex == 2)
        accel2x_in.available = 1;
        accel2x_in.time = timestamp;
        accel2x_in.signals.values = [xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel2y_in.available = 1;
        accel2y_in.time = timestamp;
        accel2y_in.signals.values = [yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel2z_in.available = 1;
        accel2z_in.time = timestamp;
        accel2z_in.signals.values = [zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate2x_in.available = 1;
        rotationrate2x_in.time = timestamp;
        rotationrate2x_in.signals.values = [xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate2x_in.available = 1;
        rotationrate2y_in.time = timestamp;
        rotationrate2y_in.signals.values = [ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate2z_in.available = 1;
        rotationrate2z_in.time = timestamp;
        rotationrate2z_in.signals.values = [zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag2x_in.available = 1;
        mag2x_in.time = timestamp;
        mag2x_in.signals.values = [xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag2x_in.available = 1;
        mag2y_in.time = timestamp;
        mag2y_in.signals.values = [ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag2z_in.available = 1;
        mag2z_in.time = timestamp;
        mag2z_in.signals.values = [zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
    if(imuindex == 3)
       accel3x_in.available = 1;
        accel3x_in.time = timestamp;
        accel3x_in.signals.values = [xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel3y_in.available = 1;
        accel3y_in.time = timestamp;
        accel3y_in.signals.values = [yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel3z_in.available = 1;
        accel3z_in.time = timestamp;
        accel3z_in.signals.values = [zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate3x_in.available = 1;
        rotationrate3x_in.time = timestamp;
        rotationrate3x_in.signals.values = [xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate3x_in.available = 1;
        rotationrate3y_in.time = timestamp;
        rotationrate3y_in.signals.values = [ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate3z_in.available = 1;
        rotationrate3z_in.time = timestamp;
        rotationrate3z_in.signals.values = [zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag3x_in.available = 1;
        mag3x_in.time = timestamp;
        mag3x_in.signals.values = [xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag3x_in.available = 1;
        mag3y_in.time = timestamp;
        mag3y_in.signals.values = [ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag3z_in.available = 1;
        mag3z_in.time = timestamp;
        mag3z_in.signals.values = [zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
    if(imuindex == 4)
        accel4x_in.available = 1;
        accel4x_in.time = timestamp;
        accel4x_in.signals.values = [xacc_value';xacc_status';xacc_rms';xacc_sequence_number';]';
        accel4y_in.available = 1;
        accel4y_in.time = timestamp;
        accel4y_in.signals.values = [yacc_value';yacc_status';yacc_rms';yacc_sequence_number';]';
        accel4z_in.available = 1;
        accel4z_in.time = timestamp;
        accel4z_in.signals.values = [zacc_value';zacc_status';zacc_rms';zacc_sequence_number';]';
        
        rotationrate4x_in.available = 1;
        rotationrate4x_in.time = timestamp;
        rotationrate4x_in.signals.values = [xgyro_value';xgyro_status';xgyro_rms';xgyro_sequence_number';]';
        rotationrate4x_in.available = 1;
        rotationrate4y_in.time = timestamp;
        rotationrate4y_in.signals.values = [ygyro_value';ygyro_status';ygyro_rms';ygyro_sequence_number';]';
        rotationrate4z_in.available = 1;
        rotationrate4z_in.time = timestamp;
        rotationrate4z_in.signals.values = [zgyro_value';zgyro_status';zgyro_rms';zgyro_sequence_number';]';
        
        mag4x_in.available = 1;
        mag4x_in.time = timestamp;
        mag4x_in.signals.values = [xmag_value';xmag_status';xmag_rms';xmag_sequence_number';]';
        mag4x_in.available = 1;
        mag4y_in.time = timestamp;
        mag4y_in.signals.values = [ymag_value';ymag_status';ymag_rms';ymag_sequence_number';]';
        mag4z_in.available = 1;
        mag4z_in.time = timestamp;
        mag4z_in.signals.values = [zmag_value';zmag_status';zmag_rms';zmag_sequence_number';]';
    end
  end
  v = strfind(listing(i).name,'Encoder');
  if(v > 0)
    ODOM_Count = ODOM_Count + 1;
    odomindex = odomindex + 1;
    odom_name = listing(i).name(1:end-4);
    disp(['Reading ODOM: ' odom_name ' Data']);
    data = readtable([scenario '/' listing(i).name]);
    timestamp = table2array(data(:,ODOM_TIMESTAMP_COLUMN));
    timestamp = timestamp(:)-timestamp(1);
    datatime_end = timestamp(end);
    
    xodom_value = table2array(data(:,ODOM_XODOM_COLUMN));
    xodom_status = table2array(data(:,ODOM_XODOM_COLUMN+1));
    xodom_rms = table2array(data(:,ODOM_XODOM_COLUMN+2));
    xodom_sequence_number = table2array(data(:,ODOM_SEQUENCE_COLUMN));
    
    yodom_value = table2array(data(:,ODOM_YODOM_COLUMN));
    yodom_status = table2array(data(:,ODOM_YODOM_COLUMN+1));
    yodom_rms = table2array(data(:,ODOM_YODOM_COLUMN+2));
    yodom_sequence_number = table2array(data(:,ODOM_SEQUENCE_COLUMN));
    
    zodom_value = table2array(data(:,ODOM_ZODOM_COLUMN));
    zodom_status = table2array(data(:,ODOM_ZODOM_COLUMN+1));
    zodom_rms = table2array(data(:,ODOM_ZODOM_COLUMN+2));
    zodom_sequence_number = table2array(data(:,ODOM_SEQUENCE_COLUMN));
    if(odomindex == 1)
        odom1x_in.available = 1;
        odom1x_in.time = timestamp;
        odom1x_in.signals.values = [xodom_value';xodom_status';xodom_rms';xodom_sequence_number';]';
        odom1y_in.available = 1;
        odom1y_in.time = timestamp;
        odom1y_in.signals.values = [yodom_value';yodom_status';yodom_rms';yodom_sequence_number';]';
        odom1z_in.available = 1;
        odom1z_in.time = timestamp;
        odom1z_in.signals.values = [zodom_value';zodom_status';zodom_rms';zodom_sequence_number';]';
    end
    if(odomindex == 2)
        odom2x_in.available = 1;
        odom2x_in.time = timestamp;
        odom2x_in.signals.values = [xodom_value';xodom_status';xodom_rms';xodom_sequence_number';]';
        odom2y_in.available = 1;
        odom2y_in.time = timestamp;
        odom2y_in.signals.values = [yodom_value';yodom_status';yodom_rms';yodom_sequence_number';]';
        odom2z_in.available = 1;
        odom2z_in.time = timestamp;
        odom2z_in.signals.values = [zodom_value';zodom_status';zodom_rms';zodom_sequence_number';]';
    end
    if(odomindex == 3)
        odom3x_in.available = 1;
        odom3x_in.time = timestamp;
        odom3x_in.signals.values = [xodom_value';xodom_status';xodom_rms';xodom_sequence_number';]';
        odom3y_in.available = 1;
        odom3y_in.time = timestamp;
        odom3y_in.signals.values = [yodom_value';yodom_status';yodom_rms';yodom_sequence_number';]';
        odom3z_in.available = 1;
        odom3z_in.time = timestamp;
        odom3z_in.signals.values = [zodom_value';zodom_status';zodom_rms';zodom_sequence_number';]';
    end
    if(odomindex == 4)
        odom4x_in.available = 1;
        odom4x_in.time = timestamp;
        odom4x_in.signals.values = [xodom_value';xodom_status';xodom_rms';xodom_sequence_number';]';
        odom4y_in.available = 1;
        odom4y_in.time = timestamp;
        odom4y_in.signals.values = [yodom_value';yodom_status';yodom_rms';yodom_sequence_number';]';
        odom4z_in.available = 1;
        odom4z_in.time = timestamp;
        odom4z_in.signals.values = [zodom_value';zodom_status';zodom_rms';zodom_sequence_number';]';
    end
  end
end


disp('Sensor Data Load Complete.');
loaded = 1;