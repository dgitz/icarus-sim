%LoadTruthData
function [loaded, max_time,...
           yawrate,pitchrate,rollrate,...
           forwardaccel,lateralaccel,vertaccel,...
           forwardvel,lateralvel,vertvel,...
           yaw,pitch,roll,...
           east,north,elev] = Load_TruthSignals(scenario)
yawrate = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
pitchrate = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
rollrate = Initialize_Signal(SignalType.SIGNALTYPE_ROTATION_RATE_);
forwardaccel = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
lateralaccel = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
vertaccel = Initialize_Signal(SignalType.SIGNALTYPE_ACCELERATION_);
forwardvel = Initialize_Signal(SignalType.SIGNALTYPE_VELOCITY_);
lateralvel = Initialize_Signal(SignalType.SIGNALTYPE_VELOCITY_);
vertvel = Initialize_Signal(SignalType.SIGNALTYPE_VELOCITY_);
yaw = Initialize_Signal(SignalType.SIGNALTYPE_ANGLE_);
pitch = Initialize_Signal(SignalType.SIGNALTYPE_ANGLE_);
roll = Initialize_Signal(SignalType.SIGNALTYPE_ANGLE_);
east = Initialize_Signal(SignalType.SIGNALTYPE_DISTANCE_);
north = Initialize_Signal(SignalType.SIGNALTYPE_DISTANCE_);
elev = Initialize_Signal(SignalType.SIGNALTYPE_DISTANCE_);
SEQUENCE_COLUMN = 2; % MAY BE CHANGED WITH EROS MSG UPDATES
TIMESTAMP_COLUMN = 3;
YAWRATE_COLUMN = 7;
PITCHRATE_COLUMN = 13;
ROLLRATE_COLUMN = 19;
FORWARDACCEL_COLUMN = 25;
LATERALACCEL_COLUMN = 31;
VERTICALACCEL_COLUMN = 37;
FORWARDVELOCITY_COLUMN = 43;
LATERALVELOCITY_COLUMN = 49;
VERTICALVELOCITY_COLUMN = 55;
YAW_COLUMN = 61;
PITCH_COLUMN = 67;
ROLL_COLUMN = 73;
EAST_COLUMN = 79;
NORTH_COLUMN = 85;
ELEV_COLUMN = 91;




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
for i = 3:length(listing)
  is_lockfile = strfind(listing(i).name,'lock');
  if(is_lockfile > 0)
    continue;
  end
  v = strfind(listing(i).name,'TruthPose');
  if(v > 0)
    truth_name = listing(i).name(1:end-4);
    disp(['Reading Truth: ' truth_name ' Data']);
    data = readtable([scenario '/' listing(i).name]);
    timestamp = table2array(data(:,TIMESTAMP_COLUMN));
    timestamp = timestamp(:)-timestamp(1);
    %quick_timestamp = 0:1/TimeCompensateConfig.QUICK_ANALYZE_RATE:timestamp(end);
    max_dt = max(diff(timestamp));
    if(max_dt > TimeCompensateConfig.TOVDELTA_ERROR)
        disp(['[WARN]: File: ' listing(i).name ' Has a Max Delta Timestamp: ' num2str(max_dt) ' > ' num2str(TimeCompensateConfig.TOVDELTA_ERROR)]);
    end
    max_time = timestamp(end);
    
    yawrate_value = table2array(data(:,YAWRATE_COLUMN));
    yawrate_status = table2array(data(:,YAWRATE_COLUMN+1));
    yawrate_rms = table2array(data(:,YAWRATE_COLUMN+2));
    yawrate_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    pitchrate_value = table2array(data(:,PITCHRATE_COLUMN));
    pitchrate_status = table2array(data(:,PITCHRATE_COLUMN+1));
    pitchrate_rms = table2array(data(:,PITCHRATE_COLUMN+2));
    pitchrate_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    rollrate_value = table2array(data(:,ROLLRATE_COLUMN));
    rollrate_status = table2array(data(:,ROLLRATE_COLUMN+1));
    rollrate_rms = table2array(data(:,ROLLRATE_COLUMN+2));
    rollrate_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    forwardaccel_value = table2array(data(:,FORWARDACCEL_COLUMN));
    forwardaccel_status = table2array(data(:,FORWARDACCEL_COLUMN+1));
    forwardaccel_rms = table2array(data(:,FORWARDACCEL_COLUMN+2));
    forwardaccel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    lateralaccel_value = table2array(data(:,LATERALACCEL_COLUMN));
    lateralaccel_status = table2array(data(:,LATERALACCEL_COLUMN+1));
    lateralaccel_rms = table2array(data(:,LATERALACCEL_COLUMN+2));
    lateralaccel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    vertaccel_value = table2array(data(:,VERTICALACCEL_COLUMN));
    vertaccel_status = table2array(data(:,VERTICALACCEL_COLUMN+1));
    vertaccel_rms = table2array(data(:,VERTICALACCEL_COLUMN+2));
    vertaccel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    forwardvel_value = table2array(data(:,FORWARDVELOCITY_COLUMN));
    forwardvel_status = table2array(data(:,FORWARDVELOCITY_COLUMN+1));
    forwardvel_rms = table2array(data(:,FORWARDVELOCITY_COLUMN+2));
    forwardvel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    lateralvel_value = table2array(data(:,LATERALVELOCITY_COLUMN));
    lateralvel_status = table2array(data(:,LATERALVELOCITY_COLUMN+1));
    lateralvel_rms = table2array(data(:,LATERALVELOCITY_COLUMN+2));
    lateralvel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    vertvel_value = table2array(data(:,VERTICALVELOCITY_COLUMN));
    vertvel_status = table2array(data(:,VERTICALVELOCITY_COLUMN+1));
    vertvel_rms = table2array(data(:,VERTICALVELOCITY_COLUMN+2));
    vertvel_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    yaw_value = table2array(data(:,YAW_COLUMN));
    yaw_status = table2array(data(:,YAW_COLUMN+1));
    yaw_rms = table2array(data(:,YAW_COLUMN+2));
    yaw_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    pitch_value = table2array(data(:,PITCH_COLUMN));
    pitch_status = table2array(data(:,PITCH_COLUMN+1));
    pitch_rms = table2array(data(:,PITCH_COLUMN+2));
    pitch_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    roll_value = table2array(data(:,ROLL_COLUMN));
    roll_status = table2array(data(:,ROLL_COLUMN+1));
    roll_rms = table2array(data(:,ROLL_COLUMN+2));
    roll_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    east_value = table2array(data(:,EAST_COLUMN));
    east_status = table2array(data(:,EAST_COLUMN+1));
    east_rms = table2array(data(:,EAST_COLUMN+2));
    east_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    north_value = table2array(data(:,NORTH_COLUMN));
    north_status = table2array(data(:,NORTH_COLUMN+1));
    north_rms = table2array(data(:,NORTH_COLUMN+2));
    north_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    elev_value = table2array(data(:,ELEV_COLUMN));
    elev_status = table2array(data(:,ELEV_COLUMN+1));
    elev_rms = table2array(data(:,ELEV_COLUMN+2));
    elev_sequence_number = table2array(data(:,SEQUENCE_COLUMN));
    
    yawrate.available = 1;
    yawrate.time = timestamp;
    yawrate.signals.values = [yawrate_value';yawrate_status';yawrate_rms';yawrate_sequence_number';]';
    
    pitchrate.available = 1;
    pitchrate.time = timestamp;
    pitchrate.signals.values = [pitchrate_value';pitchrate_status';pitchrate_rms';pitchrate_sequence_number';]';
    
    rollrate.available = 1;
    rollrate.time = timestamp;
    rollrate.signals.values = [rollrate_value';rollrate_status';rollrate_rms';rollrate_sequence_number';]';
    
    forwardaccel.available = 1;
    forwardaccel.time = timestamp;
    forwardaccel.signals.values = [forwardaccel_value';forwardaccel_status';forwardaccel_rms';forwardaccel_sequence_number';]';
      
    lateralaccel.available = 1;
    lateralaccel.time = timestamp;
    lateralaccel.signals.values = [lateralaccel_value';lateralaccel_status';lateralaccel_rms';lateralaccel_sequence_number';]';

    vertaccel.available = 1;
    vertaccel.time = timestamp;
    vertaccel.signals.values = [vertaccel_value';vertaccel_status';vertaccel_rms';vertaccel_sequence_number';]';

    forwardvel.available = 1;
    forwardvel.time = timestamp;
    forwardvel.signals.values = [forwardvel_value';forwardvel_status';forwardvel_rms';forwardvel_sequence_number';]';
    
    lateralvel.available = 1;
    lateralvel.time = timestamp;
    lateralvel.signals.values = [lateralvel_value';lateralvel_status';lateralvel_rms';lateralvel_sequence_number';]';
    
    vertvel.available = 1;
    vertvel.time = timestamp;
    vertvel.signals.values = [vertvel_value';vertvel_status';vertvel_rms';vertvel_sequence_number';]';
    
    yaw.available = 1;
    yaw.time = timestamp;
    yaw.signals.values = [yaw_value';yaw_status';yaw_rms';yaw_sequence_number';]';
    
    pitch.available = 1;
    pitch.time = timestamp;
    pitch.signals.values = [pitch_value';pitch_status';pitch_rms';pitch_sequence_number';]';
    
    roll.available = 1;
    roll.time = timestamp;
    roll.signals.values = [roll_value';roll_status';roll_rms';roll_sequence_number';]';
    
    east.available = 1;
    east.time = timestamp;
    east.signals.values = [east_value';east_status';east_rms';east_sequence_number';]';
    
    north.available = 1;
    north.time = timestamp;
    north.signals.values = [north_value';north_status';north_rms';north_sequence_number';]';
    
    elev.available = 1;
    elev.time = timestamp;
    elev.signals.values = [elev_value';elev_status';elev_rms';elev_sequence_number';]';

  end
end

disp('Truth Data Load Complete.');
loaded = 1;