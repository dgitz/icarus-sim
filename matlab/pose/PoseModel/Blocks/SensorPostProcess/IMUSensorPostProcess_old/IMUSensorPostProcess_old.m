%IMUSensorPostProcess
function [output_signals] = IMUSensorPostProcess(input_signals)
global SignalState
current_sensor_index = 1;
xmag = 0;
ymag = 0;
zmag = 0;
xmag_found = 0;
ymag_found = 0;
zmag_found = 0;
xacc = 0;
yacc = 0;
zacc = 0;
xacc_found = 0;
yacc_found = 0;
zacc_found = 0;
xmag_timestamp = 0;
ymag_timestamp = 0;
zmag_timestamp = 0;
output_signals = [];
phi = 0;
theta = 0;
phi_computed = 0;
theta_computed = 0;
for i = 1:length(input_signals)
  output_signals{length(output_signals)+1} = input_signals{i};
  if(strfind(input_signals{i}.name,"xmag") > 0)
    xmag = input_signals{i}.value;
    xmag_timestamp = input_signals{i}.timestamp;
    xmag_status = input_signals{i}.status;
    xmag_found = 1;
  elseif(strfind(input_signals{i}.name,"ymag") > 0)
    ymag = input_signals{i}.value;
    ymag_timestamp = input_signals{i}.timestamp;
    ymag_status = input_signals{i}.status;
    ymag_found = 1;
  elseif(strfind(input_signals{i}.name,"zmag") > 0)
    zmag = input_signals{i}.value;
    zmag_timestamp = input_signals{i}.timestamp;
    zmag_status = input_signals{i}.status;
    zmag_found = 1;
  elseif(strfind(input_signals{i}.name,"xacc") > 0)
    xacc = input_signals{i}.value;
    xacc_found = 1;
  elseif(strfind(input_signals{i}.name,"yacc") > 0)
    yacc = input_signals{i}.value;
    yacc_found = 1;
  elseif(strfind(input_signals{i}.name,"zacc") > 0)
    zacc = input_signals{i}.value;
    zacc_found = 1;
  end
  if((xmag_found == 1) && (ymag_found == 1) && (zmag_found == 1) && (xacc_found == 1) && (yacc_found == 1) && (zacc_found == 1))
    if((strfind(input_signals{i}.name,"acc_roll") > 0) && (phi_computed == 1))
      output_signals{i}.value = wrapangle(phi*180.0/pi);
      output_signals{i}.value = wrapangle(output_signals{i}.value);
      output_signals{i}.timestamp = max(ymag_timestamp,zmag_timestamp);
      output_signals{i}.status = xmag_status;
    elseif((strfind(input_signals{i}.name,"acc_pitch") > 0) && (theta_computed == 1))
      output_signals{i}.value = wrapangle(theta*180/pi);
      output_signals{i}.timestamp = max(xmag_timestamp,zmag_timestamp);
      output_signals{i}.status = ymag_status;
    elseif(strfind(input_signals{i}.name,"mag_yaw") > 0)
      vx = 0;
      vy = 0;
      vz = 0;
      Br = [1 1 1]';
      Bpx = xmag;
      Bpy = ymag;
      Bpz = zmag;
      phi = atan2(yacc,zacc);
      theta = atan2(-xacc,(yacc*sin(phi) + zacc*cos(phi)));
      yaw_num = (Bpz-vz)*sin(phi)-(Bpy-vy)*cos(phi);
      yaw_den = (Bpx-vx)*cos(theta)+(Bpy-vy)*sin(theta)*sin(phi)+(Bpz-vz)*sin(theta)*cos(phi);
      yaw = atan2(yaw_num,yaw_den)*180.0/pi;
      output_signals{i}.value = yaw;
      %output_signals{i}.value = atan2(xmag,ymag)*180.0/pi;
      output_signals{i}.value = wrapangle(output_signals{i}.value);
      output_signals{i}.timestamp = max(xmag_timestamp,ymag_timestamp);
      output_signals{i}.status = zmag_status;
      phi_computed = 1;
      theta_computed = 1;
    end
  end
  if(input_signals{i}.sensorindex ~= current_sensor_index)
    xmag = 0;
    ymag = 0;
    zmag = 0;
    xmag_found = 0;
    ymag_found = 0;
    zmag_found = 0;
    xacc = 0;
    yacc = 0;
    zacc = 0;
    xacc_found = 0;
    yacc_found = 0;
    zacc_found = 0;
    phi = 0;
    theta = 0;
    phi_computed = 0;
    theta_computed = 0;
    current_sensor_index = input_signals{i}.sensorindex;
  end
  
  %% Range/Dynamics Checks
  if(strcmp(input_signals{i}.sensorsource,'110012') == 1) % Razor IMUSensorPostProcess
    %output_signals{i}.value = input_signals{i}.value;
    
  else
  end


end
