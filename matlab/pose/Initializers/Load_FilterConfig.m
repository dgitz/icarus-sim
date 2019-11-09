global FilterConfig;
FilterConfig = [];
if(0)

  FilterConfig.LinearAcceleration.KF_X.InputCount = 2;
  FilterConfig.LinearAcceleration.KF_X.A = 1;
  FilterConfig.LinearAcceleration.KF_X.R = zeros(FilterConfig.LinearAcceleration.KF_X.InputCount,FilterConfig.LinearAcceleration.KF_X.InputCount);
  for i = 1:FilterConfig.LinearAcceleration.KF_X.InputCount
    FilterConfig.LinearAcceleration.KF_X.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_X.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_X.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_X.R(i,i) = 1;
  end
  FilterConfig.LinearAcceleration.KF_X.Q = .05;
  FilterConfig.LinearAcceleration.KF_X.C = zeros(FilterConfig.LinearAcceleration.KF_X.InputCount,1);

  FilterConfig.LinearAcceleration.KF_Y.InputCount = 2;
  FilterConfig.LinearAcceleration.KF_Y.A = 1;
  FilterConfig.LinearAcceleration.KF_Y.R = zeros(FilterConfig.LinearAcceleration.KF_Y.InputCount,FilterConfig.LinearAcceleration.KF_Y.InputCount);
  for i = 1:FilterConfig.LinearAcceleration.KF_Y.InputCount
    FilterConfig.LinearAcceleration.KF_Y.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Y.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Y.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Y.R(i,i) = 1;
  end
  FilterConfig.LinearAcceleration.KF_Y.Q = .05;
  FilterConfig.LinearAcceleration.KF_Y.C = zeros(FilterConfig.LinearAcceleration.KF_Y.InputCount,1);

  FilterConfig.LinearAcceleration.KF_Z.InputCount = 2;
  FilterConfig.LinearAcceleration.KF_Z.A = 1;
  FilterConfig.LinearAcceleration.KF_Z.R = zeros(FilterConfig.LinearAcceleration.KF_Z.InputCount,FilterConfig.LinearAcceleration.KF_Z.InputCount);
  for i = 1:FilterConfig.LinearAcceleration.KF_Z.InputCount
    FilterConfig.LinearAcceleration.KF_Z.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Z.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Z.R(i,i) = 1;
    FilterConfig.LinearAcceleration.KF_Z.R(i,i) = 1;
  end
  FilterConfig.LinearAcceleration.KF_Z.Q = .05;
  FilterConfig.LinearAcceleration.KF_Z.C = zeros(FilterConfig.LinearAcceleration.KF_Z.InputCount,1);



  FilterConfig.AngleRate.KF_X.InputCount = 4;
  FilterConfig.AngleRate.KF_X.A = 1;
  FilterConfig.AngleRate.KF_X.R = zeros(FilterConfig.AngleRate.KF_X.InputCount ,FilterConfig.AngleRate.KF_X.InputCount );
  for i = 1:FilterConfig.AngleRate.KF_X.InputCount 
    FilterConfig.AngleRate.KF_X.R(i,i) = 1;
    FilterConfig.AngleRate.KF_X.R(i,i) = 1;
    FilterConfig.AngleRate.KF_X.R(i,i) = 1;
    FilterConfig.AngleRate.KF_X.R(i,i) = 1;
  end
  FilterConfig.AngleRate.KF_X.Q = .05;
  FilterConfig.AngleRate.KF_X.C = zeros(FilterConfig.AngleRate.KF_X.InputCount ,1);

  FilterConfig.AngleRate.KF_Y.InputCount = 4;
  FilterConfig.AngleRate.KF_Y.A = 1;
  FilterConfig.AngleRate.KF_Y.R = zeros(FilterConfig.AngleRate.KF_Y.InputCount,FilterConfig.AngleRate.KF_X.InputCount);
  for i = 1:FilterConfig.AngleRate.KF_X.InputCount 
    FilterConfig.AngleRate.KF_Y.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Y.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Y.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Y.R(i,i) = 1;
  end
  FilterConfig.AngleRate.KF_Y.Q = .05;
  FilterConfig.AngleRate.KF_Y.C = zeros(FilterConfig.AngleRate.KF_Y.InputCount,1);

  FilterConfig.AngleRate.KF_Z.InputCount = 4;
  FilterConfig.AngleRate.KF_Z.A = 1;
  FilterConfig.AngleRate.KF_Z.R = zeros(FilterConfig.AngleRate.KF_Z.InputCount,FilterConfig.AngleRate.KF_Z.InputCount);
  for i = 1:FilterConfig.AngleRate.KF_Z.InputCount
    FilterConfig.AngleRate.KF_Z.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Z.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Z.R(i,i) = 1;
    FilterConfig.AngleRate.KF_Z.R(i,i) = 1;
  end
  FilterConfig.AngleRate.KF_Z.Q = .05;
  FilterConfig.AngleRate.KF_Z.C = zeros(FilterConfig.AngleRate.KF_Z.InputCount,1);

  global SensorFilterLink;
  SensorFilterLink = [];
  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_roll1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_roll1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_roll2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_roll2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "xgyro1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "xgyro1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "xgyro2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "xgyro2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_pitch1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_pitch1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_pitch2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_pitch2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "ygyro1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "ygyro1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "ygyro2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "ygyro2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_yaw1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_yaw1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "mag_yaw2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "diff";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.index = 2;
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{2}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "d_mag_yaw2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "zgyro1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "zgyro1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "zgyro2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "zgyro2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "deg/s";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Angle Rate";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "xacc1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "xacc1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "xacc2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "xacc2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "x";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "yacc1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "yacc1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "yacc2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "yacc2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "y";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "zacc1";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "zacc1";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";

  SensorFilterLink{length(SensorFilterLink)+1}.input{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.input{1}.name = "zacc2";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.index = 1;
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.name = "scale";
  SensorFilterLink{length(SensorFilterLink)}.operator{1}.param_1 = 1.0;
  SensorFilterLink{length(SensorFilterLink)}.output{1}.name = "zacc2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.units = "m/s^2";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.filter = "Linear Acceleration";
  SensorFilterLink{length(SensorFilterLink)}.output{1}.type = "z";
end
