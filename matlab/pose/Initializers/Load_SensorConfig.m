global SensorConfig;
if(0)
  SensorConfig = [];
  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "acc_roll1";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_acc_roll1";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";

  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "acc_roll2";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_acc_roll2";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";

  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "acc_pitch1";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_acc_pitch1";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";

  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "acc_pitch2";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_acc_pitch2";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";


  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "mag_yaw1";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_mag_yaw1";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";

  SensorConfig{length(SensorConfig)+1}.input{1}.index = 1;
  SensorConfig{length(SensorConfig)}.input{1}.name = "mag_yaw2";
  SensorConfig{length(SensorConfig)}.operator{1}.index = 1;
  SensorConfig{length(SensorConfig)}.operator{1}.name = "diff";
  SensorConfig{length(SensorConfig)}.operator{1}.param_1 = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.index = 2;
  SensorConfig{length(SensorConfig)}.operator{2}.name = "scale";
  SensorConfig{length(SensorConfig)}.operator{2}.param_1 = 1.0;
  SensorConfig{length(SensorConfig)}.output{1}.name = "d_mag_yaw2";
  SensorConfig{length(SensorConfig)}.output{1}.units = "deg/s";
end

