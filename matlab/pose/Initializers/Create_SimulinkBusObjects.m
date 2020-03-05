InputSignalObject = Simulink.Bus;
value = [];
value = Simulink.BusElement;
value.Name = 'value';
value.Dimensions = 1;
value.DimensionsMode = 'Fixed';
value.DataType = 'double';
value.SampleTime = -1;
value.Complexity = 'real';

status  = [];
status = Simulink.BusElement;
status.Name = 'status';
status.Dimensions = 1;
status.DimensionsMode = 'Fixed';
status.DataType = 'uint8';
status.SampleTime = -1;
status.Complexity = 'real';

rms = [];
rms = Simulink.BusElement;
rms.Name = 'rms';
rms.Dimensions = 1;
rms.DimensionsMode = 'Fixed';
rms.DataType = 'double';
rms.SampleTime = -1;
rms.Complexity = 'real';

sequence_number = [];
sequence_number = Simulink.BusElement;
sequence_number.Name = 'sequence_number';
sequence_number.Dimensions = 1;
sequence_number.DimensionsMode = 'Fixed';
sequence_number.DataType = 'uint32';
sequence_number.SampleTime = -1;
sequence_number.Complexity = 'real';

InputSignalObject.Elements = [value status rms sequence_number];
clear value status rms sequence_number;

OutputSignalObject = Simulink.Bus;
value = [];
value = Simulink.BusElement;
value.Name = 'value';
value.Dimensions = 1;
value.DimensionsMode = 'Fixed';
value.DataType = 'double';
value.SampleTime = -1;
value.Complexity = 'real';

status  = [];
status = Simulink.BusElement;
status.Name = 'status';
status.Dimensions = 1;
status.DimensionsMode = 'Fixed';
status.DataType = 'uint8';
status.SampleTime = -1;
status.Complexity = 'real';

rms = [];
rms = Simulink.BusElement;
rms.Name = 'rms';
rms.Dimensions = 1;
rms.DimensionsMode = 'Fixed';
rms.DataType = 'double';
rms.SampleTime = -1;
rms.Complexity = 'real';
OutputSignalObject.Elements = [value status rms];
clear value status rms

KalmanFilterObjectState = Simulink.Bus;
initialized = [];
initialized = Simulink.BusElement;
initialized.Name = 'initialized';
initialized.Dimensions = 1;
initialized.DimensionsMode = 'Fixed';
initialized.DataType = 'uint8';
initialized.SampleTime = -1;
initialized.Complexity = 'real';

update_counter = [];
update_counter = Simulink.BusElement;
update_counter.Name = 'update_counter';
update_counter.Dimensions = 1;
update_counter.DimensionsMode = 'Fixed';
update_counter.DataType = 'uint32';
update_counter.SampleTime = -1;
update_counter.Complexity = 'real';
KalmanFilterObjectState.Elements = [initialized update_counter];
clear initialized update_counter

TimeCompensatorObjectState = Simulink.Bus;
initialized = [];
initialized = Simulink.BusElement;
initialized.Name = 'initialized';
initialized.Dimensions = 1;
initialized.DimensionsMode = 'Fixed';
initialized.DataType = 'uint8';
initialized.SampleTime = -1;
initialized.Complexity = 'real';

update1 = [];
update1 = Simulink.BusElement;
update1.Name = 'signal1_update_counter';
update1.Dimensions = 1;
update1.DimensionsMode = 'Fixed';
update1.DataType = 'uint32';
update1.SampleTime = -1;
update1.Complexity = 'real';

update2 = [];
update2 = Simulink.BusElement;
update2.Name = 'signal2_update_counter';
update2.Dimensions = 1;
update2.DimensionsMode = 'Fixed';
update2.DataType = 'uint32';
update2.SampleTime = -1;
update2.Complexity = 'real';

update3 = [];
update3 = Simulink.BusElement;
update3.Name = 'signal3_update_counter';
update3.Dimensions = 1;
update3.DimensionsMode = 'Fixed';
update3.DataType = 'uint32';
update3.SampleTime = -1;
update3.Complexity = 'real';

update4 = [];
update4 = Simulink.BusElement;
update4.Name = 'signal4_update_counter';
update4.Dimensions = 1;
update4.DimensionsMode = 'Fixed';
update4.DataType = 'uint32';
update4.SampleTime = -1;
update4.Complexity = 'real';

update5 = [];
update5 = Simulink.BusElement;
update5.Name = 'signal5_update_counter';
update5.Dimensions = 1;
update5.DimensionsMode = 'Fixed';
update5.DataType = 'uint32';
update5.SampleTime = -1;
update5.Complexity = 'real';

update6 = [];
update6 = Simulink.BusElement;
update6.Name = 'signal6_update_counter';
update6.Dimensions = 1;
update6.DimensionsMode = 'Fixed';
update6.DataType = 'uint32';
update6.SampleTime = -1;
update6.Complexity = 'real';



TimeCompensatorObjectState.Elements = [initialized update1 update2 update3 update4 update5 update6];
clear initialized update1 update2 update3 update4 update5 update6