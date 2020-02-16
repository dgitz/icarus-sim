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
rms = [];
rms = Simulink.BusElement;
rms.Name = 'rms';
rms.Dimensions = 1;
rms.DimensionsMode = 'Fixed';
rms.DataType = 'double';
rms.SampleTime = -1;
rms.Complexity = 'real';
OutputSignalObject.Elements = [value status rms];

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



TimeCompensatorObjectState.Elements = [initialized update1 update2 update3];