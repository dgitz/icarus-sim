function [signal] = Initialize_Signal(signal_type)
global SignalState;
global SignalType;
count = 1;
time = [0];
signal.time = time;
type = signal_type*ones(count,1)';
value = 0*ones(count,1)';
status = SignalState.SIGNALSTATE_UNDEFINED*ones(count,1)';
rms = 0*ones(count,1)';
sequence_number = 0*ones(count,1)';
signal.signals.values = [type;value;status;rms;sequence_number;]';
signal.signals.dimensions = 5;
signal.available = 0;
end