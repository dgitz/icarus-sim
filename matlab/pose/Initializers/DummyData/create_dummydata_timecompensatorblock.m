## Author: David Gitz <robot@dgitzdev>
## Created: 2019-11-05

function [signal_vector,time_compensation_method_enum,signalstate_enum,name,time_compensation_method] = create_dummydata_timecompensatorblock ()
global TIMINGCOMPENSATION_METHOD;
global SignalState;
global SignalType;
time_compensation_method_enum = TIMINGCOMPENSATION_METHOD;
signalstate_enum = SignalState;
time_compensation_method = TIMINGCOMPENSATION_METHOD.SampleAndHold;
sig = Initialize_Signal;
sig.name = 'DummyData';
name = sig.name;
sig.tov = 0.0;
sig.type = SignalType.SIGNALTYPE_ACCELERATION;
sig.value = 0.0;
sig.status = SignalState.SIGNALSTATE_UPDATED;
signal_vector = [];
for i = 1:100
  sig.tov = i/10;
  sig.value = i*rand;
  signal_vector = [signal_vector sig];
end
endfunction
