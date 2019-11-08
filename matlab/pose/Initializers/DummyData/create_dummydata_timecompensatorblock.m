## Author: David Gitz <robot@dgitzdev>
## Created: 2019-11-05

function [signal_vector,name] = create_dummydata_timecompensatorblock ()
global TIMINGCOMPENSATION_METHOD;
global SignalState;
global SignalType;
global Config;
sig = Initialize_Signal;
sig.name = 'DummyData';
name = sig.name;
sig.tov = 0.0;
sig.type = SignalType.SIGNALTYPE_ACCELERATION;
sig.value = 0.0;
sig.status = SignalState.SIGNALSTATE_UPDATED;
signal_vector = [];
counter = 0;
signal_time = 100.0;
dt = 1;
elap_time = 0;
i = 0;
while(elap_time < signal_time)

  sig.tov = elap_time;
  %sig.value = counter;
  sig.value = sin(elap_time/4);
  signal_vector = [signal_vector sig];
  counter = counter+1;
  if(counter > 10)
    counter = 0;
  end
  elap_time = elap_time+dt+rand/100;
end
endfunction
