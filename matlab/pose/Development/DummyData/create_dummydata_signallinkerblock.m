## Author: David Gitz <robot@dgitzdev>
## Created: 2019-11-05

function [signal_vectors] = create_dummydata_signallinkerblock ()
global TIMINGCOMPENSATION_METHOD;
global SignalState;
global SignalType;
global SignalClass;
global Config;
sig_xacc1 = Initialize_Signal;
sig_yacc1 = Initialize_Signal;
sig_zacc1 = Initialize_Signal;
sig_xgyro1 = Initialize_Signal;
sig_ygyro1 = Initialize_Signal;
sig_zgyro1 = Initialize_Signal;
sig_xmag1 = Initialize_Signal;
sig_ymag1 = Initialize_Signal;
sig_zmag1 = Initialize_Signal;
sig_xacc1.name = 'xacc1';
sig_yacc1.name = 'yacc1';
sig_zacc1.name = 'zacc1';
sig_xgyro1.name = 'xgyro1';
sig_ygyro1.name = 'ygyro1';
sig_zgyro1.name = 'zgyro1';
sig_xmag1.name = 'xmag1';
sig_ymag1.name = 'ymag1';
sig_zmag1.name = 'zmag1';
sig_xacc1.tov = 0.0;
sig_yacc1.tov = 0.0;
sig_zacc1.tov = 0.0;
sig_xgyro1.tov = 0.0;
sig_ygyro1.tov = 0.0;
sig_zgyro1.tov = 0.0;
sig_xmag1.tov = 0.0;
sig_ymag1.tov = 0.0;
sig_zmag1.tov = 0.0;
sig_xacc1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_yacc1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_zacc1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_xgyro1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_ygyro1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_zgyro1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_xmag1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_ymag1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_zmag1.class = SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
sig_xacc1.type = SignalType.SIGNALTYPE_ACCELERATION;
sig_yacc1.type = SignalType.SIGNALTYPE_ACCELERATION;
sig_zacc1.type = SignalType.SIGNALTYPE_ACCELERATION;
sig_xgyro1.type = SignalType.SIGNALTYPE_ROTATION_RATE;
sig_ygyro1.type = SignalType.SIGNALTYPE_ROTATION_RATE;
sig_zgyro1.type = SignalType.SIGNALTYPE_ROTATION_RATE;
sig_xmag1.type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
sig_ymag1.type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
sig_zmag1.type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
sig_xacc1.value = 0.0;
sig_yacc1.value = 0.0;
sig_zacc1.value = 9.81;
sig_xgyro1.value = 0.0;
sig_ygyro1.value = 0.0;
sig_zgyro1.value = 0.0;
sig_xmag1.value = 0.1;
sig_ymag1.value = 0.2;
sig_zmag1.value = 0.3;
sig_xacc1.status = SignalState.SIGNALSTATE_UPDATED;
sig_yacc1.status = SignalState.SIGNALSTATE_UPDATED;
sig_zacc1.status = SignalState.SIGNALSTATE_UPDATED;
sig_xgyro1.status = SignalState.SIGNALSTATE_UPDATED;
sig_ygyro1.status = SignalState.SIGNALSTATE_UPDATED;
sig_zgyro1.status = SignalState.SIGNALSTATE_UPDATED;
sig_xmag1.status = SignalState.SIGNALSTATE_UPDATED;
sig_ymag1.status = SignalState.SIGNALSTATE_UPDATED;
sig_zmag1.status = SignalState.SIGNALSTATE_UPDATED;
signal_vectors = [];
counter = 0;
signal_time = 75.0;
dt = 1;
elap_time = 0;
i = 0;
xacc1_vector = [];
yacc1_vector = [];
zacc1_vector = [];
xgyro1_vector = [];
ygyro1_vector = [];
zgyro1_vector = [];
xmag1_vector = [];
ymag1_vector = [];
zmag1_vector = [];
while(elap_time < signal_time)
  sig_xacc1.tov = elap_time;
  sig_yacc1.tov = elap_time;
  sig_zacc1.tov = elap_time;
  sig_xgyro1.tov = elap_time;
  sig_ygyro1.tov = elap_time;
  sig_zgyro1.tov = elap_time;
  sig_xmag1.tov = elap_time;
  sig_ymag1.tov = elap_time;
  sig_zmag1.tov = elap_time;
  %sig.value = counter;
  sig_xacc1.value = sin(elap_time/4);
  sig_yacc1.value = cos(elap_time/4);
  sig_zacc1.value = 9.81*sin(elap_time/4);
  sig_xgyro1.value = counter;
  sig_ygyro1.value = counter;
  sig_zgyro1.value = counter;
  sig_xmag1.value = (counter/100)*sig_xacc1.value;
  sig_ymag1.value = (counter/100)*sig_yacc1.value;
  sig_zmag1.value = (counter/100)*sig_zacc1.value;
  xacc1_vector = [xacc1_vector sig_xacc1];
  yacc1_vector = [yacc1_vector sig_yacc1];
  zacc1_vector = [zacc1_vector sig_zacc1];
  xgyro1_vector = [xgyro1_vector sig_xgyro1];
  ygyro1_vector = [ygyro1_vector sig_ygyro1];
  zgyro1_vector = [zgyro1_vector sig_zgyro1];
  xmag1_vector = [xmag1_vector sig_xmag1];
  ymag1_vector = [ymag1_vector sig_ymag1];
  zmag1_vector = [zmag1_vector sig_zmag1];
  counter = counter+1;
  if(counter > 10)
    counter = 0;
  end
  elap_time = elap_time+dt+rand/100;
end
signal_vectors{1} = xacc1_vector;
signal_vectors{2} = yacc1_vector;
signal_vectors{3} = zacc1_vector;
signal_vectors{4} = xgyro1_vector;
signal_vectors{5} = ygyro1_vector;
signal_vectors{6} = zgyro1_vector;
signal_vectors{7} = xmag1_vector;
signal_vectors{8} = ymag1_vector;
signal_vectors{9} = zmag1_vector;
endfunction
