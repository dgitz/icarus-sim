function [str] = map_signalclass_tostring(class)
global SignalClass
str = 'UNKNOWN';
if(class == SignalClass.SIGNALCLASS_UNDEFINED)
  str = 'UNKNOWN';
elseif(class == SignalClass.SIGNALCLASS_SENSORSIGNAL)
  str = 'SENSOR';
elseif(class == SignalClass.SIGNALCLASS_TIMEDSIGNAL)
  str = 'TIMED';
elseif(class == SignalClass.SIGNALCLASS_PROCESSEDSIGNAL)
  str = 'PROCESSED';
elseif(class == SignalClass.SIGNALCLASS_INPUTSIGNAL)
  str = 'INPUT';
elseif(class == SignalClass.SIGNALCLASS_POSESIGNAL)
  str = 'POSE';
end
end