function [str] = map_signalclass_tostring(class)
str = 'UNKNOWN';
if(class == SignalClass.SIGCLASS_UNDEFINED)
  str = 'UNKNOWN';
elseif(class == SignalClass.SIGCLASS_SENSORSIGNAL)
  str = 'SENSOR';
elseif(class == SignalClass.SIGCLASS_TIMEDSIGNAL)
  str = 'TIMED';
elseif(class == SignalClass.SIGCLASS_PROCESSEDSIGNAL)
  str = 'PROCESSED';
elseif(class == SignalClass.SIGCLASS_INPUTSIGNAL)
  str = 'INPUT';
elseif(class == SignalClass.SIGCLASS_POSESIGNAL)
  str = 'POSE';
end
end