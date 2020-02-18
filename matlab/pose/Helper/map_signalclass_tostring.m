function [str] = map_signalclass_tostring(class)
str = 'UNKNOWN';
if(class == SignalClass.SIGNALCLASS_UNDEFINED_)
  str = 'UNKNOWN';
elseif(class == SignalClass.SIGNALCLASS_SENSORSIGNAL_)
  str = 'SENSOR';
elseif(class == SignalClass.SIGNALCLASS_TIMEDSIGNAL_)
  str = 'TIMED';
elseif(class == SignalClass.SIGNALCLASS_PROCESSEDSIGNAL_)
  str = 'PROCESSED';
elseif(class == SignalClass.SIGNALCLASS_INPUTSIGNAL_)
  str = 'INPUT';
elseif(class == SignalClass.SIGNALCLASS_POSESIGNAL_)
  str = 'POSE';
elseif(class == SignalClass.SIGNALCLASS_TRUTHSIGNAL_)
  str = 'TRUTH';
end
end