function [signal] = Initialize_SensorSignal()
global SignalClass
signal = Initialize_Signal;
signal.class = SignalClass.SIGNALCLASS_SENSORSIGNAL;
end
