function [signal] = Initialize_Signal()
global SignalState;
global SignalType;
signal.name = '';
signal.tov = -1.0;
signal.type = SignalType.SIGNALTYPE_UNDEFINED;
signal.value = 0.0;
signal.status = SignalState.SIGNALSTATE_UNDEFINED;
signal.rms = 0.0;
end