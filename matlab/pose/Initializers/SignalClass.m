classdef SignalClass < uint8
    enumeration
        SIGCLASS_UNDEFINED (0)
        SIGCLASS_SENSORSIGNAL (1)
        SIGCLASS_TIMEDSIGNAL (2)
        SIGCLASS_PROCESSEDSIGNAL (3)
        SIGCLASS_INPUTSIGNAL (4)
        SIGCLASS_POSESIGNAL (5)
    end
end