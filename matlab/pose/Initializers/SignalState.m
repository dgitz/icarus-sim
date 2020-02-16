classdef SignalState < uint8
    enumeration
        SIGSTATE_UNDEFINED (0)
        SIGSTATE_INVALID (1)
        SIGSTATE_INITIALIZING (2)
        SIGSTATE_UPDATED (3)
        SIGSTATE_EXTRAPOLATED (4)
        SIGSTATE_HOLD (5)
        SIGSTATE_CALIBRATING (6)
    end
end