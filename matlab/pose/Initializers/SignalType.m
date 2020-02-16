classdef SignalType < uint8
    enumeration
        SIGTYPE_UNDEFINED (0)
        SIGTYPE_ACCELERATION (1)
        SIGTYPE_ROTATION_RATE (2)
        SIGTYPE_MAGNETIC_FIELD (3)
        SIGTYPE_TEMPERATURE (4)
        SIGTYPE_DISTANCE (5)
        SIGTYPE_ANGLE (6)
        SIGTYPE_VELOCITY (7)
        SIGTYPE_TICKSPEED (8)
        SIGTYPE_UNITLESS (255)
    end
end