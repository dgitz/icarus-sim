classdef TimeCompensateConfig
    properties (Constant)
        POSE_UPDATE_RATE = 20.0;
        QUICK_ANALYZE_RATE = 1.0;
        SAMPLING_MODE = SamplingMethod.SAMPLEMETHOD_SAMPLEHOLD; %1==Sample and Hold 2 == Linear Extrapolate
        BUFFER_SIZE = 8;
        TOVDELTA_ERROR = 0.3;
    end
end