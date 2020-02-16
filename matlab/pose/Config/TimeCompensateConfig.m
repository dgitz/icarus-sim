classdef TimeCompensateConfig
    properties (Constant)
        POSE_UPDATE_RATE = 20.0;
        SAMPLING_MODE = SamplingMethod.SAMPLEMETHOD_SAMPLEHOLD; %1==Sample and Hold 2 == Linear Extrapolate
        BUFFER_SIZE = 8;
    end
end