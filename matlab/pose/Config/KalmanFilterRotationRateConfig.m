classdef KalmanFilterRotationRateConfig
    properties (Constant)
        XRotationRate_R = 0.64;
        XRotationRate_Q = 0.05;
        XRotationRate_C = 1;
        
        YRotationRate_R = 0.64;
        YRotationRate_Q = 0.05;
        YRotationRate_C = 1;
        
        ZRotationRate_R = 0.64;
        ZRotationRate_Q = 0.05;
        ZRotationRate_C = 1;
    end
end