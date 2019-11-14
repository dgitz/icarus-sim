classdef RotationRateLinker
   properties
    SignalClass;
    rotation_rates;
   end
   methods
    function obj = RotationRateLinker
    end
    function obj = init(obj,SignalClass)
      obj.SignalClass = SignalClass;
    end
    function obj = new_input(obj,rotation_rates)
      for i = 1:length(rotation_rates)
        rotation_rates{i}.x.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
        rotation_rates{i}.y.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
        rotation_rates{i}.z.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
      end
      obj.rotation_rates = rotation_rates;
      
    end
  end
end

    