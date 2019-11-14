classdef LinearAccelerationLinker
   properties
    SignalClass;
    linear_accelerations;
   end
   methods
    function obj = LinearAccelerationLinker
    end
    function obj = init(obj,SignalClass)
      obj.SignalClass = SignalClass;
    end
    function obj = new_input(obj,linear_accelerations)
      for i = 1:length(linear_accelerations)
        linear_accelerations{i}.x.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
        linear_accelerations{i}.y.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
        linear_accelerations{i}.z.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
      end
      obj.linear_accelerations = linear_accelerations;
      
    end
  end
end

    