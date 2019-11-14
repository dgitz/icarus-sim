classdef SensorPostProcess
   properties
    output;
    SignalClass;
   end
   methods
    function obj = SensorPostProcess
    end
    function obj = init(obj,SignalClass)
      obj.SignalClass = SignalClass;
    end
    function obj = new_input(obj,signal)
      obj.output = signal;
      obj.output.class = obj.SignalClass.SIGNALCLASS_PROCESSEDSIGNAL;
    end
  end
end

    