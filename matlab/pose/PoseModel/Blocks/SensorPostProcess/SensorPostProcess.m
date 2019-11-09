classdef SensorPostProcess
   properties
    output;
   end
   methods
    function obj = SensorPostProcess()
    end
    function obj = new_input(obj,signal)
      obj.output = signal;
    end
  end
end

    