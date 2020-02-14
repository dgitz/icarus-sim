classdef BaseMachineLinker
   properties
    SignalClass;
    SignalType;
    SignalState;
    Pipeline_ComputationMethod = [];
    initialized;
   end
   methods
    function obj = BaseMachineLinker
      obj.initialized = 0;
      
    end
    function obj = init(obj,SignalClass,SignalType,SignalState)
      obj.SignalClass = SignalClass;
      obj.SignalType = SignalType;
      obj.SignalState = SignalState;
    end
    function obj = new_input(obj)
      if(obj.initialized == 0)
        pipeline_index = 1;
        obj.initialized = 1;
      end   
    end
  end
end

    