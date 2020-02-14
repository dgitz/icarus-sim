classdef BaseMachineBlock
  properties
    SignalClass;
    base_machine;
  end
  methods
    function obj = BaseMachineBlock
    end
    function obj = init(obj,SignalClass)
      obj.SignalClass = SignalClass;
    end
    function obj = new_input(obj,linear_acceleration,wheel_odometry)
      obj.base_machine = base_machine;
      
    end
  end
end