classdef MasterLinker
   properties
    SignalClass;
    SignalType;
    SignalState;
    splitter;
    linearacceleration_linker;
    rotationrate_linker;
    orientation_linker;
    basemachine_linker;
    linearacceleration_signals = [];
    rotationrate_signals = [];
    orientation_signals = [];
    basemachine_signals = [];
   end
   methods
    function obj = MasterLinker()
    end
    function obj = init(obj,SignalClass,SignalType,SignalState)
      obj.SignalClass = SignalClass;
      obj.splitter = SignalSplitter;
      obj.linearacceleration_linker = LinearAccelerationLinker;
      obj.linearacceleration_linker = obj.linearacceleration_linker.init(SignalClass);
      obj.rotationrate_linker = RotationRateLinker;
      obj.rotationrate_linker = obj.rotationrate_linker.init(SignalClass);
      obj.orientation_linker = OrientationLinker;
      obj.orientation_linker = obj.orientation_linker.init(SignalClass,SignalType,SignalState);
      %obj.basemachine_linker = BaseMachineLinker;
      %obj.basemachine_linker = obj.basemachine_linker.init(SignalClass,SignalType,SignalState);
    end
    function obj = new_input(obj,signal_vector)
      obj.splitter = obj.splitter.new_input(signal_vector);
      obj.linearacceleration_linker = obj.linearacceleration_linker.new_input(obj.splitter.linear_accelerations);
      obj.rotationrate_linker = obj.rotationrate_linker.new_input(obj.splitter.rotation_rates);
      obj.orientation_linker = obj.orientation_linker.new_input([0 0 0],
        [obj.splitter.linear_accelerations,
        obj.splitter.rotation_rates,
        obj.splitter.magnetic_fields]);
      %obj.basemachine_linker = obj.basemachine_linker.new_input(obj.splitter.linear_accelerations);
    end
  end
end

    