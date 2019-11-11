classdef MasterLinker
   properties
    splitter;
   end
   methods
    function obj = MasterLinker()
      obj.splitter = SignalSplitter;
    end
    function obj = new_input(obj,signal_vector)
      obj.splitter = obj.splitter.new_input(signal_vector);
    end
  end
end

    