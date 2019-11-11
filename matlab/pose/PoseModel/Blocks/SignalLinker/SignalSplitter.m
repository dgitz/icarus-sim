classdef SignalSplitter
   properties
    output;
    initialized;
    accelerometers = [];
    gyroscopes = [];
    magnetometers = [];
   end
   methods
    function obj = SignalSplitter()
      obj.initialized = 0;
    end
    function obj = init(obj,signal_vector)
      for i = 1:length(signal_vector)
        if((strfind(signal_vector(i).name,'xacc') > 0) || (strfind(signal_vector(i).name,'yacc')) || (strfind(signal_vector(i).name,'zacc')))
          instance_name = signal_vector(i).name(5:end);
          found_instance = 0;
          for j = 1:length(obj.accelerometers)
            if strcmp(obj.accelerometers{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.accelerometers{length(obj.accelerometers)+1}.instance_name = instance_name;
            obj.accelerometers{length(obj.accelerometers)}.x.name = ['xacc' instance_name]; 
            obj.accelerometers{length(obj.accelerometers)}.y.name = ['yacc' instance_name];
            obj.accelerometers{length(obj.accelerometers)}.z.name = ['zacc' instance_name];
          end
        end
        if((strfind(signal_vector(i).name,'xgyro') > 0) || (strfind(signal_vector(i).name,'ygyro')) || (strfind(signal_vector(i).name,'zgyro')))
          instance_name = signal_vector(i).name(6:end);
          found_instance = 0;
          for j = 1:length(obj.gyroscopes)
              if strcmp(obj.gyroscopes{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.gyroscopes{length(obj.gyroscopes)+1}.instance_name = instance_name;
            obj.gyroscopes{length(obj.gyroscopes)}.x.name = ['xgyro' instance_name]; 
            obj.gyroscopes{length(obj.gyroscopes)}.y.name = ['ygyro' instance_name];
            obj.gyroscopes{length(obj.gyroscopes)}.z.name = ['zgyro' instance_name];
          end
        end
        if((strfind(signal_vector(i).name,'xmag') > 0) || (strfind(signal_vector(i).name,'ymag')) || (strfind(signal_vector(i).name,'zmag')))
          instance_name = signal_vector(i).name(5:end);
          found_instance = 0;
          for j = 1:length(obj.magnetometers)
            if strcmp(obj.magnetometers{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.magnetometers{length(obj.magnetometers)+1}.instance_name = instance_name;
            obj.magnetometers{length(obj.magnetometers)}.x.name = ['xmag' instance_name]; 
            obj.magnetometers{length(obj.magnetometers)}.y.name = ['ymag' instance_name];
            obj.magnetometers{length(obj.magnetometers)}.z.name = ['zmag' instance_name];
          end
        end
      end
      obj.initialized=1;
    end
    function obj = new_input(obj,signal_vector)
      if(obj.initialized==0)
        obj = obj.init(signal_vector);
      else
      end
      obj = obj.update_input(signal_vector);
    end
    function obj = update_input(obj,signal_vector)
      for i = 1:length(signal_vector)
        for j = 1:length(obj.accelerometers)
          if(strcmp(signal_vector(i).name,obj.accelerometers{j}.x.name) == 1)
            obj.accelerometers{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.accelerometers{j}.y.name) == 1)
            obj.accelerometers{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.accelerometers{j}.z.name) == 1)
            obj.accelerometers{j}.z = signal_vector(i);
          end
        end
        for j = 1:length(obj.gyroscopes)
          if(strcmp(signal_vector(i).name,obj.gyroscopes{j}.x.name) == 1)
            obj.gyroscopes{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.gyroscopes{j}.y.name) == 1)
            obj.gyroscopes{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.gyroscopes{j}.z.name) == 1)
            obj.gyroscopes{j}.z = signal_vector(i);
          end
        end
        for j = 1:length(obj.magnetometers)
          if(strcmp(signal_vector(i).name,obj.magnetometers{j}.x.name) == 1)
            obj.magnetometers{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.magnetometers{j}.y.name) == 1)
            obj.magnetometers{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.magnetometers{j}.z.name) == 1)
            obj.magnetometers{j}.z = signal_vector(i);
          end
        end
        
      end
    end
  end
end

    