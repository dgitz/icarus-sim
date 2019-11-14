classdef SignalSplitter
   properties
    output;
    initialized;
    linear_accelerations = [];
    rotation_rates = [];
    magnetic_fields = [];
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
          for j = 1:length(obj.linear_accelerations)
            if strcmp(obj.linear_accelerations{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.linear_accelerations{length(obj.linear_accelerations)+1}.instance_name = instance_name;
            obj.linear_accelerations{length(obj.linear_accelerations)}.x.name = ['xacc' instance_name]; 
            obj.linear_accelerations{length(obj.linear_accelerations)}.y.name = ['yacc' instance_name];
            obj.linear_accelerations{length(obj.linear_accelerations)}.z.name = ['zacc' instance_name];
          end
        end
        if((strfind(signal_vector(i).name,'xgyro') > 0) || (strfind(signal_vector(i).name,'ygyro')) || (strfind(signal_vector(i).name,'zgyro')))
          instance_name = signal_vector(i).name(6:end);
          found_instance = 0;
          for j = 1:length(obj.rotation_rates)
              if strcmp(obj.rotation_rates{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.rotation_rates{length(obj.rotation_rates)+1}.instance_name = instance_name;
            obj.rotation_rates{length(obj.rotation_rates)}.x.name = ['xgyro' instance_name]; 
            obj.rotation_rates{length(obj.rotation_rates)}.y.name = ['ygyro' instance_name];
            obj.rotation_rates{length(obj.rotation_rates)}.z.name = ['zgyro' instance_name];
          end
        end
        if((strfind(signal_vector(i).name,'xmag') > 0) || (strfind(signal_vector(i).name,'ymag')) || (strfind(signal_vector(i).name,'zmag')))
          instance_name = signal_vector(i).name(5:end);
          found_instance = 0;
          for j = 1:length(obj.magnetic_fields)
            if strcmp(obj.magnetic_fields{j}.instance_name,instance_name)==1
              found_instance=1;
            end
          end
          if(found_instance == 0)
            obj.magnetic_fields{length(obj.magnetic_fields)+1}.instance_name = instance_name;
            obj.magnetic_fields{length(obj.magnetic_fields)}.x.name = ['xmag' instance_name]; 
            obj.magnetic_fields{length(obj.magnetic_fields)}.y.name = ['ymag' instance_name];
            obj.magnetic_fields{length(obj.magnetic_fields)}.z.name = ['zmag' instance_name];
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
        for j = 1:length(obj.linear_accelerations)
          if(strcmp(signal_vector(i).name,obj.linear_accelerations{j}.x.name) == 1)
            obj.linear_accelerations{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.linear_accelerations{j}.y.name) == 1)
            obj.linear_accelerations{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.linear_accelerations{j}.z.name) == 1)
            obj.linear_accelerations{j}.z = signal_vector(i);
          end
        end
        for j = 1:length(obj.rotation_rates)
          if(strcmp(signal_vector(i).name,obj.rotation_rates{j}.x.name) == 1)
            obj.rotation_rates{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.rotation_rates{j}.y.name) == 1)
            obj.rotation_rates{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.rotation_rates{j}.z.name) == 1)
            obj.rotation_rates{j}.z = signal_vector(i);
          end
        end
        for j = 1:length(obj.magnetic_fields)
          if(strcmp(signal_vector(i).name,obj.magnetic_fields{j}.x.name) == 1)
            obj.magnetic_fields{j}.x = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.magnetic_fields{j}.y.name) == 1)
            obj.magnetic_fields{j}.y = signal_vector(i);
          elseif(strcmp(signal_vector(i).name,obj.magnetic_fields{j}.z.name) == 1)
            obj.magnetic_fields{j}.z = signal_vector(i);
          end
        end
        
      end
    end
  end
end

    