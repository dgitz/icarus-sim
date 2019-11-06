classdef TimeCompensate
   properties
    signal_vector; % Requires complete input vector to index properly.  This is only passed in once at start, 
                   % and then during execution this resembles the C++ code from an interface level and most of execution
    TIMINGCOMPENSATION_METHOD;
    SignalState;
    SignalClass;
    name;
    time_compensation_method; 
    output;
    current_index;
    debug=0;
   end
   methods
    function obj = TimeCompensate(_signal_vector,_TIMINGCOMPENSATION_METHOD,_SignalState,_SignalClass,_name,_time_compensation_method)
        obj.signal_vector = _signal_vector;
        obj.TIMINGCOMPENSATION_METHOD = _TIMINGCOMPENSATION_METHOD;
        obj.SignalState = _SignalState;
        obj.SignalClass = _SignalClass;
        obj.debug=0;
        obj.name = _name;
        obj.time_compensation_method = _time_compensation_method; 
        if(_time_compensation_method == obj.TIMINGCOMPENSATION_METHOD.SampleAndHold)
        else
          disp(['ERROR: Time Compensate Method Not Supported: '  _time_compensation_method]); 
        end
        obj.current_index = 0;
    end
    function obj = new_input(obj,log_start_time,time)
      index_updated = 0;
      if(obj.current_index == 0)
        index_updated = 1;
        obj.current_index=obj.current_index+1;
      end
      time_found = 0;
      if((log_start_time+time) > obj.signal_vector(obj.current_index).tov)
        new_index = obj.current_index;
        while((log_start_time+time) > obj.signal_vector(new_index).tov)
          new_index = new_index+1;
          if(new_index > length(obj.signal_vector))
            new_index = new_index-1;
            break;
          end
        end
        if(new_index <= length(obj.signal_vector))
          if(obj.current_index ~= new_index)
            index_updated = 1;
          end
          obj.current_index = new_index;
        end
      end
      input = obj.signal_vector(obj.current_index);
      
      output = input;
      if(index_updated == 1)
        output.tov = time;
        output.class = obj.SignalClass.SIGNALCLASS_TIMEDSIGNAL;
        output.value = input.value;
        output.rms = input.rms;
        output.status = input.status;
      else
        output.tov = time;
        output.class = obj.SignalClass.SIGNALCLASS_TIMEDSIGNAL;
        output.value = input.value;
        output.rms = input.rms;
        output.status = obj.SignalState.SIGNALSTATE_HOLD;
      end
      if(obj.debug == 1)
        disp(['Cur Time: ' num2str(time) ' Sig Index: ' num2str(obj.current_index) ' tov: ' num2str(output.tov)]);
      end
      obj.output = output;
    end
   end
end

    