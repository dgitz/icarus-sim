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
    valid_buffer;
    buffer_size;
    prev_value;
   end
   methods
    function obj = TimeCompensate(_signal_vector,_TIMINGCOMPENSATION_METHOD,_SignalState,_SignalClass,_name,_time_compensation_method)
        obj.signal_vector = _signal_vector;
        obj.TIMINGCOMPENSATION_METHOD = _TIMINGCOMPENSATION_METHOD;
        obj.SignalState = _SignalState;
        obj.SignalClass = _SignalClass;
        obj.debug=0;
        obj.buffer_size=10;
        obj.name = _name;
        obj.time_compensation_method = _time_compensation_method; 
        if((_time_compensation_method == obj.TIMINGCOMPENSATION_METHOD.SampleAndHold) ||
           (_time_compensation_method == obj.TIMINGCOMPENSATION_METHOD.LinearExtrapolate))
        else
          disp(['ERROR: Time Compensate Method Not Supported: '  num2str(_time_compensation_method)]); 
        end
        obj.valid_buffer.t = [];
        obj.valid_buffer.x = [];
        obj.current_index = 0;
        obj.prev_value.t=0;
        obj.prev_value.x=0;
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
      output.class = obj.SignalClass.SIGNALCLASS_TIMEDSIGNAL;
      if(index_updated == 1)
        output.tov = time;
        output.value = input.value;
        output.rms = input.rms;
        output.status = input.status;
        obj.valid_buffer.t(length(obj.valid_buffer.t)+1) = (input.tov-log_start_time);
        obj.valid_buffer.x(length(obj.valid_buffer.x)+1) = input.value;
        if(length(obj.valid_buffer.t)>obj.buffer_size)
          obj.valid_buffer.t(1) = [];
          obj.valid_buffer.x(1) = [];
        end
      else
        if(obj.time_compensation_method == obj.TIMINGCOMPENSATION_METHOD.SampleAndHold)
          output.status = obj.SignalState.SIGNALSTATE_HOLD;
          output.tov = time;
          output.value = input.value;
          output.rms = input.rms;
        elseif(obj.time_compensation_method == obj.TIMINGCOMPENSATION_METHOD.LinearExtrapolate)
          if(length(obj.valid_buffer.t) == 0)
            obj.valid_buffer.t(length(obj.valid_buffer.t)+1) = obj.prev_value.t;
            obj.valid_buffer.x(length(obj.valid_buffer.t)+1) = obj.prev_value.x;
            output.tov = input.tov-log_start_time;
            output.value = input.value;
            output.rms = input.rms;
            output.status = obj.SignalState.SIGNALSTATE_HOLD;
          elseif((length(obj.valid_buffer.t) == obj.buffer_size))
            t_mean = mean(obj.valid_buffer.t(end)-(obj.valid_buffer.t(end-1)));
            x_mean = mean(obj.valid_buffer.x(end)-(obj.valid_buffer.x(end-1)));
            m = x_mean/t_mean;
            output.tov = time;
            output.value = m*(time-obj.valid_buffer.t(end))+obj.valid_buffer.x(end);%B0+B1*output.tov;
            output.status = obj.SignalState.SIGNALSTATE_EXTRAPOLATED; 
            output.rms = input.rms;
          else
            
            output.tov = time;
            output.value = input.value;
            output.status = obj.SignalState.SIGNALSTATE_HOLD;
          end
        end
      end
      if(obj.debug == 1)
        disp(['Cur Time: ' num2str(time) ' Sig Index: ' num2str(obj.current_index) ' tov: ' num2str(output.tov) ' sz: ' num2str(length(obj.valid_buffer.t))]);
      end
      obj.output = output;
    end
   end
end

    