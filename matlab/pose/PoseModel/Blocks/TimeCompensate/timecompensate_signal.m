function [value,status,rms,update_count,buffers_x,buffers_t] = timecompensate_signal(current_time,update_count_in,index,value_in,status_in,rms_in,buffers_x_in,buffers_t_in)
update_count = update_count_in;
buffers_x = buffers_x_in;
buffers_t = buffers_t_in;
if(status_in == SignalState.SIGSTATE_UPDATED)
    value = value_in;
    status = status_in;
    rms = rms_in;
    if(update_count_in <= uint32(TimeCompensateConfig.BUFFER_SIZE))
        buffers_x(index,update_count_in) = value_in;
        buffers_t(index,update_count_in) = current_time;
    else
        row_x = buffers_x(index,:);
        row_x = [row_x(2:TimeCompensateConfig.BUFFER_SIZE) value_in];
        buffers_x(index,:) = row_x;
        row_t = buffers_t(index,:);
        row_t = [row_t(2:TimeCompensateConfig.BUFFER_SIZE) current_time];
        buffers_t(index,:) = row_t;
        
    end
    update_count = update_count_in + uint32(1);
else
    if(TimeCompensateConfig.SAMPLING_MODE == SamplingMethod.SAMPLEMETHOD_SAMPLEHOLD)
        value = value_in;
        status = uint8(SignalState.SIGSTATE_HOLD);
        rms = rms_in;
    elseif(TimeCompensateConfig.SAMPLING_MODE == SamplingMethod.SAMPLEMETHOD_LINEAREXTRAPOLATE)
        if(update_count_in <= uint32(TimeCompensateConfig.BUFFER_SIZE))
            value = value_in;
            status = uint8(SignalState.SIGSTATE_HOLD);
            rms = rms_in;
        else
            dt = diff(buffers_t(index,TimeCompensateConfig.BUFFER_SIZE-1:TimeCompensateConfig.BUFFER_SIZE));
            dv = diff(buffers_x(index,TimeCompensateConfig.BUFFER_SIZE-1:TimeCompensateConfig.BUFFER_SIZE));
            m = dv/dt;
            value = m*(current_time-buffers_t(index,TimeCompensateConfig.BUFFER_SIZE))+buffers_x(index,TimeCompensateConfig.BUFFER_SIZE);
            status = uint8(SignalState.SIGSTATE_EXTRAPOLATED);
            rms = rms_in;
        end
    else
        value = value_in;
        status = status_in;
        rms = rms_in;
    end
end