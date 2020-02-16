function [sig] = convert_inputsignal_sensorsignal(name,signal)
VALUE_INDEX = 1;
STATUS_INDEX = 2;
RMS_INDEX = 3;
SEQUENCENUMBER_INDEX = 4;
sig.class = SignalClass.SIGCLASS_SENSORSIGNAL;
if(strfind(name,'accel') == 1)
    seperator = strfind(name,'_');
    index = str2num(name(6:seperator-2));
    axis = name(seperator-1);
    if(axis == 'x')
        sig.name = ['xacc' num2str(index)];
    elseif(axis == 'y')
         sig.name = ['yacc' num2str(index)];
    elseif(axis == 'z')
         sig.name = ['zacc' num2str(index)];
    end
elseif(strfind(name,'rotationrate') == 1)
    seperator = strfind(name,'_');
    index = str2num(name(13:seperator-2));
    axis = name(seperator-1);
    if(axis == 'x')
        sig.name = ['xgyro' num2str(index)];
    elseif(axis == 'y')
         sig.name = ['ygyro' num2str(index)];
    elseif(axis == 'z')
         sig.name = ['zgyro' num2str(index)];
    end
elseif(strfind(name,'mag') == 1)
    seperator = strfind(name,'_');
    index = str2num(name(4:seperator-2));
    axis = name(seperator-1);
    if(axis == 'x')
        sig.name = ['xmag' num2str(index)];
    elseif(axis == 'y')
         sig.name = ['ymag' num2str(index)];
    elseif(axis == 'z')
         sig.name = ['zmag' num2str(index)];
    end
end
sig.tov = signal.time;
sig.value = signal.signals.values(:,VALUE_INDEX);
sig.status = signal.signals.values(:,STATUS_INDEX);
sig.rms = signal.signals.values(:,RMS_INDEX);
sig.sequence_number = signal.signals.values(:,SEQUENCENUMBER_INDEX);
        
%sig = [];
            %sig.class = SignalClass.SIGCLASS_SENSORSIGNAL;
            %sig.name = 'xacc1';
            %sig.tov = accel1x_in.time;
            %sig.type = accel1x_in.signals.values(:,TYPE_INDEX);
            %sig.value = accel1x_in.signals.values(:,VALUE_INDEX);
            %sig.status = accel1x_in.signals.values(:,STATUS_INDEX);
            %sig.rms = accel1x_in.signals.values(:,RMS_INDEX);
            %sig.sequence_number = accel1x_in.signals.values(:,SEQUENCENUMBER_INDEX);
end

