close all;
fig_list = [];

Plot_Jerk = 0;
Plot_Acceleration = 1;
Plot_AngularRate = 1;
Plot_MagneticField = 1;
Plot_Orientation = 0;

Plot_SensorSignals = 1;
Plot_TimedSignals = 1;
Plot_ProcessedSignals = 0;
Plot_InputSignals = 0;
Plot_TruthSignals = 1;

linear_acc_signals = [];
rotation_rate_signals = [];
magnetic_field_signals = [];
orientation_signals = [];
if(Plot_SensorSignals == 1)
    if(Plot_Acceleration == 1)
        if(accel1x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel1x_in);
            sig = convert_inputsignal_sensorsignal(name,accel1x_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel1y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel1y_in);
            sig = convert_inputsignal_sensorsignal(name,accel1y_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel1z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel1z_in);
            sig = convert_inputsignal_sensorsignal(name,accel1z_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel2x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel2x_in);
            sig = convert_inputsignal_sensorsignal(name,accel2x_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel2y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel2y_in);
            sig = convert_inputsignal_sensorsignal(name,accel2y_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel2z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel2z_in);
            sig = convert_inputsignal_sensorsignal(name,accel2z_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel3x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel3x_in);
            sig = convert_inputsignal_sensorsignal(name,accel3x_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel3y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel3y_in);
            sig = convert_inputsignal_sensorsignal(name,accel3y_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel3z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel3z_in);
            sig = convert_inputsignal_sensorsignal(name,accel3z_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel4x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel4x_in);
            sig = convert_inputsignal_sensorsignal(name,accel4x_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel4y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel4y_in);
            sig = convert_inputsignal_sensorsignal(name,accel4y_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
        if(accel4z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(accel4z_in);
            sig = convert_inputsignal_sensorsignal(name,accel4z_in);
            linear_acc_signals{length(linear_acc_signals)+1} = sig;
        end
    end
    if(Plot_AngularRate == 1)
        if(rotationrate1x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate1x_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate1x_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate1y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate1y_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate1y_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate1z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate1z_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate1z_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate2x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate2x_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate2x_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate2y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate2y_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate2y_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate2z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate2z_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate2z_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate3x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate3x_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate3x_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate3y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate3y_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate3y_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate3z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate3z_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate3z_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate4x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate4x_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate4x_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate4y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate4y_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate4y_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
        if(rotationrate4z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(rotationrate4z_in);
            sig = convert_inputsignal_sensorsignal(name,rotationrate4z_in);
            rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        end
    end
    if(Plot_MagneticField == 1)
        if(mag1x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag1x_in);
            sig = convert_inputsignal_sensorsignal(name,mag1x_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag1y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag1y_in);
            sig = convert_inputsignal_sensorsignal(name,mag1y_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag1z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag1z_in);
            sig = convert_inputsignal_sensorsignal(name,mag1z_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag2x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag2x_in);
            sig = convert_inputsignal_sensorsignal(name,mag2x_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag2y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag2y_in);
            sig = convert_inputsignal_sensorsignal(name,mag2y_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag2z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag2z_in);
            sig = convert_inputsignal_sensorsignal(name,mag2z_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag3x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag3x_in);
            sig = convert_inputsignal_sensorsignal(name,mag3x_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag3y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag3y_in);
            sig = convert_inputsignal_sensorsignal(name,mag3y_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag3z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag3z_in);
            sig = convert_inputsignal_sensorsignal(name,mag3z_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag4x_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag4x_in);
            sig = convert_inputsignal_sensorsignal(name,mag4x_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag4y_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag4y_in);
            sig = convert_inputsignal_sensorsignal(name,mag4y_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
        if(mag4z_in.signals.values(1,SignalIndex.STATUS) ~= SignalState.SIGNALSTATE_UNDEFINED_)
            name = getVarName(mag4z_in);
            sig = convert_inputsignal_sensorsignal(name,mag4z_in);
            magnetic_field_signals{length(magnetic_field_signals)+1} = sig;
        end
    end
end
if(Plot_TimedSignals == 1)
    if(Plot_Acceleration == 1)
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TIMEDSIGNAL_,'xacc1',...
            out.timed_signals.accel.accel1.accel1x.value.Time, ...
            out.timed_signals.accel.accel1.accel1x.value.Data, ...
            out.timed_signals.accel.accel1.accel1x.status.Data, ...
            out.timed_signals.accel.accel1.accel1x.rms.Data);
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TIMEDSIGNAL_,'yacc1',...
            out.timed_signals.accel.accel1.accel1y.value.Time, ...
            out.timed_signals.accel.accel1.accel1y.value.Data, ...
            out.timed_signals.accel.accel1.accel1y.status.Data, ...
            out.timed_signals.accel.accel1.accel1y.rms.Data);
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TIMEDSIGNAL_,'zacc1',...
            out.timed_signals.accel.accel1.accel1z.value.Time, ...
            out.timed_signals.accel.accel1.accel1z.value.Data, ...
            out.timed_signals.accel.accel1.accel1z.status.Data, ...
            out.timed_signals.accel.accel1.accel1z.rms.Data);
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
    end    
end
if(Plot_TruthSignals == 1)
    if(Plot_Acceleration == 1)
         sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'xacc_truth',...
            truth_forwardaccel.time, ...
            truth_forwardaccel.signals.values(:,SignalIndex.VALUE),...
            truth_forwardaccel.signals.values(:,SignalIndex.STATUS),...
            truth_forwardaccel.signals.values(:,SignalIndex.RMS));
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'yacc_truth',...
            truth_lateralaccel.time, ...
            truth_lateralaccel.signals.values(:,SignalIndex.VALUE),...
            truth_lateralaccel.signals.values(:,SignalIndex.STATUS),...
            truth_lateralaccel.signals.values(:,SignalIndex.RMS));
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'zacc_truth',...
            truth_vertaccel.time, ...
            truth_vertaccel.signals.values(:,SignalIndex.VALUE),...
            truth_vertaccel.signals.values(:,SignalIndex.STATUS),...
            truth_vertaccel.signals.values(:,SignalIndex.RMS));
        linear_acc_signals{length(linear_acc_signals)+1} = sig;
    end
    if(Plot_AngularRate == 1)
         sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'xgyro_truth',...
            truth_rollrate.time, ...
            truth_rollrate.signals.values(:,SignalIndex.VALUE),...
            truth_rollrate.signals.values(:,SignalIndex.STATUS),...
            truth_rollrate.signals.values(:,SignalIndex.RMS));
        rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'ygyro_truth',...
            truth_pitchrate.time, ...
            truth_pitchrate.signals.values(:,SignalIndex.VALUE),...
            truth_pitchrate.signals.values(:,SignalIndex.STATUS),...
            truth_pitchrate.signals.values(:,SignalIndex.RMS));
        rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
        sig = convert_modelsignal(SignalClass.SIGNALCLASS_TRUTHSIGNAL_,'zgyro_truth',...
            truth_yawrate.time, ...
            truth_yawrate.signals.values(:,SignalIndex.VALUE),...
            truth_yawrate.signals.values(:,SignalIndex.STATUS),...
            truth_yawrate.signals.values(:,SignalIndex.RMS));
        rotation_rate_signals{length(rotation_rate_signals)+1} = sig;
    end
end
if(Plot_Acceleration == 1)
  fig_list = draw_linearacceleration_graphs(0,linear_acc_signals);
end
if(Plot_AngularRate == 1)
  fig_list = draw_rotationrate_graphs(0,rotation_rate_signals);
end
if(Plot_MagneticField == 1)
     fig_list = draw_magneticfield_graphs(0,magnetic_field_signals);
end
if(Plot_Orientation == 1)
  %fig_list = draw_orientation_graphs(0,orientation_signals);
end
