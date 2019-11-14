classdef OrientationLinker
   properties
    SignalClass;
    SignalType;
    SignalState;
    orientations;
    Pipeline_ComputationMethod = [];
    initialized;
   end
   methods
    function obj = OrientationLinker
      obj.initialized = 0;
      obj.Pipeline_ComputationMethod.LinearAcceleration = 1;
      obj.Pipeline_ComputationMethod.RotationRate = 2;
      obj.Pipeline_ComputationMethod.MagneticField = 3;
      
    end
    function obj = init(obj,SignalClass,SignalType,SignalState)
      obj.SignalClass = SignalClass;
      obj.SignalType = SignalType;
      obj.SignalState = SignalState;
    end
    function obj = new_input(obj,model_orientation,inputs) %linear_accelerations,rotation_rates,magnetic_fields
      if(obj.initialized == 0)
        pipeline_index = 1;
        for i = 1:length(inputs{obj.Pipeline_ComputationMethod.LinearAcceleration})
          x = Initialize_Signal;
          y = Initialize_Signal;
          z = Initialize_Signal;
          x.name = ['roll' num2str(obj.Pipeline_ComputationMethod.LinearAcceleration) '_' num2str(i)];
          x.type = obj.SignalType.SIGNALTYPE_ANGLE;
          x.status = obj.SignalState.SIGNALSTATE_INITIALIZING;
          x.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
          y.name = ['pitch' num2str(obj.Pipeline_ComputationMethod.LinearAcceleration) '_' num2str(i)];
          y.type = obj.SignalType.SIGNALTYPE_ANGLE;
          y.status = obj.SignalState.SIGNALSTATE_INITIALIZING;
          y.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
          z.name = ['yaw' num2str(obj.Pipeline_ComputationMethod.LinearAcceleration) '_' num2str(i)];
          z.type = obj.SignalType.SIGNALTYPE_ANGLE;
          z.status = obj.SignalState.SIGNALSTATE_INVALID; %Unable to compute 3d orientation from accel's
          z.class = obj.SignalClass.SIGNALCLASS_INPUTSIGNAL;
          obj.orientations{pipeline_index}.x = x;
          obj.orientations{pipeline_index}.y = y;
          obj.orientations{pipeline_index}.z = z;
          pipeline_index = pipeline_index+1;
        end
        obj.initialized = 1;
      end   
      obj = obj.update_inputs(model_orientation,inputs);   
    end
    function obj = update_inputs(obj,model_orientation,inputs)
      mu = 0.01;
      pipeline_index = 1;
      for i = 1:length(inputs{obj.Pipeline_ComputationMethod.LinearAcceleration})
        acc_x = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}(i).x.value;
        acc_y = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}(i).y.value;
        acc_z = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}(i).z.value;
        theta = atan2(-acc_x,(((acc_y*acc_y) + (acc_z*acc_z))^0.5));
        phi = atan2(acc_y,(sign(acc_z)*((acc_z*acc_z)+mu*(acc_x*acc_x))^0.5));
        obj.orientations{pipeline_index}.x.tov = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}.x.tov;
        obj.orientations{pipeline_index}.x.status = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}.x.status;
        obj.orientations{pipeline_index}.x.value = phi*180.0/pi;
        obj.orientations{pipeline_index}.y.tov = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}.y.tov;
        obj.orientations{pipeline_index}.y.status = inputs{obj.Pipeline_ComputationMethod.LinearAcceleration}.y.status;
        obj.orientations{pipeline_index}.y.value = theta*180.0/pi;
        pipeline_index = pipeline_index+1;
      end
    end
  end
end

    