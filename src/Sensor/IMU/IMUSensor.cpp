/*
 * IMUSensor.cpp
 *
 */

#include "IMUSensor.h"

IMUSensor::IMUSensor() {
	// TODO Auto-generated constructor stub

}

IMUSensor::~IMUSensor() {
	// TODO Auto-generated destructor stub
}
bool IMUSensor::init(std::string t_name)
{
	name = t_name;
	time_imu_updated = -1.0;
	time_imumag_updated = -1.0;
	update_count = 0;
	imu_data.sequence_number = 0;
	imu_data.xacc.type = SIGNALTYPE_ACCELERATION;
	imu_data.xacc.status = SIGNALSTATE_INITIALIZING;
	imu_data.yacc.type = SIGNALTYPE_ACCELERATION;
	imu_data.yacc.status = SIGNALSTATE_INITIALIZING;
	imu_data.zacc.type = SIGNALTYPE_ACCELERATION;
	imu_data.zacc.status = SIGNALSTATE_INITIALIZING;

	imu_data.xgyro.type = SIGNALTYPE_ROTATION_RATE;
	imu_data.xgyro.status = SIGNALSTATE_INITIALIZING;
	imu_data.ygyro.type = SIGNALTYPE_ROTATION_RATE;
	imu_data.ygyro.status = SIGNALSTATE_INITIALIZING;
	imu_data.zgyro.type = SIGNALTYPE_ROTATION_RATE;
	imu_data.zgyro.status = SIGNALSTATE_INITIALIZING;

	imu_data.xmag.type = SIGNALTYPE_MAGNETIC_FIELD;
	imu_data.xmag.status = SIGNALSTATE_INITIALIZING;
	imu_data.ymag.type = SIGNALTYPE_MAGNETIC_FIELD;
	imu_data.ymag.status = SIGNALSTATE_INITIALIZING;
	imu_data.zmag.type = SIGNALTYPE_MAGNETIC_FIELD;
	imu_data.zmag.status = SIGNALSTATE_INITIALIZING;
	
	return true;
}
void IMUSensor::set_pose(gazebo::math::Pose pose)
{
	sensor_pose = pose;
	double roll = pose.rot.GetRoll();
	double pitch = pose.rot.GetPitch();
	double yaw = pose.rot.GetYaw();
	printf("IMU: %s Roll: %f Pitch: %f Yaw: %f\n",
		name.c_str(),roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
	rotate_matrix = generate_rotation_matrix(roll,pitch,yaw);
}
eros::imu IMUSensor::update_IMU(double current_time,double last_imu_update,double last_mag_update,
	ignition::math::Vector3d linear_acc,
	ignition::math::Vector3d angular_rate,
	ignition::math::Vector3d magnetic_field)
{
	imu_data.tov = current_time;
	{
		double time_updated = last_imu_update;
		if(time_updated > time_imu_updated)
		{
			time_imu_updated = time_updated;
			imu_data.xacc.status = SIGNALSTATE_UPDATED;
			imu_data.yacc.status = SIGNALSTATE_UPDATED;
			imu_data.zacc.status = SIGNALSTATE_UPDATED;
			imu_data.xgyro.status = SIGNALSTATE_UPDATED;
			imu_data.ygyro.status = SIGNALSTATE_UPDATED;
			imu_data.zgyro.status = SIGNALSTATE_UPDATED;
			//imu_data.xacc.value = linear_acc.X();
			//imu_data.yacc.value = linear_acc.Y();
			//imu_data.zacc.value = linear_acc.Z();
			{
				Eigen::RowVector3f v;
				v << linear_acc.X(),linear_acc.Y(),linear_acc.Z();
				Eigen::RowVector3f vp = rotate_matrix.Rotation_Acc*v.transpose();
				imu_data.xacc.value = vp(0);
				imu_data.yacc.value = vp(1);
				imu_data.zacc.value = vp(2);
				if(update_count == 0)
				{
					xacc_rms_mean1 = pow(imu_data.xacc.value,2.0);
				}
				else
				{
					xacc_rms_mean1 += (pow(imu_data.xacc.value,2.0)/(double)((update_count+1))) - (xacc_rms_mean1/(double)(update_count+1));
				}
				imu_data.xacc.rms = pow(xacc_rms_mean1,0.5);

				if(update_count == 0)
				{
					yacc_rms_mean1 = pow(imu_data.yacc.value,2.0);
				}
				else
				{
					yacc_rms_mean1 += (pow(imu_data.yacc.value,2.0)/(double)((update_count+1))) - (yacc_rms_mean1/(double)(update_count+1));
				}
				imu_data.yacc.rms = pow(yacc_rms_mean1,0.5);

				if(update_count == 0)
				{
					zacc_rms_mean1 = pow(imu_data.zacc.value,2.0);
				}
				else
				{
					zacc_rms_mean1 += (pow(imu_data.zacc.value,2.0)/(double)((update_count+1))) - (zacc_rms_mean1/(double)(update_count+1));
				}
				imu_data.zacc.rms = pow(zacc_rms_mean1,0.5);
			}

			//imu_data.xgyro.value = angular_rate.X()*180.0/M_PI;
			//imu_data.ygyro.value = angular_rate.Y()*180.0/M_PI;
			//imu_data.zgyro.value = angular_rate.Z()*180.0/M_PI;
			{
				Eigen::RowVector3f v;
				v << angular_rate.X()*180.0/M_PI,angular_rate.Y()*180.0/M_PI,angular_rate.Z()*180.0/M_PI;
				Eigen::RowVector3f vp = rotate_matrix.Rotation_Gyro*v.transpose();
				imu_data.xgyro.value = vp(0);
				imu_data.ygyro.value = vp(1);
				imu_data.zgyro.value = vp(2);
				if(update_count == 0)
				{
					xgyro_rms_mean1 = pow(imu_data.xgyro.value,2.0);
				}
				else
				{
					xgyro_rms_mean1 += (pow(imu_data.xgyro.value,2.0)/(double)((update_count+1))) - (xgyro_rms_mean1/(double)(update_count+1));
				}
				imu_data.xgyro.rms = pow(xgyro_rms_mean1,0.5);

				if(update_count == 0)
				{
					ygyro_rms_mean1 = pow(imu_data.ygyro.value,2.0);
				}
				else
				{
					ygyro_rms_mean1 += (pow(imu_data.ygyro.value,2.0)/(double)((update_count+1))) - (ygyro_rms_mean1/(double)(update_count+1));
				}
				imu_data.ygyro.rms = pow(ygyro_rms_mean1,0.5);

				if(update_count == 0)
				{
					zgyro_rms_mean1 = pow(imu_data.zgyro.value,2.0);
				}
				else
				{
					zgyro_rms_mean1 += (pow(imu_data.zgyro.value,2.0)/(double)((update_count+1))) - (zgyro_rms_mean1/(double)(update_count+1));
				}
				imu_data.zgyro.rms = pow(zgyro_rms_mean1,0.5);
			}
		}
		else
		{
			imu_data.xacc.status = SIGNALSTATE_HOLD;
			imu_data.yacc.status = SIGNALSTATE_HOLD;
			imu_data.zacc.status = SIGNALSTATE_HOLD;
			imu_data.xgyro.status = SIGNALSTATE_HOLD;
			imu_data.ygyro.status = SIGNALSTATE_HOLD;
			imu_data.zgyro.status = SIGNALSTATE_HOLD;
		}	
	}
	{
		double time_updated = last_mag_update;
		if(time_updated > time_imumag_updated)
		{
			time_imumag_updated = time_updated;
			imu_data.xmag.status = SIGNALSTATE_UPDATED;
			imu_data.ymag.status = SIGNALSTATE_UPDATED;
			imu_data.zmag.status = SIGNALSTATE_UPDATED;
			//imu_data.xmag.value = magnetic_field.X();
			//imu_data.ymag.value = magnetic_field.Y();
			//imu_data.zmag.value = magnetic_field.Z();
			{
				Eigen::RowVector3f v;
				v << magnetic_field.X(),magnetic_field.Y(),magnetic_field.Z();
				Eigen::RowVector3f vp = rotate_matrix.Rotation_Mag*v.transpose();
				double abs_v = sqrt((vp(0)*vp(0))+(vp(1)*vp(1))+(vp(2)*vp(2)));
				if((abs_v > MAGNETOMETER_MAGNITUDE_UPPERBOUND) or 
				   (abs_v < MAGNETOMETER_MAGNITUDE_LOWERBOUND))
				{
					//char tempstr[512];
					//sprintf(tempstr,"Magnetometer absolute value: %4.4f, likely requires calibration.",abs_v);
					//printf("%s\n",tempstr);
					//diag = update_diagnostic(imus.at(i).devicename,SENSORS,NOTICE,DIAGNOSTIC_FAILED,std::string(tempstr));
				}
				imu_data.xmag.value = vp(0);
				imu_data.ymag.value = vp(1);
				imu_data.zmag.value = vp(2);

				if(update_count == 0)
				{
					xmag_rms_mean1 = pow(imu_data.xmag.value,2.0);
				}
				else
				{
					xmag_rms_mean1 += (pow(imu_data.xmag.value,2.0)/(double)((update_count+1))) - (xmag_rms_mean1/(double)(update_count+1));
				}
				imu_data.xmag.rms = pow(xmag_rms_mean1,0.5);

				if(update_count == 0)
				{
					ymag_rms_mean1 = pow(imu_data.ymag.value,2.0);
				}
				else
				{
					ymag_rms_mean1 += (pow(imu_data.ymag.value,2.0)/(double)((update_count+1))) - (ymag_rms_mean1/(double)(update_count+1));
				}
				imu_data.ymag.rms = pow(ymag_rms_mean1,0.5);

				if(update_count == 0)
				{
					zmag_rms_mean1 = pow(imu_data.zmag.value,2.0);
				}
				else
				{
					zmag_rms_mean1 += (pow(imu_data.zmag.value,2.0)/(double)((update_count+1))) - (zmag_rms_mean1/(double)(update_count+1));
				}
				imu_data.zmag.rms = pow(zmag_rms_mean1,0.5);
			}
		}
		else
		{
			imu_data.xmag.status = SIGNALSTATE_HOLD;
			imu_data.ymag.status = SIGNALSTATE_HOLD;
			imu_data.zmag.status = SIGNALSTATE_HOLD;
		}
		
	}
	update_count++;
	imu_data.sequence_number++;
	return imu_data;
}
IMUSensor::RotationMatrix IMUSensor::generate_rotation_matrix(double roll,double pitch,double yaw)
{


	RotationMatrix R;
	Eigen::Matrix3f X;
	X.row(0) << 1.0,0.0,0.0;
	X.row(1) << 0.0,cos(roll),sin(roll);
	X.row(2) << 0.0,-sin(roll),cos(roll);

	Eigen::Matrix3f Y;
	Y.row(0) << cos(pitch),0.0,sin(pitch);
	Y.row(1) << 0.0,1.0,0.0;
	Y.row(2) << -sin(pitch),0.0,cos(pitch);

	Eigen::Matrix3f Z;
	Z.row(0) << cos(-yaw),sin(-yaw),0.0;
	Z.row(1) << -sin(-yaw),cos(-yaw),0.0;
	Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Acc_X = X;
	R.Rotation_Gyro_X = X;
	R.Rotation_Mag_X = X;

	R.Rotation_Acc_Y = Y;
	R.Rotation_Gyro_Y = Y;
	R.Rotation_Mag_Y = Y;

	R.Rotation_Acc_Z = Z;
	R.Rotation_Gyro_Z = Z;
	R.Rotation_Mag_Z = Z;

	R.Rotation_Acc = R.Rotation_Acc_Z*R.Rotation_Acc_Y*R.Rotation_Acc_X;
	R.Rotation_Gyro = R.Rotation_Gyro_Z*R.Rotation_Gyro_Y*R.Rotation_Gyro_X;
	R.Rotation_Mag = R.Rotation_Mag_Z*R.Rotation_Mag_Y*R.Rotation_Mag_X;
	return R;
}