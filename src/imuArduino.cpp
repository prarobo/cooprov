/* IMU Arduino Class
* Author: Prasanna Kannappan
* Purpose: Imu class definition that operates on the imu data from the arduino and creates a imu ros sensor message
*/

#include "imuArduino.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

//Default covariances
float default_orientation_covariance[3][3] = {{1, 0, 0},
					       {0, 1, 0},
					       {0, 0, 1}};

float default_angular_velocity_covariance[3][3] = {{1, 0, 0},
						    {0, 1, 0},
						    {0, 0, 1}};
					    
float default_linear_acceleration_covariance[3][3] = {{1, 0, 0},
						       {0, 1, 0},
						       {0, 0, 1}};
					      

//Default constructor to intialize imu values
imuArduino::imuArduino()	{
  roll = 0;
  pitch = 0;
  yaw = 0;
  gyro_x = 0;
  gyro_y = 0;
  gyro_z = 0;
  accel_x = 0;
  accel_y = 0;
  accel_z = 0;
  imu_set_cov(true);
}

//Imu value set
void imuArduino::imu_set_val(const std_msgs::Float32MultiArray& openrov_rpy)	{
  roll = openrov_rpy.data[0];
  pitch = openrov_rpy.data[1];
  yaw = openrov_rpy.data[2];
  gyro_x = openrov_rpy.data[3];
  gyro_y = openrov_rpy.data[4];
  gyro_z = openrov_rpy.data[5];
  accel_x = openrov_rpy.data[6];
  accel_y = openrov_rpy.data[7];
  accel_z = openrov_rpy.data[8];
}

//Covariance input
void imuArduino::imu_set_cov(bool defaultValues)	{
  if(defaultValues)	{
    for(int i=0; i<3; i++)	{
      for(int j=0; j<3; j++)	{
	 orientation_covariance[i][j] = default_orientation_covariance[i][j];
	 angular_velocity_covariance[i][j] = default_angular_velocity_covariance[i][j];
	 linear_acceleration_covariance[i][j] = default_linear_acceleration_covariance[i][j];    
      }
    }
  }
}

//IMU Sensor message conversion  
void imuArduino::imu_msg_converter(sensor_msgs::Imu& imuMsg)	{
  
  //Rotation matrix initialized using euler angles
  tf::Matrix3x3 rotMat;
  tf::Quaternion quat;
  rotMat.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));
  rotMat.getRotation(quat);
    
  //Quarternion
  quaternionTFToMsg(quat, imuMsg.orientation);
  
  //Angular velocity
  imuMsg.angular_velocity.x = gyro_x;
  imuMsg.angular_velocity.y = gyro_y;
  imuMsg.angular_velocity.z = gyro_z;
  
  //Linear acceleration
  imuMsg.linear_acceleration.x = accel_x;
  imuMsg.linear_acceleration.y = accel_y;
  imuMsg.linear_acceleration.z = accel_z;
  
  //Covariance matrices
  for(int i=0; i<3; i++)	{
    for(int j=0; j<3; j++)	{
	imuMsg.orientation_covariance[i*3+j] = default_orientation_covariance[i][j];
	imuMsg.angular_velocity_covariance[i*3+j] = default_angular_velocity_covariance[i][j];
	imuMsg.linear_acceleration_covariance[i*3+j] = default_linear_acceleration_covariance[i][j];    
    }
  }  
}
