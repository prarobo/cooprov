/* IMU Arduino Class
* Author: Prasanna Kannappan
* Purpose: Imu class declaration that operates on the imu data from the arduino and creates a imu ros sensor message
*/

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

using namespace std;

class imuArduino {
public:
  imuArduino();
  void imu_set_val(const std_msgs::Float32MultiArray&);
  void imu_set_cov(bool defaultValues);
  
  //IMU values
  double roll;
  double pitch;
  double yaw;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double accel_x;
  double accel_y;
  double accel_z;
  
  //Covariances
  double orientation_covariance[3][3];
  double angular_velocity_covariance[3][3];
  double linear_acceleration_covariance[3][3];
  
  //IMU Sensor message conversion  
  void imu_msg_converter(sensor_msgs::Imu& imuMsg);
};