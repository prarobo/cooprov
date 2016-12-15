/* This node is used to transform the imu roll pitch and yaw messages coming 
 * from the arduino node into messages of type imu msg
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "imuArduino.h"

using namespace std;

// List of functions used
void imu_callback(const std_msgs::Float32MultiArray&);

//imu object
imuArduino imuObj;

//imu sensor messages
sensor_msgs::Imu imu_msg;

//imu messsage publisher
ros::Publisher openrov_imu_pub;

int main(int argc, char **argv){
  //init node
  ros::init(argc, argv, "imu_msg_generator");
  ros::NodeHandle n;

  // Create publisher
  openrov_imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 2);
  
  // Create subscribers
  ros::Subscriber openrov_imu_sub = n.subscribe("openrov_rpy", 1, imu_callback);
  
  //ros loop
  ros::spin();
  return 0;
}

void imu_callback(const std_msgs::Float32MultiArray& openrov_rpy)	{
  //set imu values
  imuObj.imu_set_val(openrov_rpy);
  imuObj.imu_msg_converter(imu_msg);
  
  openrov_imu_pub.publish(imu_msg);
}





